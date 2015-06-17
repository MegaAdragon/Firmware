#include "BlockLocalPositionEstimatorMulti.hpp"
#include <mavlink/mavlink_log.h>
#include <fcntl.h>
#include <nuttx/math.h>
#include <systemlib/err.h>
#include <px4.h>

static const int 		MIN_FLOW_QUALITY = 100;
static const int 		REQ_INIT_COUNT = 100;

static const uint32_t 		VISION_POSITION_TIMEOUT = 500000;
static const uint32_t 		VISION_VELOCITY_TIMEOUT = 500000;
static const uint32_t 		VICON_TIMEOUT = 200000;	 

static const uint32_t 		XY_SRC_TIMEOUT = 2000000;

BlockLocalPositionEstimatorMulti::BlockLocalPositionEstimatorMulti() :
	// this block has no parent, and has name LPE
	SuperBlock(NULL,"LPE"),

	// test
	_nh(),

	// subscriptions
	_sub_att(_nh.subscribe("vehicle_attitude", 1,
		&BlockLocalPositionEstimatorMulti::handleAttitude, this)),
	_sub_flow(_nh.subscribe("optical_flow", 1,
		&BlockLocalPositionEstimatorMulti::handleOpticalFlow, this)),
	_sub_sensor(_nh.subscribe("sensor_combined", 1,
		&BlockLocalPositionEstimatorMulti::handleSensorCombined, this)),
	_sub_distance(_nh.subscribe("distance_sensor", 1,
		&BlockLocalPositionEstimatorMulti::handleDistanceSensor, this)),
	_sub_param_update(_nh.subscribe("param_update", 1,
		&BlockLocalPositionEstimatorMulti::handleParamUpdate, this)),
	_sub_home(_nh.subscribe("home_position", 1,
		&BlockLocalPositionEstimatorMulti::handleHome, this)),
	_sub_gps(_nh.subscribe("vehicle_gps_position", 1,
		&BlockLocalPositionEstimatorMulti::handleGPS, this)),
	_sub_vision_pos(_nh.subscribe("vision_position_estimate", 1,
		&BlockLocalPositionEstimatorMulti::handleVisionPosition, this)),
	_sub_vision_vel(_nh.subscribe("vision_velocity_estimate", 1,
		&BlockLocalPositionEstimatorMulti::handleVisionVelocity, this)),
	_sub_vicon(_nh.subscribe("vehicle_vicon_position", 1,
		&BlockLocalPositionEstimatorMulti::handleVicon, this)),

	// publications
	_pub_lpos(_nh.advertise<vehicle_local_position_s>(
		"vehicle_local_position", 1)),
	_pub_gpos(_nh.advertise<vehicle_global_position_s>(
		"vehicle_global_position", 1)),
	_pub_filtered_flow(_nh.advertise<filtered_bottom_flow_s>(
		"filtered_bottom_flow", 1)),

	// local message copies
	_att(),
	_home(),
	_gps(),
	_sensor_combined(),

	// map projection
	_map_ref(),

	// block parameters
	_integrate(this, "INTEGRATE"),
	_flow_xy_stddev(this, "FLW_XY"),
	_sonar_z_stddev(this, "SNR_Z"),
	_lidar_z_stddev(this, "LDR_Z"),
	_accel_xy_noise_power(this, "ACC_XY"),
	_accel_z_noise_power(this, "ACC_Z"),
	_baro_stddev(this, "BAR_Z"),
	_gps_xy_stddev(this, "GPS_XY"),
	_gps_z_stddev(this, "GPS_Z"),
	_gps_vxy_stddev(this, "GPS_VXY"),
	_gps_vz_stddev(this, "GPS_VZ"),
	_vision_xy_stddev(this, "VIS_XY"),
	_vision_z_stddev(this, "VIS_Z"),
	_vision_vxy_stddev(this, "VIS_VXY"),
	_vision_vz_stddev(this, "VIS_VZ"),
	_no_vision(this, "NO_VIS"),
	_beta_max(this, "BETA_MAX"),
	_vicon_p_stddev(this, "VIC_P"),
	_pn_p_noise_power(this, "PN_P"),
	_pn_v_noise_power(this, "PN_V"),

	// misc
	_polls(),
	_timeStamp(hrt_absolute_time()),
	_time_last_xy(0),
	_time_last_flow(0),
	_time_last_baro(0),
	_time_last_gps(0),
	_time_last_lidar(0),
	_time_last_sonar(0),
	_time_last_vision_p(0),
	_time_last_vision_v(0),	
	_time_last_vicon(0),
	_altHome(0),

	// mavlink log
	_mavlink_fd(open(MAVLINK_LOG_DEVICE, 0)),

	// initialization flags
	_baroInitialized(false),
	_gpsInitialized(false),
	_lidarInitialized(false),
	_sonarInitialized(false),
	_flowInitialized(false),
	_visionPosInitialized(false),
	_visionVelInitialized(false),
	_viconInitialized(false),

	// init counts
	_baroInitCount(0),
	_gpsInitCount(0),
	_lidarInitCount(0),
	_sonarInitCount(0),
	_flowInitCount(0),
	_visionPosInitCount(0),
	_visionVelInitCount(0),
	_viconInitCount(0),

	// reference altitudes
	_baroAltHome(0),
	_gpsAltHome(0),
	_lidarAltHome(0),
	_sonarAltHome(0),
	_visionHome(),
	_visionBaseVel(),
	_viconHome(),

	// flow integration
	_flowX(0),
	_flowY(0),
	_flowMeanQual(0),

	// reference lat/lon
	_gpsLatHome(0),
	_gpsLonHome(0),

	// faults
	_baroFault(0),
	_gpsFault(0),
	_lidarFault(0),
	_flowFault(0),
	_sonarFault(0),
	_visionPosFault(0),
	_visionVelFault(0),
	_viconFault(0),

	//timeouts
	_visionPosTimeout(true),
	_visionVelTimeout(true),
	_viconTimeout(true),

	// loop performance
	_loop_perf(),
	_interval_perf(),
	_err_perf(),

	// kf matrices
	_x(), _u(), _P()
{
	// setup event triggering based on new flow messages to integrate
	//_polls[POLL_FLOW].fd = _sub_flow.getHandle();
	//_polls[POLL_FLOW].events = POLLIN;

	//_polls[POLL_PARAM].fd = _sub_param_update.getHandle();
	//_polls[POLL_PARAM].events = POLLIN;

	//_polls[POLL_SENSORS].fd = _sub_sensor.getHandle();
	//_polls[POLL_SENSORS].events = POLLIN;

	// initialize P to identity*0.1
	_P.identity();
	_P *= 0.1;

	// perf counters
	_loop_perf = perf_alloc(PC_ELAPSED,
			"local_position_estimator_runtime");
	_interval_perf = perf_alloc(PC_INTERVAL,
			"local_position_estimator_interval");
	_err_perf = perf_alloc(PC_COUNT, "local_position_estimator_err");

	// map
	_map_ref.init_done = false;

	// intialize parameter dependent matrices
	updateParams();
}

BlockLocalPositionEstimatorMulti::~BlockLocalPositionEstimatorMulti() {
}

void BlockLocalPositionEstimatorMulti::update() {

	// wait for a sensor update, check for exit condition every 100 ms
	int ret = poll(_polls, 2, 100);
	if (ret < 0) {
		/* poll error, count it in perf */
		perf_count(_err_perf);
		return;
	}

	uint64_t newTimeStamp = hrt_absolute_time();
	float dt = (newTimeStamp - _timeStamp) / 1.0e6f;
	_timeStamp = newTimeStamp;

	// check for sane values of dt
	// to prevent large control responses
	if (dt > 1.0f || dt < 0)
		return;

	// set dt for all child blocks
	setDt(dt);

	// see which updates are available
	//bool paramsUpdated = _sub_param_update.updated();

	// get new data
	//updateSubscriptions();

	// check for timeouts on external sources
	if((hrt_absolute_time() - _time_last_vision_p > VISION_POSITION_TIMEOUT) && _visionPosInitialized)
	{
		if(!_visionPosTimeout)
		{
			_visionPosTimeout = true;
			mavlink_log_info(_mavlink_fd, "[lpe] vision position timeout ");
			warnx("[lpe] vision position timeout ");
		}
	} else
		_visionPosTimeout = false;

	if((hrt_absolute_time() - _time_last_vision_v > VISION_VELOCITY_TIMEOUT) && _visionVelInitialized)
	{
		if(!_visionVelTimeout)
		{
			_visionVelTimeout = true;
			mavlink_log_info(_mavlink_fd, "[lpe] vision velocity timeout ");
			warnx("[lpe] vision velocity timeout ");
		}
	} else
		_visionVelTimeout = false;

	if((hrt_absolute_time() -_time_last_vicon > VICON_TIMEOUT) && _viconInitialized)
	{
		if(!_viconTimeout)
		{
			_viconTimeout = true;
			mavlink_log_info(_mavlink_fd, "[lpe] vicon timeout ");
			warnx("[lpe] vicon timeout ");
		}
	} else {
		_viconTimeout = false;
	}

	// determine if we should start estimating
	bool canEstimateZ = _baroInitialized;	
	bool canEstimateXY = 
		(_gpsInitialized && !_gpsFault) ||
		 (_flowInitialized && !_flowFault) ||
		 (_visionPosInitialized && !_visionPosTimeout && !_visionPosFault) ||
		 (_visionVelInitialized && !_visionVelTimeout && !_visionVelFault) ||
		 (_viconInitialized && !_viconTimeout && !_viconFault);

	if(canEstimateXY) {
		_time_last_xy = hrt_absolute_time();	
	}
		
	// if we have no lat, lon initialized projection at 0,0
	if (canEstimateXY && !_map_ref.init_done) {
		map_projection_init(&_map_ref, 0, 0);
	}

	// reinitialize x if necessary
	bool reinit_x = false;
	for (int i=0; i< n_x; i++) {
		// should we do a reinit
		// of sensors here?
		// don't want it to take too long
		if (!isfinite(_x(i))) {
			reinit_x = true;
			break;
		}
	}
	if (reinit_x) {
		for (int i=0; i< n_x; i++) {
			_x(i) = 0;
		}
		mavlink_log_info(_mavlink_fd, "[lpe] reinit x");
		warnx("[lpe] reinit x");
	}

	// reinitialize P if necessary
	bool reinit_P = false;
	for (int i=0; i< n_x; i++) {
		for (int j=0; j< n_x; j++) {
			if (!isfinite(_P(i,j))) {
				reinit_P = true;
				break;
			}
		}
		if (reinit_P) break;
	}
	if (reinit_P) {
		mavlink_log_info(_mavlink_fd, "[lpe] reinit P");
		warnx("[lpe] reinit P");
		_P.identity();
		_P *= 0.1;
	}

	// do prediction
	predict(canEstimateXY, canEstimateZ);

	bool est_timeout = (hrt_absolute_time() - _time_last_xy > XY_SRC_TIMEOUT);
	if (!est_timeout) {
		// update all publications if possible
		publishLocalPos(true, true);
		publishGlobalPos(!canEstimateXY && !est_timeout);
	} else {
		// publish only Z estimate
		publishLocalPos(true, false);
	}
}

void BlockLocalPositionEstimatorMulti::initBaro(const sensor_combined_s & msg) {
	// collect baro data
	if (!_baroInitialized &&
		(msg.baro_timestamp != _time_last_baro)) {
		_time_last_baro = msg.baro_timestamp;
		_baroAltHome += msg.baro_alt_meter;
		if (_baroInitCount++ > REQ_INIT_COUNT) {
			_baroAltHome /= _baroInitCount;
			mavlink_log_info(_mavlink_fd,
				"[lpe] baro offs: %d m", (int)_baroAltHome);
			warnx("[lpe] baro offs: %d m", (int)_baroAltHome);
			_baroInitialized = true;
		}
	}
}

void BlockLocalPositionEstimatorMulti::initGps(const vehicle_gps_position_s & msg) {
	// collect gps data
	if (!_gpsInitialized && msg.fix_type > 2) {
		double lat = msg.lat*1e-7;
		double lon = msg.lon*1e-7;
		float alt = msg.alt*1e-3f;
		// increament sums for mean
		_gpsLatHome += lat;
		_gpsLonHome += lon;
		_gpsAltHome += alt;
		_time_last_gps = msg.timestamp_position;
		if (_gpsInitCount++ > REQ_INIT_COUNT) {
			_gpsLatHome /= _gpsInitCount;
			_gpsLonHome /= _gpsInitCount;
			_gpsAltHome /= _gpsInitCount;
			map_projection_init(&_map_ref, lat, lon);
			mavlink_log_info(_mavlink_fd, "[lpe] gps init: "
					"lat %d, lon %d, alt %d m",
					int(_gpsLatHome), int(_gpsLonHome), int(_gpsAltHome));
			warnx("[lpe] gps init: lat %d, lon %d, alt %d m",
					int(_gpsLatHome), int(_gpsLonHome), int(_gpsAltHome));
			_gpsInitialized = true;
		}
	}
}

void BlockLocalPositionEstimatorMulti::initLidar(const distance_sensor_s & msg) {
	
	if (msg.type != distance_sensor_s::MAV_DISTANCE_SENSOR_LASER) return;
	
	// collect lidar data
	bool valid = false;
	float d = msg.current_distance;
	if (d < msg.max_distance &&
		d > msg.min_distance)
		valid = true;

	if (!_lidarInitialized && valid) {
		// increament sums for mean
		_lidarAltHome += msg.current_distance;
		if (_lidarInitCount++ > REQ_INIT_COUNT) {
			_lidarAltHome /= _lidarInitCount;
			mavlink_log_info(_mavlink_fd, "[lpe] lidar init: "
					"alt %d cm",
					int(100*_lidarAltHome));
			warnx("[lpe] lidar init: alt %d cm",
					int(100*_lidarAltHome));
			_lidarInitialized = true;
		}
	}
}

void BlockLocalPositionEstimatorMulti::initSonar(const distance_sensor_s & msg) {

	if(msg.type != distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND) return;

	// collect sonar data
	bool valid = false;
	float d = msg.current_distance;
	if (d < msg.max_distance &&
		d > msg.min_distance)
		valid = true;

	if (!_sonarInitialized && valid) {
		// increament sums for mean
		_sonarAltHome += msg.current_distance;
		if (_sonarInitCount++ > REQ_INIT_COUNT) {
			_sonarAltHome /= _sonarInitCount;
			mavlink_log_info(_mavlink_fd, "[lpe] sonar init: "
					"alt %d cm",
					int(100*_sonarAltHome));
			warnx("[lpe] sonar init: alt %d cm",
					int(100*_sonarAltHome));
			_sonarInitialized = true;
		}
	}
}

void BlockLocalPositionEstimatorMulti::initFlow(const optical_flow_s & msg) {
	
	// collect pixel flow data 
	if (!_flowInitialized) {
		// increament sums for mean
		_flowMeanQual += msg.quality;

		if (_flowInitCount++ > REQ_INIT_COUNT) {
			_flowMeanQual /= _flowInitCount;
			if(_flowMeanQual < MIN_FLOW_QUALITY)	
			{
				// retry initialisation till we have better flow data
				warnx("[lpe] flow quality bad, retrying init : %d",
					int(_flowMeanQual));
				_flowMeanQual = 0;
				_flowInitCount = 0;
				return;
			}
			mavlink_log_info(_mavlink_fd, "[lpe] flow init: "
					"quality %d",
					int(_flowMeanQual));
			warnx("[lpe] flow init: quality %d",
					int(_flowMeanQual));
			_flowInitialized = true;
		}
	}
}

void BlockLocalPositionEstimatorMulti::initVisionPos(const vision_position_estimate_s & msg) {
	// collect vision position data
	if (!_visionPosInitialized) {
		// increament sums for mean
		math::Vector<3> pos;
		pos(0) = msg.x;
		pos(1) = msg.y;
		pos(2) = msg.z;
		_visionHome += pos;
		if (_visionPosInitCount++ > REQ_INIT_COUNT) {
			_visionHome /= _visionPosInitCount;
			mavlink_log_info(_mavlink_fd, "[lpe] vision position init: "
					"%f, %f, %f m", double(pos(0)), double(pos(1)), double(pos(2)));
			warnx("[lpe] vision position init: "
					"%f, %f, %f m", double(pos(0)), double(pos(1)), double(pos(2)));
			_visionPosInitialized = true;
		}
	}
}

void BlockLocalPositionEstimatorMulti::initVisionVel(const vision_speed_estimate_s & msg) {
	// collect vision velocity data
	if (!_visionVelInitialized) {
		// increament sums for mean
		math::Vector<3> vel;
		vel(0) = msg.x;
		vel(1) = msg.y;
		vel(2) = msg.z;
		_visionBaseVel += vel;
		if (_visionVelInitCount++ > REQ_INIT_COUNT) {
			_visionBaseVel /= _visionVelInitCount;
			mavlink_log_info(_mavlink_fd, "[lpe] vision velocity init: "
					"%f, %f, %f m/s", double(vel(0)), double(vel(1)), double(vel(2)));
			warnx("[lpe] vision velocity init: "
					"%f, %f, %f m/s", double(vel(0)), double(vel(1)), double(vel(2)));
			_visionVelInitialized = true;
		}
	}
}

void BlockLocalPositionEstimatorMulti::initVicon(const vehicle_vicon_position_s & msg) {
	// collect vicon data
	if (!_viconInitialized) {
		// increament sums for mean
		math::Vector<3> pos;
		pos(0) = msg.x;
		pos(1) = msg.y;
		pos(2) = msg.z;
		_viconHome += pos;
		if (_viconInitCount++ > REQ_INIT_COUNT) {
			_viconHome /= _viconInitCount;
			mavlink_log_info(_mavlink_fd, "[lpe] vicon init: "
					"%f, %f, %f m", double(pos(0)), double(pos(1)), double(pos(2)));
			warnx("[lpe] vicon init: "
					"%f, %f, %f m", double(pos(0)), double(pos(1)), double(pos(2)));
			_viconInitialized = true;
		}
	}
}

void BlockLocalPositionEstimatorMulti::publishLocalPos(bool z_valid, bool xy_valid) {
	// publish local position
	if (isfinite(_x(X_x)) && isfinite(_x(X_y)) && isfinite(_x(X_z)) &&
		isfinite(_x(X_vx)) && isfinite(_x(X_vy))
		&& isfinite(_x(X_vz))) {
		vehicle_local_position_s msg;
		msg.timestamp = _timeStamp;
		msg.xy_valid = xy_valid;
		msg.z_valid = z_valid;
		msg.v_xy_valid = xy_valid;
		msg.v_z_valid = z_valid;
		msg.x = _x(X_x); // north
		msg.y = _x(X_y); // east
		msg.z = _x(X_z); // down
		msg.vx = _x(X_vx); // north
		msg.vy = _x(X_vy); // east
		msg.vz = _x(X_vz); // down
		msg.yaw = _att.yaw;
		msg.xy_global = _home.timestamp != 0; // need home for reference
		msg.z_global = _baroInitialized;
		msg.ref_timestamp = _home.timestamp;
		msg.ref_lat = _map_ref.lat_rad*180/M_PI;
		msg.ref_lon = _map_ref.lon_rad*180/M_PI;
		msg.ref_alt = _home.alt;
		// TODO, terrain alt
		msg.dist_bottom = -_x(X_z);
		msg.dist_bottom_rate = -_x(X_vz);
		msg.surface_bottom_timestamp = 0;
		msg.dist_bottom_valid = true;
		msg.eph = sqrtf(_P(X_x, X_x) + _P(X_y, X_y));
		msg.epv = sqrtf(_P(X_z, X_z));
		_pub_lpos.publish(msg);
	}
}

void BlockLocalPositionEstimatorMulti::publishGlobalPos(bool dead_reckoning) {
	// publish global position
	double lat = 0;
	double lon = 0;
	map_projection_reproject(&_map_ref, _x(X_x), _x(X_y), &lat, &lon);
	float alt = -_x(X_z) + _home.alt;
	if(isfinite(lat) && isfinite(lon) && isfinite(alt) &&
			isfinite(_x(X_vx)) && isfinite(_x(X_vy)) &&
			isfinite(_x(X_vz))) {
		vehicle_global_position_s msg;
		msg.timestamp = _timeStamp;
		msg.time_utc_usec = _gps.time_utc_usec;
		msg.lat = lat;
		msg.lon = lon;
		msg.alt = alt;
		msg.vel_n = _x(X_vx);
		msg.vel_e = _x(X_vy);
		msg.vel_d = _x(X_vz);
		msg.yaw = _att.yaw;
		msg.eph = sqrtf(_P(X_x, X_x) + _P(X_y, X_y));
		msg.epv = sqrtf(_P(X_z, X_z));
		msg.terrain_alt = 0;
		msg.terrain_alt_valid = false;
		msg.dead_reckoning = dead_reckoning;
		_pub_gpos.publish(msg);
	}
}

void BlockLocalPositionEstimatorMulti::predict(bool canEstimateXY,
		bool canEstimateZ) {
	// if can't update anything, don't propagate
	// state or covariance
	if (!canEstimateXY && !canEstimateZ) return;

	if (_integrate.get() && _att.R_valid) {
		math::Matrix<3,3> R_att(_att.R);
		math::Vector<3> a(_sensor_combined.accelerometer_m_s2);
		_u = R_att*a;
		_u(2) += 9.81f; // add g
	} else
		_u = math::Vector<3>({0,0,0});

	// dynamics matrix
	math::Matrix<n_x, n_x>  A; // state dynamics matrix
	// derivative of position is velocity
	A(X_x, X_vx) = 1;
	A(X_y, X_vy) = 1;
	A(X_z, X_vz) = 1;
	// derivative of velocity is accelerometer
	// 	bias + acceleration
	//_A(X_vx, X_bx) = 1;
	//_A(X_vy, X_by) = 1;
	//_A(X_vz, X_bz) = 1;

	// input matrix
	math::Matrix<n_x, n_u>  B; // input matrix
	B(X_vx, U_ax) = 1;
	B(X_vy, U_ay) = 1;
	B(X_vz, U_az) = 1;

	// input noise covariance matrix
	math::Matrix<n_u, n_u> R;
	R(U_ax, U_ax) = _accel_xy_noise_power.get();
	R(U_ay, U_ay) = _accel_xy_noise_power.get();
	R(U_az, U_az) = _accel_z_noise_power.get();

	// process noise covariance matrix
	math::Matrix<n_x, n_x>  Q;
	Q(X_x, X_x) = _pn_p_noise_power.get();
	Q(X_y, X_y) = _pn_p_noise_power.get();
	Q(X_z, X_z) = _pn_p_noise_power.get();
	Q(X_vx, X_vx) = _pn_v_noise_power.get();
	Q(X_vy, X_vy) = _pn_v_noise_power.get();
	Q(X_vz, X_vz) = _pn_v_noise_power.get();

	// continuous time kalman filter prediction
	math::Vector<n_x>  dx = (A*_x + B*_u)*getDt();

	// only predict for components we have
	// valid measurements for
	if (!canEstimateXY) {
		dx(X_x) = 0;
		dx(X_y) = 0;
		dx(X_vx) = 0;
		dx(X_vy) = 0;
	}

	if (!canEstimateZ) {
		dx(X_z) = 0;
		dx(X_vz) = 0;
	}

	// propagate
	_x += dx;
	_P += (A*_P + _P*A.transposed() +
		B*R*B.transposed() + Q)*getDt();

}

void BlockLocalPositionEstimatorMulti::correctFlow(const optical_flow_s & msg) {	

	// flow measurement matrix and noise matrix
	math::Matrix<n_y_flow, n_x> C;
	C(Y_flow_x, X_x) = 1;
	C(Y_flow_y, X_y) = 1;

	math::Matrix<n_y_flow, n_y_flow> R;
	R(Y_flow_x, Y_flow_x) =
		_flow_xy_stddev.get()*_flow_xy_stddev.get();
	R(Y_flow_y, Y_flow_y) =
		_flow_xy_stddev.get()*_flow_xy_stddev.get();

	float flow_speed[3] = {0.0f, 0.0f, 0.0f};
	float global_speed[3] = {0.0f, 0.0f, 0.0f};

	// calc dt between flow timestamps
	// ignore first flow msg
	if (_time_last_flow == 0) {
		_time_last_flow = msg.timestamp;
		return;
	}
	float dt = (msg.timestamp - _time_last_flow) * 1.0e-6f ;
	_time_last_flow = msg.timestamp;
	
	// calculate velocity over ground
	if (msg.integration_timespan > 0) {
		flow_speed[0] = (msg.pixel_flow_x_integral /
			(msg.integration_timespan / 1e6f) - 
			_att.pitchspeed) *		// Body rotation correction TODO check this
			_x(X_z);
		flow_speed[1] = (msg.pixel_flow_y_integral /
			(msg.integration_timespan / 1e6f) -
			_att.rollspeed) *		// Body rotation correction
			_x(X_z);
	} else {
		flow_speed[0] = 0;
		flow_speed[1] = 0;
	}
	flow_speed[2] = 0.0f;

	// publish filtered flow
	filtered_bottom_flow_s flow_msg;
	flow_msg.sumx += flow_speed[0] * dt;
	flow_msg.sumy += flow_speed[1] * dt;
	flow_msg.vx = flow_speed[0];
	flow_msg.vy = flow_speed[1];
	_pub_filtered_flow.publish(flow_msg);

	// TODO add yaw rotation correction (with distance to vehicle zero)

	// convert to globalframe velocity
	for(uint8_t i = 0; i < 3; i++) {
		float sum = 0.0f;
		for(uint8_t j = 0; j < 3; j++) {
			sum += flow_speed[j] * PX4_R(_att.R, i, j);
		}
		global_speed[i] = sum;
	}

	// flow integral
	_flowX += global_speed[0]*dt;
	_flowY += global_speed[1]*dt;

	// measurement
	math::Vector<2> y;
	y(0) = _flowX;
	y(1) = _flowY;

	// residual
	math::Vector<2> r = y - C*_x;

	// residual covariance, (inversed)
	math::Matrix<n_y_flow, n_y_flow> S_I =
		(C*_P*C.transposed() + R).inversed();

	// fault detection
	float beta = sqrtf(r*(S_I*r));

	if (msg.quality < MIN_FLOW_QUALITY) {
		if (!_flowFault) {
			mavlink_log_info(_mavlink_fd, "[lpe] bad flow data ");
			warnx("[lpe] bad flow data ");
		}
		_flowFault = 2;
	// 3 std devations away
	} else if (beta > _beta_max.get()) {
		if (!_flowFault) {
			mavlink_log_info(_mavlink_fd, "[lpe] flow fault,  beta %5.2f", double(beta));
			warnx("[lpe] flow fault,  beta %5.2f", double(beta));
		}
		_flowFault = 1;
	// turn off if fault ok
	} else if (_flowFault) {
		_flowFault = 0;
		mavlink_log_info(_mavlink_fd, "[lpe] flow OK");
		warnx("[lpe] flow OK");
	}

	// kalman filter correction if no fault
	if (_flowFault < 1) {
		math::Matrix<n_x, n_y_flow> K =
			_P*C.transposed()*S_I;
		_x += K*r;
		_P -= K*C*_P;
	// reset flow integral to current estimate of position
	// if a fault occurred
	} else {
		_flowX = _x(X_x);
		_flowY = _x(X_y);	
	}

}

void BlockLocalPositionEstimatorMulti::correctSonar(const distance_sensor_s & msg) {
	
	if(msg.type != distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND)
		return;
	
	float d = msg.current_distance;

	// sonar measurement matrix and noise matrix
	math::Matrix<n_y_sonar, n_x> C;
	C(Y_sonar_z, X_z) = -1;

	// use parameter covariance unless sensor provides reasonable value
	math::Matrix<n_y_sonar, n_y_sonar> R;
	float cov = msg.covariance;
	if (cov < 1.0e-3f)
		R(0,0) = _sonar_z_stddev.get()*_sonar_z_stddev.get();
	else
		R(0,0) = cov;

	// measurement
	math::Vector<1> y;
	y(0) = (d - _sonarAltHome)*
		cosf(_att.roll)*
		cosf(_att.pitch);

	// residual
	math::Vector<1> r = y - C*_x;

	// residual covariance, (inversed)
	math::Matrix<n_y_sonar, n_y_sonar> S_I =
		(C*_P*C.transposed() + R).inversed();

	// fault detection
	float beta = sqrtf(r*(S_I*r));

	if (d < msg.min_distance ||
	    d > msg.max_distance) {
		if (!_sonarFault) {
			mavlink_log_info(_mavlink_fd, "[lpe] sonar out of range");
			warnx("[lpe] sonar out of range");
		}
		_sonarFault = 2;	
	// 3 std devations away
	} else if (beta > _beta_max.get()) {
		if (!_sonarFault) {
			mavlink_log_info(_mavlink_fd, "[lpe] sonar fault,  beta %5.2f", double(beta));
			warnx("[lpe] sonar fault,  beta %5.2f", double(beta));
		}
		_sonarFault = 1;
	// turn off if fault ok
	} else if (_sonarFault) {
		_sonarFault = 0;
		mavlink_log_info(_mavlink_fd, "[lpe] sonar OK");
		warnx("[lpe] sonar OK");
	}

	// kalman filter correction if no fault
	if (_sonarFault < 1) {
		math::Matrix<n_x, n_y_sonar> K =
			_P*C.transposed()*S_I;
		_x += K*r;
		_P -= K*C*_P;
	}
	_time_last_sonar = msg.timestamp;	
	
}

void BlockLocalPositionEstimatorMulti::correctBaro(const sensor_combined_s & msg) {

	math::Vector<1> y;
	y(0) = msg.baro_alt_meter - _baroAltHome;

	// baro measurement matrix
	math::Matrix<n_y_baro, n_x> C;
	C(Y_baro_z, X_z) = -1; // measured altitude, negative down dir.

	math::Matrix<n_y_baro, n_y_baro> R;
	R(0,0) = _baro_stddev.get()*_baro_stddev.get();

	// residual
	math::Matrix<1,1> S_I =
		((C*_P*C.transposed()) + R).inversed();
	math::Vector<1> r = y- (C*_x);

	// fault detection
	float beta = sqrtf(r*(S_I*r));
	if (beta > _beta_max.get()) {
		if (!_baroFault) {
			mavlink_log_info(_mavlink_fd, "[lpe] baro fault, beta %5.2f", double(beta));
			warnx("[lpe] baro fault, beta %5.2f", double(beta));
		}
		_baroFault = 1;
		// lower baro trust
		S_I = ((C*_P*C.transposed()) + R*10).inversed();
	} else if (_baroFault) {
		_baroFault = 0;
		mavlink_log_info(_mavlink_fd, "[lpe] baro OK");
		warnx("[lpe] baro OK");
	}

	// kalman filter correction if no hard fault
	// always trust baro a little
	if (_baroFault < 2) {
		math::Matrix<n_x, n_y_baro> K = _P*C.transposed()*S_I;
		_x = _x + K*r;
		_P -= K*C*_P;
	}
	_time_last_baro = msg.baro_timestamp;
}

void BlockLocalPositionEstimatorMulti::correctLidar(const distance_sensor_s & msg) {

	if(msg.type != distance_sensor_s::MAV_DISTANCE_SENSOR_LASER)
		return;	

	float d = msg.current_distance;
	
	math::Matrix<n_y_lidar, n_x> C;
	C(Y_lidar_z, X_z) = -1; // measured altitude,
				 // negative down dir.

	// use parameter covariance unless sensor provides reasonable value
	math::Matrix<n_y_lidar, n_y_lidar> R;
	float cov = msg.covariance;
	if (cov < 1.0e-3f)
		R(0,0) = _lidar_z_stddev.get()*_lidar_z_stddev.get();
	else
		R(0,0) = cov;

	math::Vector<1> y;
	y(0) = (d - _lidarAltHome)*
		cosf(_att.roll)*
		cosf(_att.pitch);

	// residual
	math::Matrix<1,1> S_I = ((C*_P*C.transposed()) + R).inversed();
	math::Vector<1> r = y - C*_x;

	// fault detection
	float beta = sqrtf(r*(S_I*r));

	// zero is an error code for the lidar
	if (d < msg.min_distance ||
	    d > msg.max_distance) {
		if (!_lidarFault) {
			mavlink_log_info(_mavlink_fd, "[lpe] lidar out of range");
			warnx("[lpe] lidar out of range");
		}
		_lidarFault = 2;
	} else if (beta > _beta_max.get()) {
		if (!_lidarFault) {
			mavlink_log_info(_mavlink_fd, "[lpe] lidar fault, beta %5.2f", double(beta));
			warnx("[lpe] lidar fault, beta %5.2f", double(beta));
		}
		_lidarFault = 1;
	// disable fault if ok
	} else if (_lidarFault) {
		_lidarFault = 0;
		mavlink_log_info(_mavlink_fd, "[lpe] lidar OK");
		warnx("[lpe] lidar OK");
	}

	// kalman filter correction if no fault
	// want to ignore corrections > beta_max since lidar gives
	// bogus readings at times
	if (_lidarFault == 0) {
		math::Matrix<n_x, n_y_lidar> K = _P*C.transposed()*S_I;
		_x += K*r;
		_P -= K*C*_P;
	}
	_time_last_lidar = msg.timestamp;
}

void BlockLocalPositionEstimatorMulti::correctGps(const vehicle_gps_position_s & msg) {
	// TODO : use another other metric for glitch detection

	// gps measurement in local frame
	double  lat = msg.lat*1.0e-7;
	double  lon = msg.lon*1.0e-7;
	float  alt = msg.alt*1.0e-3f;

	float px = 0;
	float py = 0;
	float pz = alt - _gpsAltHome;
	map_projection_project(&_map_ref, lat, lon, &px, &py);

	//printf("gps: lat %10g, lon, %10g alt %10g\n", lat, lon, double(alt));
	//printf("home: lat %10g, lon, %10g alt %10g\n", lat, lon, double(alt));
	//printf("local: x %10g y %10g z %10g\n", double(px), double(py), double(pz));

	math::Vector<6> y;
	y(0) = px;
	y(1) = py;
	y(2) = pz;
	y(3) = msg.vel_n_m_s;
	y(4) = msg.vel_e_m_s;
	y(5) = msg.vel_d_m_s;

	// gps measurement matrix, measures position and velocity
	math::Matrix<n_y_gps, n_x> C;
	C(Y_gps_x, X_x) = 1;
	C(Y_gps_y, X_y) = 1;
	C(Y_gps_z, X_z) = 1;
	C(Y_gps_vx, X_vx) = 1;
	C(Y_gps_vy, X_vy) = 1;
	C(Y_gps_vz, X_vz) = 1;

	// gps covariance matrix
	math::Matrix<n_y_gps, n_y_gps> R;

	// default to parameter, use gps cov if provided
	float var_xy = _gps_xy_stddev.get()*_gps_xy_stddev.get();
	float var_z = _gps_z_stddev.get()*_gps_z_stddev.get();
	float var_vxy = _gps_vxy_stddev.get()*_gps_vxy_stddev.get();
	float var_vz = _gps_vz_stddev.get()*_gps_vz_stddev.get();

	// if field is not zero, set it to the value provided
	if (msg.eph > 1e-3f)
		var_xy = msg.eph*msg.eph;
	if (msg.epv > 1e-3f)
		var_z = msg.epv*msg.epv;

	// TODO is velocity covariance provided from gps sub
	R(0,0) = var_xy;
	R(1,1) = var_xy;
	R(2,2) = var_z;
	R(3,3) = var_vxy;
	R(4,4) = var_vxy;
	R(5,5) = var_vz;

	// residual
	math::Matrix<6,6> S_I = ((C*_P*C.transposed()) + R).inversed();
	math::Vector<6> r = y - C*_x;

	// fault detection
	float beta = sqrtf(r*(S_I*r));
	if (beta > _beta_max.get()) {
		if (!_gpsFault) {
			mavlink_log_info(_mavlink_fd, "[lpe] gps fault, beta: %5.2f", double(beta));
			warnx("[lpe] gps fault, beta: %5.2f", double(beta));
			mavlink_log_info(_mavlink_fd, "[lpe] r: %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f", 
					double(r(0)),  double(r(1)), double(r(2)),
					double(r(3)), double(r(4)), double(r(5)));
			mavlink_log_info(_mavlink_fd, "[lpe] S_I: %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f", 
					double(S_I(0,0)),  double(S_I(1,1)), double(S_I(2,2)),
					double(S_I(3,3)),  double(S_I(4,4)), double(S_I(5,5)));
			mavlink_log_info(_mavlink_fd, "[lpe] r: %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f", 
					double(r(0)),  double(r(1)), double(r(2)),
					double(r(3)), double(r(4)), double(r(5)));
		}
		_gpsFault = 1;
		// trust GPS less
		S_I = ((C*_P*C.transposed()) + R*10).inversed();
	} else if (_gpsFault) {
		_gpsFault = 0;
		mavlink_log_info(_mavlink_fd, "[lpe] GPS OK");
		warnx("[lpe] GPS OK");
	}

	// kalman filter correction if no hard fault
	if (_gpsFault < 2) {
		math::Matrix<n_x, n_y_gps> K = _P*C.transposed()*S_I;
		_x += K*r;
		_P -= K*C*_P;
	}
	_time_last_gps = _timeStamp;
}

void BlockLocalPositionEstimatorMulti::correctVisionPos(const vision_position_estimate_s & msg) {

	math::Vector<3> y;
	y(0) = msg.x - _visionHome(0);
	y(1) = msg.y - _visionHome(1);
	y(2) = msg.z - _visionHome(2);

	// vision measurement matrix, measures position
	math::Matrix<n_y_vision_pos, n_x> C;
	C(Y_vision_x, X_x) = 1;
	C(Y_vision_y, X_y) = 1;
	C(Y_vision_z, X_z) = 1;

	// noise matrix
	math::Matrix<n_y_vision_pos, n_y_vision_pos> R;
	R(Y_vision_x, Y_vision_x) = _vision_xy_stddev.get()*_vision_xy_stddev.get();
	R(Y_vision_y, Y_vision_y) = _vision_xy_stddev.get()*_vision_xy_stddev.get();
	R(Y_vision_z, Y_vision_z) = _vision_z_stddev.get()*_vision_z_stddev.get();

	// residual
	math::Matrix<3,3> S_I = ((C*_P*C.transposed()) + R).inversed();
	math::Vector<3> r = y - C*_x;

	// fault detection
	float beta = sqrtf(r*(S_I*r));
	if (beta > _beta_max.get()) {
		if (!_visionPosFault) {
			mavlink_log_info(_mavlink_fd, "[lpe] vision position fault, beta %5.2f", double(beta));
			warnx("[lpe] vision position fault, beta %5.2f", double(beta));
		}
		_visionPosFault = 1;
		// trust less
		S_I = ((C*_P*C.transposed()) + R*10).inversed();
	} else if (_visionPosFault) {
		_visionPosFault = 0;
		mavlink_log_info(_mavlink_fd, "[lpe] vision position OK");
		warnx("[lpe] vision position OK");
	}

	// kalman filter correction if no fault
	if (_visionPosFault < 2) {
		math::Matrix<n_x, n_y_vision_pos> K = _P*C.transposed()*S_I;
		_x += K*r;
		_P -= K*C*_P;
	}

	_time_last_vision_p = msg.timestamp_boot;
}

void BlockLocalPositionEstimatorMulti::correctVisionVel(const vision_speed_estimate_s & msg) {

	math::Vector<3> y;
	y(3) = msg.x - _visionBaseVel(0);
	y(4) = msg.y - _visionBaseVel(1);
	y(5) = msg.z - _visionBaseVel(2);

	// vision measurement matrix, measures velocity
	math::Matrix<n_y_vision_pos, n_x> C;
	C(Y_vision_vx, X_vx) = 1;
	C(Y_vision_vy, X_vy) = 1;
	C(Y_vision_vz, X_vz) = 1;

	// noise matrix
	math::Matrix<n_y_vision_vel, n_y_vision_vel> R;
	R(Y_vision_vx, Y_vision_vx) = _vision_vxy_stddev.get()*_vision_vxy_stddev.get();
	R(Y_vision_vy, Y_vision_vy) = _vision_vxy_stddev.get()*_vision_vxy_stddev.get();
	R(Y_vision_vz, Y_vision_vz) = _vision_vz_stddev.get()*_vision_vz_stddev.get();

	// residual
	math::Matrix<3,3> S_I = ((C*_P*C.transposed()) + R).inversed();
	math::Vector<3> r = y - C*_x;

	// fault detection
	float beta = sqrtf(r*(S_I*r));
	if (beta > _beta_max.get()) {
		if (!_visionVelFault) {
			mavlink_log_info(_mavlink_fd, "[lpe] vision velocity fault, beta %5.2f", double(beta));
			warnx("[lpe] vision position fault, beta %5.2f", double(beta));
		}
		_visionVelFault = 1;
		// trust less
		S_I = ((C*_P*C.transposed()) + R*10).inversed();
	} else if (_visionVelFault) {
		_visionVelFault = 0;
		mavlink_log_info(_mavlink_fd, "[lpe] vision velocity OK");
		warnx("[lpe] vision velocity OK");
	}

	// kalman filter correction if no fault
	if (_visionVelFault < 2) {
		math::Matrix<n_x, n_y_vision_vel> K = _P*C.transposed()*S_I;
		_x += K*r;
		_P -= K*C*_P;
	}

	_time_last_vision_v = msg.timestamp_boot;
}

void BlockLocalPositionEstimatorMulti::correctVicon(const vehicle_vicon_position_s & msg) {

	math::Vector<3> y;
	y(0) = msg.x - _viconHome(0);
	y(1) = msg.y - _viconHome(1);
	y(2) = msg.z - _viconHome(2);

	// vicon measurement matrix, measures position
	math::Matrix<n_y_vicon, n_x> C;
	C(Y_vicon_x, X_x) = 1;
	C(Y_vicon_y, X_y) = 1;
	C(Y_vicon_z, X_z) = 1;

	// noise matrix
	math::Matrix<n_y_vicon, n_y_vicon> R;
	float vicon_p_var = _vicon_p_stddev.get()*_vicon_p_stddev.get();
	R(Y_vicon_x, Y_vicon_x) = vicon_p_var;
	R(Y_vicon_y, Y_vicon_y) = vicon_p_var;
	R(Y_vicon_z, Y_vicon_z) = vicon_p_var;

	// residual
	math::Matrix<3,3> S_I = ((C*_P*C.transposed()) + R).inversed();
	math::Vector<3> r = y - C*_x;

	// fault detection
	float beta = sqrtf(r*(S_I*r));
	if (beta > _beta_max.get()) {
		if (!_viconFault) {
			mavlink_log_info(_mavlink_fd, "[lpe] vicon fault, beta %5.2f", double(beta));
			warnx("[lpe] vicon fault, beta %5.2f", double(beta));
		}
		_viconFault = 1;
		// trust less
		S_I = ((C*_P*C.transposed()) + R*10).inversed();
	} else if (_viconFault) {
		_viconFault = 0;
		mavlink_log_info(_mavlink_fd, "[lpe] vicon OK");
		warnx("[lpe] vicon OK");
	}

	// kalman filter correction if no fault
	if (_viconFault < 2) {
		math::Matrix<n_x, n_y_vicon> K = _P*C.transposed()*S_I;
		_x += K*r;
		_P -= K*C*_P;
	}

	_time_last_vicon = msg.timestamp;
}

void BlockLocalPositionEstimatorMulti::handleAttitude(const vehicle_attitude_s & msg) {
	_att = msg;
}

void BlockLocalPositionEstimatorMulti::handleOpticalFlow(const optical_flow_s & msg) {
	if (!_flowInitialized)
		initFlow(msg);
	else {
		perf_begin(_loop_perf);
		correctFlow(msg);	
		perf_count(_interval_perf);
		perf_end(_loop_perf);
	}
}

void BlockLocalPositionEstimatorMulti::handleSensorCombined(const sensor_combined_s & msg) {
	if (!_baroInitialized)
		initBaro(msg);
	else {
		correctBaro(msg);
	}
	_sensor_combined = msg;
}

void BlockLocalPositionEstimatorMulti::handleDistanceSensor(const distance_sensor_s & msg) {
	if (msg.type == distance_sensor_s::MAV_DISTANCE_SENSOR_LASER) {
		if (!_lidarInitialized) {
			initLidar(msg);
		} else {
			correctLidar(msg);
		}
	} else if (msg.type == distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND) {
		if (!_sonarInitialized) {
			initSonar(msg);
		} else {
			correctSonar(msg);
		}
	} else if (msg.type == distance_sensor_s::MAV_DISTANCE_SENSOR_INFRARED) {
		mavlink_log_info(_mavlink_fd, "[lpe] no support to short-range infrared sensors ");
		warnx("[lpe] short-range infrared detected. Ignored... ");		
	}
}

void BlockLocalPositionEstimatorMulti::handleParamUpdate(const parameter_update_s & msg) {
	updateParams();
}

void BlockLocalPositionEstimatorMulti::handleHome(const home_position_s & msg) {
	double lat = msg.lat;
	double lon = msg.lon;
	float alt = msg.alt;
	mavlink_log_info(_mavlink_fd, "[lpe] home: lat %5.0f, lon %5.0f, alt %5.0f", lat, lon, double(alt));
	warnx("[lpe] home: lat %5.0f, lon %5.0f, alt %5.0f", lat, lon, double(alt));
	map_projection_init(&_map_ref, lat, lon);
	float delta_alt = alt - _altHome;
	_altHome = alt;
	_gpsAltHome += delta_alt;
	_baroAltHome +=  delta_alt;
	_lidarAltHome +=  delta_alt;
	_sonarAltHome +=  delta_alt;
	_home = msg;
}

void BlockLocalPositionEstimatorMulti::handleGPS(const vehicle_gps_position_s & msg) {
	if (!_gpsInitialized) {
		initGps(msg);
	} else {
		correctGps(msg);
	}
	_gps = msg;
}



void BlockLocalPositionEstimatorMulti::handleVicon(const vehicle_vicon_position_s & msg) {
	if (!_viconInitialized) {
		initVicon(msg);
	} else {
		correctVicon(msg);
	}
}

void BlockLocalPositionEstimatorMulti::handleVisionPosition(const vision_position_estimate_s & msg) {
	if (_no_vision.get() == CBRK_NO_VISION_KEY) return;
	if (!_visionPosInitialized) {
		initVisionPos(msg);
	} else {
		correctVisionPos(msg);
	}
}

void BlockLocalPositionEstimatorMulti::handleVisionVelocity(const vision_speed_estimate_s & msg) {
	if (_no_vision.get() == CBRK_NO_VISION_KEY) return;
	if (!_visionVelInitialized) {
		initVisionVel(msg);
	} else {
		correctVisionVel(msg);
	}
}
