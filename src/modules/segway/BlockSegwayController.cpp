#include "BlockSegwayController.hpp"

// px4
#include <geo/geo.h>
#include <drivers/drv_hrt.h>

BlockSegwayController::BlockSegwayController() :
	SuperBlock(NULL, "SEG"),

	// subscriptions
	_att(&getSubscriptions(), ORB_ID(vehicle_attitude), 3),
	_pos(&getSubscriptions() , ORB_ID(vehicle_global_position), 3),
	_posCmd(&getSubscriptions(), ORB_ID(position_setpoint_triplet), 3),
	_localPos(&getSubscriptions() , ORB_ID(vehicle_local_position), 3),
	_localPosCmd(&getSubscriptions(), ORB_ID(vehicle_local_position_setpoint), 3),
	_manual(&getSubscriptions(), ORB_ID(manual_control_setpoint), 3),
	_status(&getSubscriptions(), ORB_ID(vehicle_status), 3),
	_param_update(&getSubscriptions(), ORB_ID(parameter_update), 1000), // limit to 1 Hz
	_encoders(&getSubscriptions(), ORB_ID(encoders), 10), // limit to 100 Hz
	_battery(&getSubscriptions(), ORB_ID(battery_status), 10), // limit to 100 Hz

	// publications
	_attCmd(&getPublications(), ORB_ID(vehicle_attitude_setpoint)),
	_ratesCmd(&getPublications(), ORB_ID(vehicle_rates_setpoint)),
	_globalVelCmd(&getPublications(), ORB_ID(vehicle_global_velocity_setpoint)),
	_actuators(&getPublications(), ORB_ID(actuator_controls_1)),

	_yaw2r(this, "YAW2R"),
	_r2v(this, "R2V"),
	_th2v(this, "TH2V"),
	_q2v(this, "Q2V"),
	_x2vel(this, "X2VEL"),
	_vel2th(this, "VEL2TH"),
	_thLimit(this, "TH_LIM"),
	_velLimit(this, "VEL_LIM"),
	_thStop(this, "TH_STOP"),
	_trimPitch(this, "TRIM_PITCH", false),
	_pulsesPerRev(this, "ENCP_PPR", false),

	_mgl(this, "MGL"),
	_J(this, "J"),
	_k_emf(this, "K_EMF"),
	_k_damp(this, "K_DAMP"),
	_wn_theta(this, "WN_THETA"),
	_zeta_theta(this, "ZETA_THETA"),
	_trimPitch(this, "TRIM_PITCH", false),
	_bemf(this, "BEMF"),
	_sysIdEnable(this, "SYSID_ENABLE"),
	_sysIdAmp(this, "SYSID_AMP"),
	_sysIdFreq(this, "SYSID_FREQ"),
	_attPoll(),
	_timeStamp(0),
	_thCmd(0),
	_rCmd(0),
	_yawCmd(0),
	_velCmd(0),
	_xCmd(0),
	_controlPitch(0),
	_controlYaw(0)
{
	_attPoll.fd = _att.getHandle();
	_attPoll.events = POLLIN;
}

void BlockSegwayController::update()
{
	// wait for a sensor update, check for exit condition every 100 ms
	if (poll(&_attPoll, 1, 100) < 0) { return; } // poll error

	uint64_t newTimeStamp = hrt_absolute_time();
	float dt = (newTimeStamp - _timeStamp) / 1.0e6f;
	_timeStamp = newTimeStamp;

	// check for sane values of dt
	// to prevent large control responses
	if (dt > 1.0f || dt < 0) { return; }

	// set dt for all child blocks
	setDt(dt);

	// check for new updates
	if (_param_update.updated()) { updateParams(); }

	// get new information from subscriptions
	updateSubscriptions();

	// handle modes
	if (_sysIdEnable.get() > 0) { // sysid
		handleSysIdModes();
	} else { // normal modes
		handleNormalModes();
	}

	// call custom publication update function
	updatePublications();
}

void BlockSegwayController::handleNormalModes()
{
	setControlsToZero();
	if (_status.main_state == MAIN_STATE_MANUAL) {
		// user controls vel cmd and yaw rate cmd
		_thCmd = -_thLimit.getMax()*_manual.x; // note negative, since neg pitch goes fwd
		_rCmd = _manual.y;
	} else if (_status.main_state == MAIN_STATE_ALTCTL) {
		// user controls vel cmd and yaw rate cmd
		_velCmd = _manual.x * _velLimit.getMax();
		velCmd2PitchCmd();
		_rCmd = _manual.y;
	} else if (_status.main_state == MAIN_STATE_ACRO) {
		// user controls th cmd and yaw rate cmd
		_thCmd = -_thLimit.getMax()*_manual.x; // note negative, since neg pitch goes fwd
		_rCmd = _manual.y;
	} else if (_status.main_state == MAIN_STATE_POSCTL) {
		// user controls pos cmd and yaw rate cmd
		_xCmd = 0.5f * _manual.x;
		xCmd2VelocityCmd();
		velCmd2PitchCmd();
		_rCmd = _manual.y;
	} else if (_status.main_state == MAIN_STATE_AUTO_MISSION) {
		_xCmd = _localPosCmd.x;
		xCmd2VelocityCmd();
		velCmd2PitchCmd();
		_yawCmd = _localPosCmd.yaw;
		yawCmd2YawRateCmd();
	} else if (_status.main_state == MAIN_STATE_AUTO_LOITER) {
		_xCmd = _localPosCmd.x;
		// TODO check if local pos cmd is set to loiter pos.
		xCmd2VelocityCmd();
		velCmd2PitchCmd();
		_yawCmd = _localPosCmd.yaw;
		yawCmd2YawRateCmd();
	} else if (_status.main_state == MAIN_STATE_AUTO_RTL) {
		_xCmd = 0;
		xCmd2VelocityCmd();
		velCmd2PitchCmd();
		_yawCmd = _localPosCmd.yaw;
		yawCmd2YawRateCmd();
	} else if (_status.main_state == MAIN_STATE_OFFBOARD) {
	} else if (_status.main_state == MAIN_STATE_MAX) {
	}

	// compute angles and rates
	float th = _att.pitch -_trimPitch.get();
	float th_dot = _att.pitchspeed;
	float r = _att.yawspeed;
	float alpha_dot_left = _encoders.velocity[0]/_pulsesPerRev.get();
	float alpha_dot_right = _encoders.velocity[1]/_pulsesPerRev.get();
	float alpha_dot_mean = (alpha_dot_left + alpha_dot_right)/2;
	float alpha_dot_diff = (alpha_dot_left - alpha_dot_right);

	// constants
	float k_emf = _k_emf.get();
	float k_damp = _k_damp.get();
	float wn_theta = _wn_theta.get();
	float zeta_theta = _zeta_theta.get();
	float J = _J.get();
	float mgl = _mgl.get();
	float V_batt = _battery.voltage_filtered_v;

	// dynamic inversion
	float th_ddot_d = -2*zeta_theta*wn_theta*th_dot - wn_theta**2(th - _thCmd);
	float V_pitch = -J*th_ddot_d/(2*k_emf) - mgl*sinf(th)/(2*k_emf) + alpha_dot_mean*k_damp/k_emf;
	float V_yaw = _r2v.update(_rCmd - r) + alpha_dot_diff*k_damp/k_emf;

	// compute duty (0-1)
	float dutyPitch = V_pitch/V_batt;
	float dutyYaw = V_yaw/V_batt;

	float inv_dynamics_yaw = _bemf.get()*
		(alpha_dot_left - alpha_dot_right)/2;
	float inv_dynamics_pitch = _mgl.get()*sinf(th)
		+ _bemf.get()*(alpha_dot_left + alpha_dot_right)/2;

	// compute control for pitch
	_controlPitch = _th2v.update(_thCmd - th)
		- _q2v.update(_att.pitchspeed) - inv_dynamics_pitch;

	// compute control for yaw
	_controlYaw = _r2v.update(_rCmd - _att.yawspeed) - inv_dynamics_yaw;

	// output scaling by manual throttle
	_controlPitch *= _manual.z;
	_controlYaw *= _manual.z;
}

void BlockSegwayController::handleSysIdModes()
{
	setControlsToZero();
	float sineWave = _sysIdAmp.get() *
		sinf(2.0f * M_PI_F * _sysIdFreq.get() * _timeStamp / 1.0e6f);
	float squareWave = 0;
	if (sineWave > 0) {
		squareWave = _sysIdAmp.get();
	} else {
		squareWave = -_sysIdAmp.get();
	}
	if (_status.main_state == MAIN_STATE_MANUAL) {
		_controlPitch = _manual.x;
	} else if (_status.main_state == MAIN_STATE_ALTCTL) {
		_controlPitch = sineWave;
	} else if (_status.main_state == MAIN_STATE_POSCTL) {
		_controlPitch = squareWave;
	}
	// output scaling by manual throttle
	dutyPitch *= _manual.z;
	dutyYaw *= _manual.z;
}

void BlockSegwayController::updatePublications() {
	// attitude set point
	_attCmd.timestamp = _timeStamp;
	_attCmd.pitch_body = _thCmd;
	_attCmd.roll_body = 0;
	_attCmd.yaw_body = _yawCmd;
	_attCmd.R_valid = false;
	_attCmd.q_d_valid = false;
	_attCmd.q_e_valid = false;
	_attCmd.thrust = 0;
	_attCmd.roll_reset_integral = false;
	_attCmd.update();

	// rates set point
	_ratesCmd.timestamp = _timeStamp;
	_ratesCmd.roll = 0;
	_ratesCmd.pitch = 0;
	_ratesCmd.yaw = _rCmd;
	_ratesCmd.thrust = 0;
	_ratesCmd.update();

	// global velocity set point
	_globalVelCmd.vx = _velCmd;
	_globalVelCmd.vy = 0;
	_globalVelCmd.vz = 0;
	_globalVelCmd.update();

	// send outputs if armed and pitch less
	// than shut off pitch
	if (_status.arming_state == ARMING_STATE_ARMED &&
	    fabsf(_att.pitch) < _thStop.get()) {
		// controls
		_actuators.timestamp = _timeStamp;
		_actuators.control[0] = 0; // roll
		_actuators.control[1] = dutyPitch; // pitch
		_actuators.control[2] = dutyYaw; // yaw
		_actuators.control[3] = 0; // thrust
		_actuators.update();

	} else {
		// controls
		_actuators.timestamp = _timeStamp;
		_actuators.control[0] = 0; // roll
		_actuators.control[1] = 0; // pitch
		_actuators.control[2] = 0; // yaw
		_actuators.control[3] = 0; // thrust
		_actuators.update();
	}
}

void BlockSegwayController::xCmd2VelocityCmd()
{
	_velCmd = _velLimit.update(_x2vel.update(_xCmd - _localPos.x));
}

void BlockSegwayController::yawCmd2YawRateCmd()
{
	float yawError = _yawCmd - _att.yaw;

	// wrap yaw error to between -180 and 180
	if (yawError > M_PI_F / 2) { yawError = yawError - 2 * M_PI_F; }

	if (yawError < -M_PI_F / 2) { yawError = yawError + 2 * M_PI_F; }

	_rCmd = _yaw2r.update(yawError);
}

void BlockSegwayController::velCmd2PitchCmd()
{
	// negative sign since need to lean in negative pitch to move forward
	_thCmd = -_thLimit.update(_vel2th.update(_velCmd - _localPos.vx));
}

void BlockSegwayController::setControlsToZero() {
	// initialize all controls to zero
	_thCmd = 0; // pitch command
	_rCmd = 0; // yaw rate command
	_yawCmd = 0; // always point north for now, can use localPosCmd.yaw later
	_velCmd = 0; // velocity command
	_xCmd = 0; // position command
	_controlPitch = 0;
	_controlYaw = 0;
}
