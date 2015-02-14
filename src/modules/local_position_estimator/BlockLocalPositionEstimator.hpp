#pragma once

#include <controllib/uorb/blocks.hpp>
#include <mathlib/mathlib.h>
#include <systemlib/perf_counter.h>

// uORB Subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/sensor_combined.h>
#include <drivers/drv_range_finder.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/manual_control_setpoint.h>

// uORB Publications
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/filtered_bottom_flow.h>

using namespace control;

class BlockLocalPositionEstimator : public control::SuperBlock {
//
// The purpose of this estimator is to provide a robust solution for
// indoor flight.
//
// dynamics:
//
//	x(+) = A * x(-) + B * u(+)
//	y_i = C_i*x
//
// kalman filter
//
//	E[xx'] = P
//	E[uu'] = W
//	E[y_iy_i'] = R_i
//
//	prediction
//		x(+|-) = A*x(-|-) + B*u(+)
//		P(+|-) = A*P(-|-)*A' + B*W*B'
//
//	correction
//		x(+|+) =  x(+|-) + K_i * (y_i - H_i * x(+|-) )
//		
//
// input:
// 	ax, ay, az (acceleration NED)
//
// states: 
// 	px, py, pz , ( position NED)
// 	vx, vy, vz ( vel NED),
// 	bx, by, bz ( TODO accelerometer bias)
//
// measurements:
//
// 	sonar: pz (TODO should account for roll/pitch)
//
// 	baro: pz (TODO should account for roll/pitch)
//
// 	flow: vx, vy (flow is in body x, y frame)
//
public:
	BlockLocalPositionEstimator();
	void update();
	virtual ~BlockLocalPositionEstimator();

private:

	// constants
	static const uint8_t n_x = 6;
	static const uint8_t n_u = 3; // 3 accelerations
	static const uint8_t n_y_flow = 3;
	static const uint8_t n_y_baro = 1;
	static const uint8_t n_y_lidar = 1;
	enum {X_px=0, X_py, X_pz, X_vx, X_vy, X_vz}; //, X_bx, X_by, X_bz};
	enum {U_ax=0, U_ay, U_az};
	enum {Y_baro_z=0};
	enum {Y_lidar_z=0};
	enum {Y_flow_vx=0, Y_flow_vy, Y_flow_z};
	enum {POLL_FLOW, POLL_SENSORS, POLL_PARAM};
	enum {CH_LEFT, CH_RIGHT};

	// methods
	// ----------------------------
	
	// predict the next state
	void predict();

	// update the state prediction wtih a measurement
	void update_flow();
	void update_baro();
	//void update_lidar();

	// attributes
	// ----------------------------

	// subscriptions
	uORB::Subscription<vehicle_status_s> _status;
	uORB::Subscription<actuator_armed_s> _armed;
	uORB::Subscription<vehicle_control_mode_s> _control_mode;
	uORB::Subscription<vehicle_attitude_s> _att;
	uORB::Subscription<vehicle_attitude_setpoint_s> _att_sp;
	uORB::Subscription<optical_flow_s> _flow;
	uORB::Subscription<sensor_combined_s> _sensor;
	uORB::Subscription<range_finder_report> _range_finder;
	uORB::Subscription<parameter_update_s> _param_update;
	uORB::Subscription<manual_control_setpoint_s> _manual;

	// publications
	uORB::Publication<vehicle_local_position_s> _pos;
	uORB::Publication<filtered_bottom_flow_s> _filtered_flow;

	BlockParamFloat  _flow_v_stddev;
	BlockParamFloat  _flow_z_stddev;
	BlockParamFloat  _lidar_z_stddev;
	BlockParamFloat  _accel_xy_stddev;
	BlockParamFloat  _accel_z_stddev;
	BlockParamFloat  _baro_stddev;

	struct pollfd _polls[3];
	uint64_t _timeStamp;
	uint64_t _time_last_flow;
	uint64_t _baro_timestamp;

	perf_counter_t _loop_perf;
	perf_counter_t _interval_perf;
	perf_counter_t _err_perf;

	math::Matrix<n_x, n_x>  _A; // state dynamics matrix
	math::Matrix<n_x, n_u>  _B; // input matrix
	math::Matrix<n_x, n_x>  _Q; // process noise
	math::Matrix<n_y_flow, n_x> _C_flow; // flow measurement matrix
	math::Matrix<n_y_baro, n_x> _C_baro; // baro measurement matrix

	math::Vector<n_x>  _x; // state vecotr
	math::Vector<n_u>  _u; // input vector
	math::Matrix<n_x, n_x>  _P; // state covariance matrix
};
