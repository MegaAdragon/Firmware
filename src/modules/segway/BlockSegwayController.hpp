#pragma once

// system
#include <poll.h>

// subscription topics
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_global_position_set_triplet.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/encoders.h>

// publication topics
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/actuator_controls.h>

// control blocks
#include <controllib/blocks.hpp>

#include <controllib/uorb/UOrbSubscription.hpp>
#include <controllib/uorb/UOrbPublication.hpp>

using namespace control;

class BlockSegwayController : public control::SuperBlock {
public:
	BlockSegwayController();
	void update();
private:
	enum {CH_LEFT, CH_RIGHT};

	// subscriptions
	UOrbSubscription<vehicle_attitude_s> _att;
	UOrbSubscription<vehicle_global_position_s> _pos;
	UOrbSubscription<vehicle_global_position_set_triplet_s> _posCmd;
	UOrbSubscription<vehicle_local_position_s> _localPos;
	UOrbSubscription<vehicle_local_position_setpoint_s> _localPosCmd;
	UOrbSubscription<manual_control_setpoint_s> _manual;
	UOrbSubscription<vehicle_status_s> _status;
	UOrbSubscription<parameter_update_s> _param_update;
	UOrbSubscription<encoders_s> _encoders;

	// publications
	UOrbPublication<vehicle_attitude_setpoint_s> _attCmd;
	UOrbPublication<vehicle_rates_setpoint_s> _ratesCmd;
	UOrbPublication<vehicle_global_velocity_setpoint_s> _globalVelCmd;
	UOrbPublication<actuator_controls_s> _actuators;

	// control blocks
	BlockP _yaw2r; // yaw error to yaw rate cmd
	BlockP _r2v; // yaw rate error to voltage cmd
	BlockP _th2v; // pitch error to voltage cmd (PD P term with q2v)
	BlockP _q2v; // pitch rate error to voltage cmd (PD D term with th2v)
	BlockPI _x2vel; // position error to velocity cmd
	BlockPI _vel2th; // velocity error to pitch cmd
	BlockLimitSym _thLimit; // pitch limit
	BlockLimitSym _velLimit; // velocity limit
	BlockParamFloat _thStop; // angle at which motors are stopped (safety)

	// sysid
	BlockParamFloat _sysIdAmp; // amplitude of sysid wave
	BlockParamFloat _sysIdFreq; // frequency of sysid wave

	// timing
	struct pollfd _attPoll; // attitude polling
	uint64_t _timeStamp; // timestamp for loop timing
};
