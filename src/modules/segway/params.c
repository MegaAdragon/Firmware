#include <systemlib/param/param.h>
#include <math.h><math.h><math.h><math.h>

// 16 is max name length
PARAM_DEFINE_FLOAT(SEG_YAW2R, 1.0f); // yaw error to yaw rate
PARAM_DEFINE_FLOAT(SEG_R2V, 0.2f); // yaw rate error to voltage
PARAM_DEFINE_FLOAT(SEG_TH2V, 10.0f); // pitch error to voltage
PARAM_DEFINE_FLOAT(SEG_Q2V, 0.7f); // pitch rate error to voltage

// position error to velocity command
PARAM_DEFINE_FLOAT(SEG_X2VEL_P, 0.2f); // proportional gain
PARAM_DEFINE_FLOAT(SEG_X2VEL_I, 0.0f); // integrator gain
PARAM_DEFINE_FLOAT(SEG_X2VEL_I_MAX, 0.0f); // max integrator windup

// velocity error to pitch command
PARAM_DEFINE_FLOAT(SEG_VEL2TH_P, 0.2f); // proportional gain
PARAM_DEFINE_FLOAT(SEG_VEL2TH_I, 0.0f); // integrator gain
PARAM_DEFINE_FLOAT(SEG_VEL2TH_I_MAX, 0.0f); // max integrator windup

PARAM_DEFINE_FLOAT(SEG_TH_LIM_MAX, 5*M_PI/180); // pitch limit
PARAM_DEFINE_FLOAT(SEG_VEL_LIM_MAX, 0.2f); // velocity limit
PARAM_DEFINE_FLOAT(SEG_TH_STOP, 10*M_PI/180); // turn off motors when over

// system id
PARAM_DEFINE_FLOAT(SEG_SYSID_AMP, 0.5f); // wave amplitude, deg pitch
PARAM_DEFINE_FLOAT(SEG_SYSID_FREQ, 0.1f); // wave frquency, Hz

// encoder position estimator
PARAM_DEFINE_FLOAT(ENCP_RWHEEL, 0.1f); // radius of wheel
PARAM_DEFINE_FLOAT(ENCP_PPR, 1200.0f); // encoder pulses per revolution of wheel
