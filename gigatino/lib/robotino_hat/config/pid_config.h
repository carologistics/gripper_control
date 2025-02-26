#ifndef PID_CONFIG_H_
#define PID_CONFIG_H_

#include <config/stepper_config.h>

// SPEED needs to be in unit per second (not steps!)
// ACCEL needs to be in unit per second^2 (not steps!)

#define MOT_X_PID_P 0.5
#define MOT_X_PID_I 0.0
#define MOT_X_PID_D 0.0
#define MOT_X_MAX_ACCEL 2000.f * MOT_X_ROT_PER_STEP / MOT_X_GEAR_RATIO
#define MOT_X_MAX_SPEED 2600.f * MOT_X_ROT_PER_STEP / MOT_X_GEAR_RATIO
#define MOT_X_MIN_SPEED 20.f * MOT_X_ROT_PER_STEP / MOT_X_GEAR_RATIO

#define MOT_YAW_PID_P 2.5
#define MOT_YAW_PID_I 0.001
#define MOT_YAW_PID_D 1.5
#define MOT_YAW_MAX_ACCEL 2000.f * MOT_YAW_ROT_PER_STEP / MOT_YAW_GEAR_RATIO
// equates to 360 rpm, which is around max (given maximal limit of axis)
#define MOT_YAW_MAX_SPEED 1200.f * MOT_YAW_ROT_PER_STEP / MOT_YAW_GEAR_RATIO
#define MOT_YAW_MIN_SPEED 20.f * MOT_YAW_ROT_PER_STEP / MOT_YAW_GEAR_RATIO

#define MOT_Z_PID_P 4.0
#define MOT_Z_PID_I 0.05
#define MOT_Z_PID_D 0.24
#define MOT_Z_MAX_ACCEL 2000.f * MOT_Z_ROT_PER_STEP / MOT_Z_GEAR_RATIO
#define MOT_Z_MAX_SPEED 1200.f * MOT_Z_ROT_PER_STEP / MOT_Z_GEAR_RATIO
#define MOT_Z_MIN_SPEED 20.f * MOT_Z_ROT_PER_STEP / MOT_Z_GEAR_RATIO

#define MOT_U_PID_P 0.1
#define MOT_U_PID_I 0.0
#define MOT_U_PID_D 0.0
#define MOT_U_MAX_ACCEL 2000.f * MOT_U_ROT_PER_STEP / MOT_U_GEAR_RATIO
#define MOT_U_MAX_SPEED 2800.f * MOT_U_ROT_PER_STEP / MOT_U_GEAR_RATIO
#define MOT_U_MIN_SPEED 20.f * MOT_U_ROT_PER_STEP / MOT_U_GEAR_RATIO

#endif // PID_CONFIG_H_
