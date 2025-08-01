#ifndef MOTION_CONTROLLER_CONFIG_H_
#define MOTION_CONTROLLER_CONFIG_H_

#include <config/stepper_config.h>

// in units (per second ^0 ^1 ^2 or ^3, respectively)

#define MOT_X_MOTION_MIN_SPEED 30.f
#define MOT_X_MOTION_MAX_SPEED 2000.f
#define MOT_X_MOTION_MAX_ACCEL 8000.f
#define MOT_X_MOTION_MAX_JERK 20000.f
#define MOT_X_MOTION_SHORT_DIST 0.0f // mm
/**
#define MOT_X_MOTION_MIN_SPEED 100.f
#define MOT_X_MOTION_MAX_SPEED 100.f
#define MOT_X_MOTION_MAX_ACCEL 200.f
#define MOT_X_MOTION_MAX_JERK 400.f
#define MOT_X_MOTION_SHORT_DIST 10000.f // mm
 */

#define MOT_YAW_MOTION_MIN_SPEED 30.f
#define MOT_YAW_MOTION_MAX_SPEED 800.f
#define MOT_YAW_MOTION_MAX_ACCEL 3200.f
#define MOT_YAW_MOTION_MAX_JERK 4000.f
#define MOT_YAW_MOTION_SHORT_DIST 10.f // deg

#define MOT_Z_MOTION_MIN_SPEED 30.f
#define MOT_Z_MOTION_MAX_SPEED 2000.f
#define MOT_Z_MOTION_MAX_ACCEL 5000.f
#define MOT_Z_MOTION_MAX_JERK 12000.f
#define MOT_Z_MOTION_SHORT_DIST 10.f // mm

#define MOT_U_MOTION_MIN_SPEED 30.f
#define MOT_U_MOTION_MAX_SPEED 2000.f
#define MOT_U_MOTION_MAX_ACCEL 2000.f
#define MOT_U_MOTION_MAX_JERK 2000.f
#define MOT_U_MOTION_SHORT_DIST 10.f // mm

#endif
