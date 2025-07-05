#ifndef SERVO_CONFIG_H_
#define SERVO_CONFIG_H_

// freq in hz
#define SERVO_GRIPPER_PWM_TIM_FREQ 240000000
#define SERVO_ROTATION_PWM_TIM_FREQ 240000000

/*
 * 240 Mhz -> ~4ns per increase.
 * Setting prescaler to 240 -> 1 Mhz
 * -> 1 tick = 1 microsecond
 */
#define SERVO_GRIPPER_PWM_PRESCALER 240
#define SERVO_ROTATION_PWM_PRESCALER 240

// period frequency in hz
#define SERVO_GRIPPER_PWM_TARGET_FREQ 330
#define SERVO_ROTATION_PWM_TARGET_FREQ 50

// PWM_TIM_FREQ/PRESCALER/PWM_TARGET_FREQ gives timer reset value

// in microseconds
#define SERVO_GRIPPER_PWM_MIN_PULSE_WIDTH 544
#define SERVO_ROTATION_PWM_MIN_PULSE_WIDTH 800

// in microseconds
#define SERVO_GRIPPER_PWM_MAX_PULSE_WIDTH 2400
#define SERVO_ROTATION_PWM_MAX_PULSE_WIDTH 2200

// MIN_PULSE_WIDTH causes this angle
#define SERVO_GRIPPER_MIN_ANGLE 0
#define SERVO_ROTATION_MIN_ANGLE 0

// MAX_PULSE_WIDTH causes this angle
#define SERVO_GRIPPER_MAX_ANGLE 165
#define SERVO_ROTATION_MAX_ANGLE 165

// speed in deg per millisecond
#define SERVO_GRIPPER_SPEED 272.72  // assuming 0.22sec per 60 deg
#define SERVO_ROTATION_SPEED 272.72 // assuming 0.22sec per 60 deg

#endif // SERVO_CONFIG_H_
