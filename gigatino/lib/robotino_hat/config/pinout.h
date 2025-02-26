#ifndef PINOUT_H_
#define PINOUT_H_

/**
 * Wiring reflects the wiring on the robotino HAT, so do not change anything
 * unless you know what you are doing.
 */

#define MOT_X_ENABLE_PIN D13
#define MOT_YAW_ENABLE_PIN D1
#define MOT_Z_ENABLE_PIN D22
#define MOT_U_ENABLE_PIN D27

#define MOT_X_DIR_PIN D7
#define MOT_YAW_DIR_PIN D2
#define MOT_Z_DIR_PIN D19
#define MOT_U_DIR_PIN D25

#define NO_BREAK_PIN 127u // pin 127 u does not exist
#define MOT_X_BREAK_PIN NO_BREAK_PIN
#define MOT_YAW_BREAK_PIN NO_BREAK_PIN
#define MOT_Z_BREAK_PIN D54
#define MOT_U_BREAK_PIN NO_BREAK_PIN

// ----- TIMER pins -----
// Chosen to match timer access and cannot be changed arbitrarily
// Also, changing pins here is not enough, the timer setup
// (pin and port names, AF number) needs to be adjusted as well
// in stepper_motors.cpp
#define MOT_X_PWM_PIN D5
#define MOT_YAW_PWM_PIN D3
#define MOT_Z_PWM_PIN D8
#define MOT_U_PWM_PIN D9

#define MOT_X_ENC_PIN_A D10
#define MOT_YAW_ENC_PIN_A D68
#define MOT_Z_ENC_PIN_A D4
#define MOT_U_ENC_PIN_A D83 // D83 = A7

#define MOT_X_ENC_PIN_B D12
#define MOT_YAW_ENC_PIN_B D15
#define MOT_Z_ENC_PIN_B D11
#define MOT_U_ENC_PIN_B D66

#define SERVO_GRIPPER_PWM_PIN D6
#define SERVO_ROTATION_PWM_PIN D69
// ----- timer pins end -----
/**
 * mot_x pin 1 is 5v
 * mot_x pin 2 is D10 (enc x pin a)
 * mot_x pin 3 is D12 (enc xpin b)
 * mot_x pin 4 is D0 (enc x pin n)
 * mot_x pin 5 is GND
 *
 * enc_x pin 1 is D5 (mot x pwm)
 * enc_x pin 2 is 5v
 * enc_x pin 3 is D7 (mot x dir)
 * enc_x pin 4 is GND
 * enc_x pin 5 is D13 (mot x enable)
 * enc_x_oin 6 is GND
 */

// ----- ISR pins -----
// Need to be on different ISR multiplexers (port numbers)
// With this pinout the ports 0,1,6,8-11,15 can still be used for ISRs

#define MOT_X_ENC_PIN_N D0    // PB7
#define MOT_YAW_ENC_PIN_N D14 // PG14
#define MOT_Z_ENC_PIN_N D24   // PG12
#define MOT_U_ENC_PIN_N D23   // PG13

/*
 * Endstops X,Z,U are wired as NC (normally closed), while endstop YAW is wired
 * as NO (normally open)
 */
#define MOT_X_ENDSTOP_PIN D29   // PJ2
#define MOT_YAW_ENDSTOP_PIN D31 // PJ3
#define MOT_Z_ENDSTOP_PIN D33   // PJ4
#define MOT_U_ENDSTOP_PIN D35   // PJ5
// ----- ISR pins end -----

#endif // PINOUT_H_
