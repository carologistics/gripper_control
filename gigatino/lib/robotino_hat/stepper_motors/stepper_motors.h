#pragma once

#ifndef STEPPER_MOTORS_H_
#define STEPPER_MOTORS_H_
/**
 * Basic setup for the 4 stepper motors on the robotino hat.
 * X and Z are the linear axis, YAW is the rotation axis and U is unused.
 *
 * Note that in Rev 1.0 of the robotino hat, the X axis is not wired correctly,
 * hence we use U for X.
 */
#include <pid_controller/pid_controller.h>
#include <s_controller/s_controller.h>
#include <timer_setup/timer_setup.h>

namespace stepper_motors {
//
/*
 * Stepper Setup for motors and encoders
 * Encoder counters will count upwards when the motor spins in the clockwise
 * direction. Likewise, abs_position_step_offset will increase accoridingly with
 * clockwise direction and decrease with counter-clockwise direction.
 *
 * Determine the abs position by:
 *  - reading the encoder counter
 *  - substracting the base value ()
 *  - and adding the abs_position_step_offset
 */
struct StepperMotorSetup {
  TimerPinSetup pwm;
  TimerPinSetup encoder_a;
  TimerPinSetup encoder_b;
  const uint16_t encoder_n_pin;
  const uint16_t micro_steps;
  const float gear_ratio;
  const int32_t encoder_rev_count;
  int32_t encoder_endstop_delta;
  const float rot_per_step;
  const uint16_t enable_pin;
  const uint16_t dir_pin;
  const uint16_t endstop_pin;
  const uint16_t break_pin;
  const uint16_t pwm_prescaler;
  uint32_t pwm_freq;
  const uint16_t steps_per_rev;
  bool direction;
  volatile int32_t abs_position_step_offset; // controlled by n-phase interrupty
  const bool reference_dir;
  const float reference_speed;
  const bool positive_dir;
  const bool invert_enc_count;
  const bool invert_endstop;
  const float precision_threshold;
  float curr_steps;
  float curr_steps_per_sec;
  float step_loss_threshold;
  bool emergency_stop;
  // for smooth operation use this PID controller to control the motor
  PIDController pid_controller;
  SCurveMotionController motion_controller;

  // call once to setup everything
  void init(void);
  // drive towards endstop with constant speed
  void reference(void);
  // drive away from endstop with constant speed
  void out_of_endstop(void);
  // use this to move axis with constant speed or combinded with pid controller
  // automatically releases motor breaks if defined
  void set_speed(float steps_per_sec);
  // get current speed in steps per second
  float get_curr_steps_per_sec(void);
  /*
   * Set either DIR_CW or DIR_CCW
   * mot_yaw:
   * mot_z: DIR_CW -> move towards side of motor/endstop
   * mot_x: DIR_CW -> move towards endstop
   * mot_y: DIR_CW
   */
  void set_dir(bool clock_wise);
  // used internally when speed is set.
  void set_break(bool should_break);
  // call this via ISR when encoder N is read
  void update_abs_position(void);
  // gets absolute pos (in steps), only useful once referenced
  float get_absolute_pos(void);
  // call this via ISR when endstop is hit
  void endstop_hit(void);

  // convert from steps to the translated unit
  // typically mm or degree, depending on the semantics of gear ratio
  float steps_to_unit(float steps);
  float unit_to_steps(float steps);
  // internal helpers
  void set_zero_pos(void);
  float encoder_count_to_steps();
  void update_curr_steps(float dt);
};
extern StepperMotorSetup mot_x;
extern StepperMotorSetup mot_yaw;
extern StepperMotorSetup mot_z;
extern StepperMotorSetup mot_u;
} // namespace stepper_motors
#endif
