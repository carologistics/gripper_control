// Copyright (c) 2025 Carologistics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "stepper_motors.h"
#include <config/pid_config.h>
#include <config/pinout.h>
#include <config/s_controller_config.h>
#include <config/stepper_config.h>
#include <limits>
#include <timer_setup/timer_setup.h>

void StepperMotorSetup::init(void) {
  setup_pwm(pwm, pwm_prescaler);
  setup_encoder(encoder_a, encoder_b, 8 * encoder_rev_count,
                invert_enc_count); // TODO reset value
  pinMode(enable_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  digitalWrite(enable_pin, HIGH);
  digitalWrite(dir_pin, HIGH);
  encoder_a.get_timer()->CNT = 4 * encoder_rev_count;
  if (break_pin != NO_BREAK_PIN) {
    pinMode(break_pin, OUTPUT);
  }
}

void StepperMotorSetup::set_break(bool should_break) {
  if (break_pin == NO_BREAK_PIN) {
    return;
  } else {
    digitalWrite(break_pin, !should_break);
  }
}
void StepperMotorSetup::set_speed(float steps_per_sec) {
  if (steps_per_sec == 0) {
    set_pwm(pwm, 0, 0);
    set_break(true);
    return;
  }
  set_break(false);
  float tick_speed = (pwm_freq / pwm_prescaler); // Mhz
  uint16_t tim_max = min(std::numeric_limits<uint16_t>::max() - 1,
                         tick_speed / (steps_per_sec * micro_steps));
  uint16_t duty_cycle = tim_max * 0.5f;
  set_pwm(pwm, tim_max + 1, duty_cycle);
}

void StepperMotorSetup::set_dir(bool clock_wise) {
  digitalWrite(dir_pin, clock_wise);
  direction = clock_wise;
}

float StepperMotorSetup::encoder_count_to_steps() {
  return ((static_cast<int32_t>(encoder_a.get_timer()->CNT) -
           4 * encoder_rev_count) *
          (steps_per_rev / (float)encoder_rev_count));
}

void StepperMotorSetup::update_abs_position(void) {

  float curr_steps = encoder_count_to_steps();
  /**
   * whenever the first encoder rev is observed, we compute the actual delta
   * and correct all readings with observed offset
   * we do not use the actual direction, instead we determine dir by looking at
   * the sign of the current steps to account for dir change by force
   * e.g., pushing a forward moving axis in the opposite direction.
   */
  if (abs_position_step_offset == 0) {
    encoder_endstop_delta = curr_steps;
    abs_position_step_offset = encoder_endstop_delta;
    encoder_a.get_timer()->CNT = 4 * encoder_rev_count;
    return;
  }

  // Adjust absolute position offset based on direction
  int rev_count = (int)((abs(curr_steps) / steps_per_rev) + 0.5);
  abs_position_step_offset +=
      (curr_steps > 0) ? rev_count * steps_per_rev : -rev_count * steps_per_rev;

  encoder_a.get_timer()->CNT = 4 * encoder_rev_count;
}

void StepperMotorSetup::set_zero_pos(void) {
  abs_position_step_offset = 0;
  encoder_a.get_timer()->CNT = 4 * encoder_rev_count;
}

float StepperMotorSetup::get_absolute_pos(void) {
  return abs_position_step_offset + encoder_count_to_steps();
}

void StepperMotorSetup::endstop_hit(void) {
  set_zero_pos();
  set_speed(0);
}

void StepperMotorSetup::reference(void) {
  set_dir(reference_dir);
  set_speed(reference_speed);
}

float StepperMotorSetup::steps_to_unit(float steps) {
  return steps * rot_per_step / gear_ratio;
}

float StepperMotorSetup::unit_to_steps(float unit) {
  return unit * gear_ratio / rot_per_step;
}

// Motor X is actually Encoder X and vice versa, so this makes no sense and
// should be avoided for now
StepperMotorSetup mot_x = {
    .pwm = {PortID::PORT_A, 7, MOT_X_PWM_PIN, TimerID::ID_TIM14,
            TimerChannel::CHANNEL_1, 9}, // pwm
    .encoder_a = {PortID::PORT_K, 1, MOT_X_ENC_PIN_A, TimerID::ID_TIM1,
                  TimerChannel::CHANNEL_1, 1}, // encoder a
    .encoder_b = {PortID::PORT_J, 11, MOT_X_ENC_PIN_B, TimerID::ID_TIM1,
                  TimerChannel::CHANNEL_2, 1}, // encoder_b
    .encoder_n_pin = D0,                       // it is D0/PB7
    .micro_steps = MOT_X_MICROSTEPS,
    .gear_ratio = MOT_X_GEAR_RATIO,
    .encoder_rev_count = MOT_X_ENC_REV_COUNT,
    .encoder_endstop_delta = 0,
    .rot_per_step = MOT_X_ROT_PER_STEP,
    .enable_pin = MOT_X_ENABLE_PIN,
    .dir_pin = MOT_X_DIR_PIN,
    .endstop_pin = MOT_X_ENDSTOP_PIN,
    .break_pin = MOT_X_BREAK_PIN,
    .pwm_prescaler = MOT_X_PWM_PRESCALER,
    .pwm_freq = MOT_X_PWM_TIM_FREQ,
    .steps_per_rev = MOT_X_STEPS_PER_REV,
    .direction = DIR_CW,
    .abs_position_step_offset = 0,
    .reference_dir = MOT_X_REFERENCE_DIR,
    .reference_speed = MOT_X_REFERENCE_SPEED,
    .positive_dir = MOT_X_POSITIVE_DIR,
    .invert_enc_count = MOT_X_INVERT_ENC_COUNT,
    .invert_endstop = MOT_X_INVERT_ENDSTOP,
    .precision_threshold = MOT_X_PRECISION_THRESHOLD,
    .pid_controller = {MOT_X_PID_P, MOT_X_PID_I, MOT_X_PID_D, MOT_X_MAX_ACCEL,
                       MOT_X_MAX_SPEED, MOT_X_MIN_SPEED},
    .motion_controller = {MOT_X_MOTION_MAX_SPEED, MOT_X_MOTION_MAX_ACCEL,
                          MOT_X_MOTION_MAX_JERK},
};

StepperMotorSetup mot_yaw = {
    .pwm = {PortID::PORT_A, 2, MOT_YAW_PWM_PIN, TimerID::ID_TIM15,
            TimerChannel::CHANNEL_1, 4}, // pwm
    .encoder_a = {PortID::PORT_C, 6, MOT_YAW_ENC_PIN_A, TimerID::ID_TIM3,
                  TimerChannel::CHANNEL_1, 2}, // encoder a
    .encoder_b = {PortID::PORT_C, 7, MOT_YAW_ENC_PIN_B, TimerID::ID_TIM3,
                  TimerChannel::CHANNEL_2, 2}, // encoder_b
    .encoder_n_pin = D14,                      // encoder_n it is D14/PG14
    .micro_steps = MOT_YAW_MICROSTEPS,
    .gear_ratio = MOT_YAW_GEAR_RATIO,
    .encoder_rev_count = MOT_YAW_ENC_REV_COUNT,
    .encoder_endstop_delta = 0,
    .rot_per_step = MOT_YAW_ROT_PER_STEP,
    .enable_pin = MOT_YAW_ENABLE_PIN,
    .dir_pin = MOT_YAW_DIR_PIN,
    .endstop_pin = MOT_YAW_ENDSTOP_PIN,
    .break_pin = MOT_YAW_BREAK_PIN,
    .pwm_prescaler = MOT_YAW_PWM_PRESCALER,
    .pwm_freq = MOT_YAW_PWM_TIM_FREQ,
    .steps_per_rev = MOT_YAW_STEPS_PER_REV,
    .direction = DIR_CW,
    .abs_position_step_offset = 0,
    .reference_dir = MOT_YAW_REFERENCE_DIR,
    .reference_speed = MOT_YAW_REFERENCE_SPEED,
    .positive_dir = MOT_YAW_POSITIVE_DIR,
    .invert_enc_count = MOT_YAW_INVERT_ENC_COUNT,
    .invert_endstop = MOT_YAW_INVERT_ENDSTOP,
    .precision_threshold = MOT_YAW_PRECISION_THRESHOLD,
    .pid_controller = {MOT_YAW_PID_P, MOT_YAW_PID_I, MOT_YAW_PID_D,
                       MOT_YAW_MAX_ACCEL, MOT_YAW_MAX_SPEED, MOT_YAW_MIN_SPEED},
    .motion_controller = {MOT_YAW_MOTION_MAX_SPEED, MOT_YAW_MOTION_MAX_ACCEL,
                          MOT_YAW_MOTION_MAX_JERK}};

StepperMotorSetup mot_z = {
    .pwm = {PortID::PORT_B, 8, MOT_Z_PWM_PIN, TimerID::ID_TIM16,
            TimerChannel::CHANNEL_1, 1}, // pwm
    .encoder_a = {PortID::PORT_J, 8, MOT_Z_ENC_PIN_A, TimerID::ID_TIM8,
                  TimerChannel::CHANNEL_1, 3}, // encoder a
    .encoder_b = {PortID::PORT_J, 10, MOT_Z_ENC_PIN_B, TimerID::ID_TIM8,
                  TimerChannel::CHANNEL_2, 3}, // encoder_b
    .encoder_n_pin = D24,                      // encoder_n TODO, it is D24/PG12
    .micro_steps = MOT_Z_MICROSTEPS,
    .gear_ratio = MOT_Z_GEAR_RATIO,
    .encoder_rev_count = MOT_Z_ENC_REV_COUNT,
    .encoder_endstop_delta = 0,
    .rot_per_step = MOT_Z_ROT_PER_STEP,
    .enable_pin = MOT_Z_ENABLE_PIN,
    .dir_pin = MOT_Z_DIR_PIN,
    .endstop_pin = MOT_Z_ENDSTOP_PIN,
    .break_pin = MOT_Z_BREAK_PIN,
    .pwm_prescaler = MOT_Z_PWM_PRESCALER,
    .pwm_freq = MOT_Z_PWM_TIM_FREQ,
    .steps_per_rev = MOT_Z_STEPS_PER_REV,
    .direction = DIR_CW,
    .abs_position_step_offset = 0,
    .reference_dir = MOT_Z_REFERENCE_DIR,
    .reference_speed = MOT_Z_REFERENCE_SPEED,
    .positive_dir = MOT_Z_POSITIVE_DIR,
    .invert_enc_count = MOT_Z_INVERT_ENC_COUNT,
    .invert_endstop = MOT_Z_INVERT_ENDSTOP,
    .precision_threshold = MOT_Z_PRECISION_THRESHOLD,
    .pid_controller = {MOT_Z_PID_P, MOT_Z_PID_I, MOT_Z_PID_D, MOT_Z_MAX_ACCEL,
                       MOT_Z_MAX_SPEED, MOT_Z_MIN_SPEED},
    .motion_controller = {MOT_Z_MOTION_MAX_SPEED, MOT_Z_MOTION_MAX_ACCEL,
                          MOT_Z_MOTION_MAX_JERK}};

StepperMotorSetup mot_u = {
    .pwm = {PortID::PORT_B, 9, MOT_U_PWM_PIN, TimerID::ID_TIM17,
            TimerChannel::CHANNEL_1, 1}, // pwm
    .encoder_a = {PortID::PORT_A, 0, MOT_U_ENC_PIN_A, TimerID::ID_TIM2,
                  TimerChannel::CHANNEL_1, 1}, // encoder a
    .encoder_b = {PortID::PORT_A, 1, MOT_U_ENC_PIN_B, TimerID::ID_TIM2,
                  TimerChannel::CHANNEL_2, 1}, // encoder_b
    .encoder_n_pin = D23,
    .micro_steps = MOT_U_MICROSTEPS,
    .gear_ratio = MOT_U_GEAR_RATIO,
    .encoder_rev_count = MOT_U_ENC_REV_COUNT,
    .encoder_endstop_delta = 0,
    .rot_per_step = MOT_U_ROT_PER_STEP,
    .enable_pin = MOT_U_ENABLE_PIN,
    .dir_pin = MOT_U_DIR_PIN,
    .endstop_pin = MOT_U_ENDSTOP_PIN,
    .break_pin = MOT_U_BREAK_PIN,
    .pwm_prescaler = MOT_U_PWM_PRESCALER,
    .pwm_freq = MOT_U_PWM_TIM_FREQ,
    .steps_per_rev = MOT_U_STEPS_PER_REV,
    .direction = DIR_CW,
    .abs_position_step_offset = 0,
    .reference_dir = MOT_U_REFERENCE_DIR,
    .reference_speed = MOT_U_REFERENCE_SPEED,
    .positive_dir = MOT_U_POSITIVE_DIR,
    .invert_enc_count = MOT_U_INVERT_ENC_COUNT,
    .invert_endstop = MOT_U_INVERT_ENDSTOP,
    .precision_threshold = MOT_U_PRECISION_THRESHOLD,
    .pid_controller = {MOT_U_PID_P, MOT_U_PID_I, MOT_U_PID_D, MOT_U_MAX_ACCEL,
                       MOT_U_MAX_SPEED, MOT_U_MIN_SPEED},
    .motion_controller = {MOT_U_MOTION_MAX_SPEED, MOT_U_MOTION_MAX_ACCEL,
                          MOT_U_MOTION_MAX_JERK}};
