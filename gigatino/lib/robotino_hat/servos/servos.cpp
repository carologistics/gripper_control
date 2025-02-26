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

#include <config/pinout.h>
#include <config/servo_config.h>
#include <servos/servos.h>

void ServoSetup::init(void) {
  setup_pwm(pwm, pwm_prescaler);
  set_position(min_angle);
  set_pwm(pwm, timer_freq / pwm_prescaler / pwm_target_freq, pwm_threshold);
}

void ServoSetup::set_position(float angle) {
  angle = constrain(angle, min_angle, max_angle);
  uint16_t pulse_width =
      map(angle, min_angle, max_angle, min_pulse_width, max_pulse_width);
  set_pwm(pwm, timer_freq / pwm_prescaler / pwm_target_freq, pulse_width);
}

ServoSetup servo_gripper = {
    .pwm = {PortID::PORT_D, 13, SERVO_GRIPPER_PWM_PIN, TimerID::ID_TIM4,
            TimerChannel::CHANNEL_2, 2},
    .min_angle = SERVO_GRIPPER_MIN_ANGLE,
    .min_pulse_width = SERVO_GRIPPER_PWM_MIN_PULSE_WIDTH,
    .max_angle = SERVO_GRIPPER_MAX_ANGLE,
    .max_pulse_width = SERVO_GRIPPER_PWM_MAX_PULSE_WIDTH,
    .timer_freq = SERVO_GRIPPER_PWM_TIM_FREQ,
    .pwm_prescaler = SERVO_GRIPPER_PWM_PRESCALER,
    .pwm_target_freq = SERVO_GRIPPER_PWM_TARGET_FREQ,
    .pwm_threshold = SERVO_GRIPPER_MIN_ANGLE};

ServoSetup servo_rotation = {
    .pwm = {PortID::PORT_I, 0, SERVO_ROTATION_PWM_PIN, TimerID::ID_TIM5,
            TimerChannel::CHANNEL_4, 2},
    .min_angle = SERVO_ROTATION_MIN_ANGLE,
    .min_pulse_width = SERVO_ROTATION_PWM_MIN_PULSE_WIDTH,
    .max_angle = SERVO_ROTATION_MAX_ANGLE,
    .max_pulse_width = SERVO_ROTATION_PWM_MAX_PULSE_WIDTH,
    .timer_freq = SERVO_ROTATION_PWM_TIM_FREQ,
    .pwm_prescaler = SERVO_ROTATION_PWM_PRESCALER,
    .pwm_target_freq = SERVO_ROTATION_PWM_TARGET_FREQ,
    .pwm_threshold = SERVO_ROTATION_MIN_ANGLE};
