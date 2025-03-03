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

#include "pid_controller.h"
#include <Arduino.h>

void PIDController::reset() {
  integral_ = 0;
  prev_error_ = 0;
  prev_speed_ = 0;
}

float PIDController::compute(float target, float current, float dt) {
  float error = target - current;
  integral_ += error * dt;
  float derivative = (error - prev_error_) / dt;
  prev_error_ = error;

  // Compute raw PID output (desired speed)
  float target_speed = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;

  // Apply acceleration limiting
  float speed_diff = target_speed - prev_speed_;

  if (speed_diff > max_accel_ * dt) {
    target_speed = prev_speed_ + max_accel_ * dt;
  } else if (speed_diff < -max_accel_ * dt) {
    target_speed = prev_speed_ - max_accel_ * dt;
  }
  if (target_speed > 0) {
    target_speed = constrain(target_speed, min_speed_, max_speed_);
  } else {
    target_speed = constrain(target_speed, -max_speed_, -min_speed_);
  }
  // Store previous speed
  prev_speed_ = target_speed;

  return target_speed;
}
