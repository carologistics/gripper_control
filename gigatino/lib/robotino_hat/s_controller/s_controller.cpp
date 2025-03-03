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

#include "s_controller.h"
#include <Arduino.h>

void SCurveMotionController::set_dist(float dist) {
  dist = abs(dist);
  peak_speed_ = sqrt(max_accel_ * dist);
}

float SCurveMotionController::compute(float target, float current, float dt) {
  float distance_remaining = target - current;

  // Compute braking distance based on current acceleration & speed
  float braking_distance = (peak_speed_ * peak_speed_) / (2 * max_accel_);

  if (distance_remaining == 0) {
    speed_ = 0;
    accel_ = 0;
  } else if (distance_remaining > 0) {
    // Moving forward
    if (braking_distance >= distance_remaining) {
      // Deceleration phase
      accel_ -= max_jerk_ * dt;
    } else {
      // Acceleration phase
      accel_ += max_jerk_ * dt;
    }
  } else {
    // Moving backward
    if (braking_distance <= -distance_remaining) {
      accel_ += max_jerk_ * dt; // Decelerate (negative direction)
    } else {
      accel_ -= max_jerk_ * dt; // Accelerate (negative direction)
    }
  }

  // Constrain acceleration within limits
  accel_ = constrain(accel_, -max_accel_, max_accel_);

  // Compute velocity using constrained acceleration
  speed_ += accel_ * dt;
  speed_ = constrain(speed_, -peak_speed_, peak_speed_);

  return speed_;
}

void SCurveMotionController::reset() {
  speed_ = 0;
  accel_ = 0;
}
