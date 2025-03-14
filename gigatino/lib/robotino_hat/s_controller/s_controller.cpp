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

void SCurveMotionController::plan_curve(float start_pos, float target_pos) {
  finished_ = false;
  Serial.print("Plan curve at start ");
  Serial.print(start_pos);
  Serial.print(" to target ");
  Serial.print(target_pos);
  Serial.print(" short dist ");
  Serial.println(short_dist_);
  start_pos_ = start_pos;
  float dist = max(0.0f, abs(target_pos - start_pos) - short_dist_);
  forward_ = target_pos > start_pos;
  peak_accel_ = max_accel_;
  peak_speed_ = max_speed_;
  dist_phase_1_ = 0.0;

  // Calculation:
  float t_1 = peak_accel_ / max_jerk_;
  float d_1 = max_jerk_ / 6.0f * std::pow(t_1, 3);
  float v_1 = max_jerk_ / 2.0f * std::pow(t_1, 2);
  float d_decel = v_1 * t_1 + max_jerk_ * std::pow(t_1, 3) / 6.0f;

  // Check if we will not reach max acceleration at all
  if (2 * d_1 + 2 * d_decel > dist) {
    Serial.println("Short distance");
    // 0.6f = 3/5
    peak_accel_ = std::pow(dist * 0.6f * std::pow(max_jerk_, 2), 1.0f / 3.0f);
    t_1 = peak_accel_ / max_jerk_;
    d_1 = max_jerk_ / 6.0 * std::pow(t_1, 3);
    v_1 = max_jerk_ / 2.0 * std::pow(t_1, 2);
    d_decel = v_1 * t_1 + max_jerk_ / 6.0f * std::pow(t_1, 3);

    dist_phase_2_ = 2 * dist; // disable phase
    dist_phase_3_ = d_1;
    dist_phase_4_ = 2 * dist; // disable phase
    dist_phase_5_ = dist_phase_3_ + d_decel;
    dist_phase_6_ = 2 * dist; // disable phase
    dist_phase_7_ = dist_phase_5_ + d_decel;
    return;
  }
  dist_phase_2_ = d_1;
  // Speed to decelerate with linear jerk increase until a_max
  float v_2 = max_speed_ - v_1;
  float t_12 = (v_2 - v_1) / peak_accel_;
  float d_peak_accel_ = v_1 * t_12 + peak_accel_ / 2.0f * std::pow(t_12, 2);
  // float t_2 = t_12 + t_1;
  float d_2 = d_peak_accel_ + d_1;
  dist_phase_3_ = d_2;

  // Compute deceleration from max speed with 0 accel to peak_accel_
  d_decel = peak_speed_ * t_1 - max_jerk_ / 6.0f * std::pow(t_1, 3);
  float d_3 = d_2 + d_decel;

  // Throttle max speed as we cannot reach true max speed
  if (d_3 >= dist / 2.0) {
    peak_speed_ =
        std::sqrt(
            2 * ((dist / 2.0f) - d_1 + max_jerk_ / 6.0f * std::pow(t_1, 3)) *
                peak_accel_ +
            peak_accel_ * t_1 - v_1) -
        t_1 * max_accel_ + v_1;

    // Re-do calculations for d_2 and d_3 with new peak speed:
    v_2 = peak_speed_ - v_1;
    t_12 = (v_2 - v_1) / peak_accel_;
    d_peak_accel_ = v_1 * t_12 + peak_accel_ / 2.0f * std::pow(t_12, 2);
    // t_2 = t_12 + t_1;
    d_2 = d_peak_accel_ + d_1;
    dist_phase_3_ = d_2;

    d_decel = peak_speed_ * t_1 - max_jerk_ / 6.0f * std::pow(t_1, 3);
    d_3 = d_2 + d_decel;
  }
  dist_phase_4_ = d_3;
  dist_phase_7_ = dist - d_1;
  dist_phase_6_ = dist - d_2;
  dist_phase_5_ = dist - d_3;
  Serial.print(" p1: ");
  Serial.print(dist_phase_1_);
  Serial.print(" p2: ");
  Serial.print(dist_phase_2_);
  Serial.print(" p3: ");
  Serial.print(dist_phase_3_);
  Serial.print(" p4: ");
  Serial.print(dist_phase_4_);
  Serial.print(" p5: ");
  Serial.print(dist_phase_5_);
  Serial.print(" p6: ");
  Serial.print(dist_phase_6_);
  Serial.print(" p7: ");
  Serial.println(dist_phase_7_);
}

float SCurveMotionController::compute(float target, float current, float dt) {
  if (finished_) {
    return 0.f;
  }
  float distance_traveled = abs(start_pos_ - current);
  float dist_remaining = forward_ ? target - current : current - target;
  if (abs(dist_remaining) < threshold_) {
    speed_ = 0;
    accel_ = 0;
    curr_phase_ = 8;
    finished_ = true;
    return speed_;
  }
  if (dist_remaining < 0) {
    forward_ = !forward_;
    speed_ = forward_ ? min_speed_ : -min_speed_;
    accel_ = 0;
    curr_phase_ = 8;
    return speed_;
  }
  if (distance_traveled >= dist_phase_7_) {
    curr_phase_ = 7;
    accelerate_to_zero(dt);
  } else if (distance_traveled >= dist_phase_6_) {
    curr_phase_ = 6;
    max_change(dt);
  } else if (distance_traveled >= dist_phase_5_) {
    curr_phase_ = 5;
    decelerate_from_zero(dt);
  } else if (distance_traveled >= dist_phase_4_) {
    curr_phase_ = 4;
    ((void)0); // do nothing
  } else if (distance_traveled >= dist_phase_3_) {
    curr_phase_ = 3;
    decelerate_to_zero(dt);
  } else if (distance_traveled >= dist_phase_2_) {
    curr_phase_ = 2;
    max_change(dt);
  } else {
    curr_phase_ = 1;
    accelerate_from_zero(dt);
  }
  return speed_;
}

void SCurveMotionController::accelerate_from_zero(float dt) {
  if (forward_) {
    accel_ += max_jerk_ * dt;
    accel_ = constrain(accel_, 0, 2 * peak_accel_);
  } else {
    accel_ -= max_jerk_ * dt;
    accel_ = constrain(accel_, -2 * peak_accel_, 0);
  }
  speed_ += accel_ * dt;
  if (forward_) {
    speed_ = constrain(speed_, 0, 2 * peak_speed_);
  } else {
    speed_ = constrain(speed_, -2 * peak_speed_, 0);
  }
}

// use this to reverse the change of decelleration
void SCurveMotionController::accelerate_to_zero(float dt) {
  if (forward_) {
    accel_ += max_jerk_ * dt;
    accel_ = constrain(accel_, -peak_accel_, 0);
  } else {
    accel_ -= max_jerk_ * dt;
    accel_ = constrain(accel_, 0, peak_accel_);
  }
  speed_ += accel_ * dt;
  if (forward_) {
    speed_ = constrain(speed_, min_speed_, peak_speed_);
  } else {
    speed_ = constrain(speed_, -peak_speed_, -min_speed_);
  }
}

// use this to reverse the change of acceleration
void SCurveMotionController::decelerate_to_zero(float dt) {
  if (forward_) {
    accel_ -= max_jerk_ * dt;
    accel_ = constrain(accel_, 0, peak_accel_);
  } else {
    accel_ += max_jerk_ * dt;
    accel_ = constrain(accel_, -peak_accel_, 0);
  }
  speed_ += accel_ * dt;
  if (forward_) {
    speed_ = constrain(speed_, 0, peak_speed_);
  } else {
    speed_ = constrain(speed_, -peak_speed_, 0);
  }
}
// use this to actually decelerate
void SCurveMotionController::decelerate_from_zero(float dt) {
  if (forward_) {
    accel_ -= max_jerk_ * dt;
    accel_ = constrain(accel_, -peak_accel_, 0);
  } else {
    accel_ += max_jerk_ * dt;
    accel_ = constrain(accel_, 0, peak_accel_);
    speed_ += accel_ * dt;
  }
  if (forward_) {
    speed_ = constrain(speed_, 0, peak_speed_);
  } else {
    speed_ = constrain(speed_, -peak_speed_, 0);
  }
}
void SCurveMotionController::max_change(float dt) {
  speed_ += accel_ * dt;
  if (forward_) {
    speed_ = constrain(speed_, min_speed_, peak_speed_);
  } else {
    speed_ = constrain(speed_, -peak_speed_, -min_speed_);
  }
}

void SCurveMotionController::reset() {
  speed_ = 0;
  accel_ = 0;
  peak_speed_ = max_speed_;
  peak_accel_ = max_accel_;
}
