#pragma once

#ifndef SERVOS_H_
#define SERVOS_H_
#include <timer_setup/timer_setup.h>

namespace servos {
struct ServoSetup {
  TimerPinSetup pwm;
  const float min_angle;
  const uint16_t min_pulse_width;
  const float max_angle;
  const uint16_t max_pulse_width;

  const uint32_t timer_freq;
  const uint16_t pwm_prescaler;
  const uint32_t pwm_target_freq;
  const uint16_t pwm_threshold;
  float approx_angle; // approximate using elapsed time and assuming constant
                      // speed
  const float speed;  // in deg per millisecond

  // call once to setup everything
  void init(void);
  // use this to move servo
  void set_position(float angle);
};

extern ServoSetup servo_gripper;
extern ServoSetup servo_rotation;
} // namespace servos

#endif // SERVOS_H_
