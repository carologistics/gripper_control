#pragma once

#include <Arduino.h>
enum TimerID {
  ID_TIM1,
  ID_TIM2,
  ID_TIM3,
  ID_TIM4,
  ID_TIM5,
  ID_TIM8,
  ID_TIM14,
  ID_TIM15,
  ID_TIM16,
  ID_TIM17,
  ID_TIM_COUNT
};

enum PortID {
  PORT_A,
  PORT_B,
  PORT_C,
  PORT_D,
  PORT_E,
  PORT_F,
  PORT_G,
  PORT_H,
  PORT_I,
  PORT_J,
  PORT_K,
  PORT_COUNT
};

enum TimerChannel { CHANNEL_1, CHANNEL_2, CHANNEL_3, CHANNEL_4, CHANNEL_COUNT };

// Pin-to-Timer structure

/**
 * Pin-to-Timer mapping interface.
 *
 * General information needed:
 * 1. find out which port and pin maps to the giga pin
 *    that can access the desired timer using giga full pinout
 *    Example: Giga Pin D64 on STM32 Port H Pin 10 can use TIM5 channel 1
 * 2. check the alternative function index using stm32 datasheet p 89ff
 *    Example (continued): PH10 TIM5 ch1 is AF 2
 * -> TimerPinSetup foo = {PortID::PORT_H, 10, D64, TimerID::ID_TIM5,
 * TimerChannel::CHANNEL_1, 2}
 */
struct TimerPinSetup {
  PortID port;
  uint8_t pin;          // stm32 pin number belonging to port (0-15)
  uint32_t arduino_pin; // Arduino pin number (e.g., A2, D62)
  TimerID timer;        // Associated timer
  TimerChannel channel; // Associated timer channel
  uint8_t alternateFn;  // Alternate function for the GPIO pin

  GPIO_TypeDef *get_gpio_port();
  TIM_TypeDef *get_timer();
};
/*
 * Tested Configurations for encoder mode:
 * TIM 1
 * TimerPinSetup a = {PortID::PORT_K, 1, D10, TimerID::ID_TIM1,
 * TimerChannel::CHANNEL_1, 1}; TimerPinSetup b = {PortID::PORT_J, 11, D12,
 * TimerID::ID_TIM1, TimerChannel::CHANNEL_2, 1};
 *
 * TIM 2
 * TimerPinSetup a = {PortID::PORT_A, 0, A7, TimerID::ID_TIM2,
 * TimerChannel::CHANNEL_1, 1}; TimerPinSetup b = {PortID::PORT_A, 1, D66,
 * TimerID::ID_TIM2, TimerChannel::CHANNEL_2, 1}; TimerPinSetup a =
 * {PortID::PORT_A, 5, D85, TimerID::ID_TIM2, TimerChannel::CHANNEL_1, 1};
 * TimerPinSetup b = {PortID::PORT_A, 1, D66, TimerID::ID_TIM2,
 * TimerChannel::CHANNEL_2, 1};
 *
 * TIM 3
 * TimerPinSetup a = {PortID::PORT_C, 6, D68, TimerID::ID_TIM3,
 * TimerChannel::CHANNEL_1, 2}; TimerPinSetup b = {PortID::PORT_C, 7, D15,
 * TimerID::ID_TIM3, TimerChannel::CHANNEL_2, 2};
 *
 * TimerPinSetup a = {PortID::PORT_C, 6, D68, TimerID::ID_TIM3,
 * TimerChannel::CHANNEL_1, 2}; TimerPinSetup b = {PortID::PORT_A, 7, D5,
 * TimerID::ID_TIM3, TimerChannel::CHANNEL_2, 2};
 *
 * TIM 4
 * TimerPinSetup a = {PortID::PORT_B, 6, D101, TimerID::ID_TIM4,
 * TimerChannel::CHANNEL_1, 2}; TimerPinSetup b = {PortID::PORT_D, 13, D6,
 * TimerID::ID_TIM4, TimerChannel::CHANNEL_2, 2};
 *
 * TIM 5
 * TimerPinSetup a = {PortID::PORT_H, 10, D64, TimerID::ID_TIM5,
 * TimerChannel::CHANNEL_1, 2}; TimerPinSetup b = {PortID::PORT_H, 11, D63,
 * TimerID::ID_TIM5, TimerChannel::CHANNEL_2, 2};
 *
 * TIM 8
 * TimerPinSetup a = {PortID::PORT_J, 8, D4, TimerID::ID_TIM8,
 * TimerChannel::CHANNEL_1, 3}; TimerPinSetup b = {PortID::PORT_J, 10, D11,
 * TimerID::ID_TIM8, TimerChannel::CHANNEL_2, 3};
 *
 * Tested configurations for PWM mode:
 * TIM 14
 * TimerPinSetup pwm = {PortID::PORT_A, 7, D5, TimerID::ID_TIM14,
 * TimerChannel::CHANNEL_1, 9};
 *
 * TIM 15
 * TimerPinSetup pwm = {PortID::PORT_A, 2, D3, TimerID::ID_TIM15,
 * TimerChannel::CHANNEL_1, 4};
 *
 * TIM 16
 * TimerPinSetup pwm = {PortID::PORT_B, 8, D8, TimerID::ID_TIM16,
 * TimerChannel::CHANNEL_1, 1};
 *
 * TIM 17
 * TimerPinSetup pwm = {PortID::PORT_B, 9, D9, TimerID::ID_TIM17,
 * TimerChannel::CHANNEL_1, 1};
 *
 * TIM 3
 * TimerPinSetup pwm = {PortID::PORT_B, 0, A2, TimerID::ID_TIM3,
 * TimerChannel::CHANNEL_3, 2};
 */

/*
 * Enable the clock for a specific timer.
 *
 * Do not call this function directly, use setup_pwm or setup_encoder instead.
 * Maps peripheral (timer) to bus via table 7 (p 138) of RM and then sets the
 * appropriate enable bit and then delays to ensure subsequent operations can
 * rely on the timer being actually active.
 */
void enable_timer_clock(TimerID timer);

/*
* Configure a pin for usage with a timer channel.

* Do not call this function directly, use setup_pwm or setup_encoder instead.
* This involves setting the AF bits in MODER and the AF number in AFR.
*/
void setup_gpio_alternative_function(TimerPinSetup setup);

/*
 * Start a timer.
 *
 * Do not call this function directly, use setup_pwm or setup_encoder instead.
 * This involves setting the enable bit in BDTR and CR1 as well es triggering
 * an update event via EGR.
 */
void start_timer(TIM_TypeDef *timer);

/*
* Setup a given pin to use the timer to generate a PWM signal.

* This involves setting the timers PSC, ARR, CCMRx, CCRx, CCER, and CR1
* registers (where x is the channel number).
* Also calls start_timer, hence no need to call it again.

* @param reload: Determines the frequency of the PWM, depending on the counting
speed of the clock.
*                Higher values result in lower frequency.
* @duty_cycle:   value between 0 and reload-1 from which the PWM signal is high.
*/
void setup_pwm(TimerPinSetup &p, uint16_t prescaler);

/*
 * Setup a pin pair to act as encoder inputs processed by a timer.
 *
 * Requires pin a to be channel 1 of the timer and pin b to be channel 2.
 * It is not possible to use channels 3 and 4 for this as only 1 and 2 are wired
 * to the encoder feature.
 *
 * This involves setting the timers PSC, ARR, CCMRx, CCRx, CCER, and CR1
 * registers (where x is the channel number).
 *
 * @param reset: Set the reset value of the counter, value depends on method of
 * counting.
 */
void setup_encoder(TimerPinSetup &a, TimerPinSetup &b, uint16_t reset,
                   bool inverted = false);

void set_pwm(TimerPinSetup &p, uint16_t reload, uint16_t duty_cycle);
