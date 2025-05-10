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

#include "timer_setup.h"

GPIO_TypeDef *TimerPinSetup::get_gpio_port() {
  switch (port) {
  case PORT_A:
    return GPIOA;
  case PORT_B:
    return GPIOB;
  case PORT_C:
    return GPIOC;
  case PORT_D:
    return GPIOD;
  case PORT_E:
    return GPIOE;
  case PORT_F:
    return GPIOF;
  case PORT_G:
    return GPIOG;
  case PORT_H:
    return GPIOH;
  case PORT_I:
    return GPIOI;
  case PORT_J:
    return GPIOJ;
  case PORT_K:
    return GPIOK;
  default:
    return nullptr; // Invalid port, return nullptr
  }
}

TIM_TypeDef *TimerPinSetup::get_timer() {
  switch (timer) {
  case ID_TIM1:
    return TIM1;
    break;
  case ID_TIM2:
    return TIM2;
    break;
  case ID_TIM3:
    return TIM3;
    break;
  case ID_TIM4:
    return TIM4;
    break;
  case ID_TIM5:
    return TIM5;
    break;
  case ID_TIM8:
    return TIM8;
    break;
  case ID_TIM14:
    return TIM14;
    break;
  case ID_TIM15:
    return TIM15;
    break;
  case ID_TIM16:
    return TIM16;
    break;
  case ID_TIM17:
    return TIM17;
    break;
  default:
    return nullptr; // Unsupported timer
  }
}

void enable_timer_clock(TimerID timer) {
  // Uses information from table 7 (p 138) of reference manual to find out
  // which bus belongs to the peripheral (timer).
  switch (timer) {
  // cases for APB2
  case ID_TIM1:
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    break;
  case ID_TIM8:
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    break;
  case ID_TIM15:
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    break;
  case ID_TIM16:
    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
    break;
  case ID_TIM17:
    RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
    break;
  // cases for APB1
  case ID_TIM2:
    RCC->APB1LENR |= RCC_APB1LENR_TIM2EN;
    break;
  case ID_TIM3:
    RCC->APB1LENR |= RCC_APB1LENR_TIM3EN;
    break;
  case ID_TIM4:
    RCC->APB1LENR |= RCC_APB1LENR_TIM4EN;
    break;
  case ID_TIM5:
    RCC->APB1LENR |= RCC_APB1LENR_TIM5EN;
    break;
  case ID_TIM14:
    RCC->APB1LENR |= RCC_APB1LENR_TIM14EN;
    break;
  default:
    return;
  }
  // TODO: probably does not need that much time.
  delay(100);
}

void setup_gpio_alternative_function(TimerPinSetup setup) {
  GPIO_TypeDef *gpioPort = setup.get_gpio_port();
  gpioPort->MODER &= ~(0x3 << (setup.pin * 2)); // Clear mode bits for the pin
  gpioPort->MODER |= (0x2 << (setup.pin * 2));  // Set Alternate Function mode
  if (setup.pin < 8) {
    // Use AFRL (pins 0–7)
    gpioPort->AFR[0] &=
        ~(0xF << (setup.pin * 4)); // Clear existing AF bits for the pin
    gpioPort->AFR[0] |=
        (setup.alternateFn << (setup.pin * 4)); // Set new AF number
  } else if (setup.pin < 16) {
    // Use AFRH (pins 8–15)
    gpioPort->AFR[1] &=
        ~(0xF << ((setup.pin - 8) * 4)); // Clear existing AF bits for the pin
    gpioPort->AFR[1] |=
        (setup.alternateFn << ((setup.pin - 8) * 4)); // Set new AF number
  }
  // TODO: Make sure to also explicitly configure other registers like PUPDR
  //       and OTYPER
  /*
   * Serial.print("PIN: ");
   * Serial.println(setup.arduino_pin);
   * Serial.print("MODER: ");
   * Serial.println(gpioPort->MODER, HEX);
   * Serial.print("AFR0: ");
   * Serial.println(gpioPort->AFR[0], HEX);
   * Serial.print("AFR1: ");
   * Serial.println(gpioPort->AFR[1], HEX);
   * Serial.print("PUPDR: ");
   * Serial.println(gpioPort->PUPDR, HEX);
   */
}

/*
 * Start a timer.
 *
 * This involves setting the enable bit in BDTR and CR1 as well es triggering
 * an update event via EGR.
 */
void start_timer(TIM_TypeDef *timer) {
  // Mainly needed for timers with Dead-time generators to work.
  timer->BDTR |= TIM_BDTR_MOE;
  // Generate an update event to reload the prescaler value immediately
  timer->EGR = TIM_EGR_UG;
  // Start the timer
  timer->CR1 |= TIM_CR1_CEN;
  // TODO: probably does not need that much time.
  delay(100);
}

void setup_pwm(TimerPinSetup &p, uint16_t prescaler) {
  enable_timer_clock(p.timer);
  pinMode(p.arduino_pin, OUTPUT);
  setup_gpio_alternative_function(p);
  TIM_TypeDef *timer = p.get_timer();

  // Configure timer prescaler and auto-reload value
  timer->PSC = prescaler - 1; // Prescaler (PSC + 1)
  // Configure the specific channel for PWM
  uint32_t channel = p.channel;
  switch (channel) {
  case CHANNEL_1:
    timer->CCMR1 &= ~(TIM_CCMR1_OC1M); // Clear output compare mode bits
    timer->CCMR1 |= (0x6 << TIM_CCMR1_OC1M_Pos); // Set PWM mode 1
    timer->CCMR1 |= TIM_CCMR1_OC1PE;             // Enable preload
    timer->CCER |= TIM_CCER_CC1E;                // Enable output
    break;
  case CHANNEL_2:
    timer->CCMR1 &= ~(TIM_CCMR1_OC2M); // Clear output compare mode bits
    timer->CCMR1 |= (0x6 << TIM_CCMR1_OC2M_Pos); // Set PWM mode 1
    timer->CCMR1 |= TIM_CCMR1_OC2PE;             // Enable preload
    timer->CCER |= TIM_CCER_CC2E;                // Enable output
    break;
  case CHANNEL_3:
    timer->CCMR2 &= ~(TIM_CCMR2_OC3M); // Clear output compare mode bits
    timer->CCMR2 |= (0x6 << TIM_CCMR2_OC3M_Pos); // Set PWM mode 1
    timer->CCMR2 |= TIM_CCMR2_OC3PE;             // Enable preload
    timer->CCER |= TIM_CCER_CC3E;                // Enable output
    break;
  case CHANNEL_4:
    timer->CCMR2 &= ~(TIM_CCMR2_OC4M); // Clear output compare mode bits
    timer->CCMR2 |= (0x6 << TIM_CCMR2_OC4M_Pos); // Set PWM mode 1
    timer->CCMR2 |= TIM_CCMR2_OC4PE;             // Enable preload
    timer->CCER |= TIM_CCER_CC4E;                // Enable output
    break;
  default:
    return; // Unsupported channel
  }
  set_pwm(p, 0, 0);
  // Enable auto-reload preload
  timer->CR1 |= TIM_CR1_ARPE;
  start_timer(timer);
}

void set_pwm(TimerPinSetup &p, uint16_t reload, uint16_t duty_cycle) {
  TIM_TypeDef *timer = p.get_timer();
  timer->ARR = reload;
  // duty cycle depends on channel
  uint32_t channel = p.channel;
  switch (channel) {
  case CHANNEL_1:
    timer->CCR1 = duty_cycle;
    break;
  case CHANNEL_2:
    timer->CCR2 = duty_cycle;
    break;
  case CHANNEL_3:
    timer->CCR3 = duty_cycle;
    break;
  case CHANNEL_4:
    timer->CCR3 = duty_cycle;
    break;
  default:
    return; // Unsupported channel
  }
  timer->EGR = TIM_EGR_UG;
}

void setup_encoder(TimerPinSetup &a, TimerPinSetup &b, uint16_t reset,
                   bool inverted) {
  // Enable the clock for TIM3
  if (a.timer != b.timer) {
    // Serial.println("Setup of encoder requires a and b to share a timer");
    return;
  }
  if (a.channel != TimerChannel::CHANNEL_1 ||
      b.channel != TimerChannel::CHANNEL_2) {
    // Serial.println("Setup of encoder requires a to use channel 1 and b to use
    // channel 2");
    return;
  }
  // Serial.println("Encoder Setup:");
  enable_timer_clock(a.timer);
  pinMode(a.arduino_pin, INPUT);
  pinMode(b.arduino_pin, INPUT);
  setup_gpio_alternative_function(a);
  setup_gpio_alternative_function(b);

  TIM_TypeDef *timer = a.get_timer();
  timer->CR1 &= ~(TIM_CR1_CEN); // make sure timer is disabled while configuring
  delay(100);

  timer->PSC = 0;
  timer->CCER &= ~(TIM_CCER_CC1E); // disable capture-compare
  timer->CCER &= ~(TIM_CCER_CC2E); // disable capture-compare
  timer->DCR = 0x0;                // clear DMA register

  timer->DMAR = 0x0; // clear DMA register

  // Configure Channels in Encoder Mode
  timer->CCMR1 &= ~(TIM_CCMR1_CC1S);
  timer->CCMR1 &= ~(TIM_CCMR1_CC2S);
  timer->CCMR1 |=
      (0x1 << TIM_CCMR1_CC1S_Pos); // ch1 Input with IC1 mapped to TI1
  timer->CCMR1 |=
      (0x1 << TIM_CCMR1_CC2S_Pos);     // ch2 Input with IC2 mapped to TI2
  timer->CCMR1 &= ~(TIM_CCMR1_IC1F);   // disable filtering ch1
  timer->CCMR1 &= ~(TIM_CCMR1_IC1PSC); // disable prescaler ch1
  timer->CCMR1 &= ~(TIM_CCMR1_IC2F);   // disable filtering ch2
  timer->CCMR1 &= ~(TIM_CCMR1_IC2PSC); // disable prescaler ch2

  timer->DIER = 0x0; // clear DIER just to be safe ...
                     // sometimes default values are not 0x0 even though it is
                     // explicitly stated in reference manual ...
  timer->CCER &= ~(TIM_CCER_CC1P);               // non-inverted ch1
  timer->CCER |= (inverted ? TIM_CCER_CC1P : 0); // invert ch1 if needed
  timer->CCER &= ~(TIM_CCER_CC1NP);              // non-inverted ch1
  timer->CCER &= ~(TIM_CCER_CC2P);               // non-inver:wted ch2
  timer->CCER &= ~(TIM_CCER_CC2NP);              // non-inverted ch2
  timer->ARR = reset;
  timer->SMCR &= ~(TIM_SMCR_SMS);           // clear-slave mode
  timer->SMCR |= (0x3 << TIM_SMCR_SMS_Pos); // set Encoder mode 3

  // For some reason this bit was sometimes set for no reason, clear it.
  timer->CR1 &= ~(TIM_CR1_CMS);

  timer->CCER |= (TIM_CCER_CC1E); // enable capture-compare
  timer->CCER |= (TIM_CCER_CC2E); // enable capture-compare

  start_timer(timer);
}

void setup_gpio_pull_down(TimerPinSetup &p) {
  GPIO_TypeDef *gpioPort = p.get_gpio_port();
  gpioPort->PUPDR &= ~(0x3 << (p.pin * 2)); // Clear PUPD bits
  gpioPort->PUPDR |= (0x2 << (p.pin * 2));  // Pull-down (10)
}
