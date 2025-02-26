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

// we ended up not using them
enum EEVChannel {
  CHANNEL_1,
  CHANNEL_2,
  CHANNEL_3,
  CHANNEL_4,
  CHANNEL_5,
  CHANNEL_6,
  CHANNEL_7,
  CHANNEL_8,
  CHANNEL_9,
  CHANNEL_COUNT
};
