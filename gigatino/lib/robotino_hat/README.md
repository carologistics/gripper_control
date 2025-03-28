# Robotino HAT
Setup helpers to interface against the Robotino HAT for the Arduino Giga.

Reference Docs:
- https://www.st.com/en/microcontrollers-microprocessors/stm32h7-series/documentation.html
- Datasheet: DS12930
- Reference Manual: RM0399
- Arduino Giga pinout: ABX00063

## ERRORS
The socket mot_x actually is wired for the x axis encoder:
 - mot_x pin 1 is 5v
 - mot_x pin 2 is D10 (enc x pin a)
 - mot_x pin 3 is D12 (enc xpin b)
 - mot_x pin 4 is D0 (enc x pin n)
 - mot_x pin 5 is GND

The socket enc_x is actually wired for the x axis d7 controller:
 - enc_x pin 1 is D5 (mot x pwm)
 - enc_x pin 2 is 5v
 - enc_x pin 3 is D7 (mot x dir)
 - enc_x pin 4 is GND
 - enc_x pin 5 is D13 (mot x enable)
 - enc_x_oin 6 is GND



## Configuration
The 'config' directory is the interface if you want to (re)-configure the system.

## Hardware
All sockets on the robotino HAT have a white point indicating first pin.
### Stepper Motors
The motor needs to be setup such that when setting DIR_CW / not connecting DIR pin
results in the motor spinning in the clockwise direction
(when looking at the motor from the front) as described in the d7/d8 data sheet.
You can invert the directions by tweaking the software config as needed (see 'stepper_config.h').

### Endstops and WP Sensor
X, Z, and U motors come with NC endstops, while YAW and the WP sensor are wired NO.
NEVER plug in a NC sensor in NO socket!
The wiring between those is also different:

NC:
 - pin 1: signal (black)
 - pin 2: power (brown)
 - pin 3: ground (blue)

NO:
 - pin 1: power (brown)
 - pin 2: signal (black)
 - pin 3: ground (blue)

### Encoders
M12 8 pin socket is wired like this (numbers according to motor datasheet):
- pin 1: white
- pin 2: brown
- pin 3: green
- pin 4: yellow
- pin 5: gray
- pin 6: pink
- pin 7: blue
- pin 8: red

THe sockets on the hat follow this wiring (we do not use complementary signals A/ B/ and N/):
 - pin 1: A (white)
 - pin 2: 5v (red)
 - pin 3: B (green)
 - pin 4: 0v (grey)
 - pin 5: N (blue)
 - pin 6: unused

### Break
M8 Socket wiring:
- pin 1: 24v (brown)
- pin 3: 0v (blue)
- pin 4: unused (black)

Robotino HAT socket wiring:
- pin 1: unused
- pin 2: brown
- pin 3: blue

### Servos
Servo Cable output:
 - red: 5v
 - yellow: signal
 - brown: ground

Robotino HAT socket wiring:
 - pin 1: 5v (brown)
 - pin 2: signal (black)
 - pin 3: ground (blue)

## D7/D8
Robotino HAT socket wiring:
- pin 1: unused (is connected to 5v)
- pin 2: step+ (white)
- pin 3: dir+ (old grey -> new brown)
- pin 4: en+ (old yellow -> new blue)
- pin 5: en-, dir- and step- (green, brown and blue -> black)

The D8 has additional out- and out+ connectors that we do not use.

The controllers also need to be configured using SW switches.

  - D7: SW1 OFF, SW2 OFF, SW3 ON (1.6A current, should be appropriate for NEMA 17)
  - D8: SW1 OFF, SW2 ON, SW3 OF (4A current, should be appropriate for NEMA 24)
  - SW7: ON (standstill current 50% to reduce heat while idling)
  - SW4, SW5, SW6 and SW8 determine micosteps (between 1 and 128). Make sure to also set the correct microsteps in 'stepper_config.h'.
