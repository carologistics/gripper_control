# Timer Setup
We heavily rely on timers to create PWM signals and to read encoders.

To ease the setup, a `TimerPinSetup` struct is used, it is documented in `timer_config.h`.

## PWM Generation
Use the `setup_pwm` function to setup a timer as generator for PWM signals.

## Encoder Setup
Use the `setup_encoder` function to configure timer as counter for quadrature encoders.
