#ifndef STEPPER_CONFIG_H_
#define STEPPER_CONFIG_H_

/**
 * Timer clocks are depending on the system and bus clock setup
 * See also:
 *  - Figure 55: Core and bus clock generation
 *  - Table 59: Ratio between clock timer and pclk
 * We do not mess with lowering frequencies.B2)
 */
#define MOT_X_PWM_TIM_FREQ 240000000   // in Mhz
#define MOT_YAW_PWM_TIM_FREQ 240000000 // in Mhz
#define MOT_Z_PWM_TIM_FREQ 240000000   // in Mhz
#define MOT_U_PWM_TIM_FREQ 240000000   // in Mhz

/*
 * PWM counter setup
 * The counter counts to MAX and sets the PWM signal high once duty-cycle is
 * high Prescaler divides the counted value
 *
 * Timers tick with 240 Mhz -> ~4ns per increase.
 * Setting prescaler to 120 -> 2 Mhz between ticks
 * d7/d8 needs at least 250ns signal width
 *
 * Frequency of pulse is between 2 hz and 1 Mhz
 *
 * It is essential to set the prescaler appropriately with respect to the
 * 16 bit uint range for the respective timer counter.
 *
 * float tick_speed = (pwm_freq / pwm_prescaler); // Mhz
 * float tim_max = tick_speed / (steps_per_sec*micro_steps);
 */
#define MOT_X_PWM_PRESCALER 120
#define MOT_YAW_PWM_PRESCALER 120
#define MOT_Z_PWM_PRESCALER 120
#define MOT_U_PWM_PRESCALER 120

/**
 * Microsteps are set via the SW switch on igus D7/D8 controller,
 * the values here need to match the switch config!
 * (rot_per_step / microsteps ) / gear ratio = precision per pulse (mm or deg)
 */
#define MOT_X_MICROSTEPS 8   // 0.0275 mm
#define MOT_YAW_MICROSTEPS 8 // 0.0059 deg
#define MOT_Z_MICROSTEPS 16  // 0.021875 mm
#define MOT_U_MICROSTEPS 4

/**
 * Gear ratios come from the axis datasheets
 */
#define MOT_X_GEAR_RATIO (360 / 44.f) // feed rate (rev/mm)
#define MOT_YAW_GEAR_RATIO 38         // 38:1 ratio axis/motor rev
#define MOT_Z_GEAR_RATIO (360 / 70.f) // feed rate (rev/mm)
#define MOT_U_GEAR_RATIO (360 / 44.f) // feed rate (rev/mm)

/**
 * All Igus encoders have 500 pulses per revolution,
 * quadrature encoders count 4 times per pulse
 * This value also effects the encoder timer setup,
 * as we configure the timer as follows:
 *  - Encoder start/reset value is 4 * rev_count
 *  - max value to 8 * rev_count
 * Hence, a signal of N needs to reset the counter to 4 * rev_count.
 */
#define MOT_X_ENC_REV_COUNT 2000
#define MOT_YAW_ENC_REV_COUNT 2000
#define MOT_Z_ENC_REV_COUNT 2000
#define MOT_U_ENC_REV_COUNT 2000

#define MOT_X_STEPS_PER_REV 200
#define MOT_YAW_STEPS_PER_REV 200
#define MOT_Z_STEPS_PER_REV 200
#define MOT_U_STEPS_PER_REV 200

// in degree
#define MOT_X_ROT_PER_STEP 1.8
#define MOT_YAW_ROT_PER_STEP 1.8
#define MOT_Z_ROT_PER_STEP 1.8
#define MOT_U_ROT_PER_STEP 1.8

/**
 * Do not touch this, this refelects the hardware setup of the d7/d8 controllers
 */
#define DIR_CW false
#define DIR_CCW true

// precision threshold (in steps)
#define MOT_X_PRECISION_THRESHOLD 0.5   // mm
#define MOT_YAW_PRECISION_THRESHOLD 1.0 // 0.02 degree
#define MOT_Z_PRECISION_THRESHOLD 0.5   // mm
#define MOT_U_PRECISION_THRESHOLD 0.5   // mm

/**
 * Direction to move when referencing (calibrating) the system.
 * Flip this if calibrating drives away from the endstop.
 */
#define MOT_X_REFERENCE_DIR DIR_CCW
#define MOT_YAW_REFERENCE_DIR DIR_CW
#define MOT_Z_REFERENCE_DIR DIR_CW
#define MOT_U_REFERENCE_DIR DIR_CCW

/**
 * This determines the direction where we expect positive progress
 * in the absolute position. This also needs to agree with the encoder
 * counting direction (the relative count per revolution) that can be set below.
 */
#define MOT_X_POSITIVE_DIR DIR_CW
#define MOT_YAW_POSITIVE_DIR DIR_CCW
#define MOT_Z_POSITIVE_DIR DIR_CCW
#define MOT_U_POSITIVE_DIR DIR_CW

/**
 * The encoder should count up in forward direction.
 * If this is not the case, this needs to be flipped.
 */
#define MOT_X_INVERT_ENC_COUNT false
#define MOT_YAW_INVERT_ENC_COUNT true
#define MOT_Z_INVERT_ENC_COUNT true
#define MOT_U_INVERT_ENC_COUNT false

/**
 * Invert this if endstop is inverted (currently the case for all NC endstops)
 */
#define MOT_X_INVERT_ENDSTOP true
#define MOT_YAW_INVERT_ENDSTOP false
#define MOT_Z_INVERT_ENDSTOP true
#define MOT_U_INVERT_ENDSTOP true

// in units per second
#define MOT_X_REFERENCE_SPEED 50
#define MOT_YAW_REFERENCE_SPEED 30
#define MOT_Z_REFERENCE_SPEED 50
#define MOT_U_REFERENCE_SPEED 50

#endif // STEPPER_CONFIG_H_
