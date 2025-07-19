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

#include <Arduino.h>
#include <shared_mem_data.h>

// #define DEBUG_VIA_RPC 1

#define SERIAL_OUTPUT 1

#ifdef DEBUG_VIA_RPC
#include <RPC.h>
#endif

#include <servos/servos.h>
#include <stepper_motors/stepper_motors.h>

#include <array>

#define CONTROL_DT 2 // milliseconds

Feedback current_feedback;
int m4_data_read = -1;

Command current_command;
bool new_command_received = false;

unsigned long last_loop = 0;
unsigned long loop_delta = CONTROL_DT;

// Used in the calibrate function
bool leave_endstop_phase = false;
bool second_endstop_hit = false;

// this determines the order for the position commands to be interpreted
// e.g., first motor in this list will be x, second yaw, third z, fourth u

// mot_u is broken as TIM2 is used by us_ticker of mbed-os on M4
const std::array<stepper_motors::StepperMotorSetup *, 3> stepper_setup = {
    &stepper_motors::mot_x, &stepper_motors::mot_yaw, &stepper_motors::mot_z};
// const std::array<StepperMotorSetup*,1> stepper_setup =
// {&stepper_motors::mot_x};

// servo_rotation is broken as TIM5 is used by us_ticker of mbed-os on M7
const std::array<servos::ServoSetup *, 1> servo_setup = {
    &servos::servo_gripper};
// const std::array<ServoSetup*,0> servo_setup = {};

bool stepper_positions_reached = false;
bool servo_positions_reached = false;

void mot_x_enc_isr() { stepper_motors::mot_x.update_abs_position(); }
void mot_yaw_enc_isr() { stepper_motors::mot_yaw.update_abs_position(); }
void mot_z_enc_isr() { stepper_motors::mot_z.update_abs_position(); }

void mot_u_enc_isr() { stepper_motors::mot_u.update_abs_position(); }

void mot_x_endstop_isr() { stepper_motors::mot_x.endstop_hit(); }

void mot_yaw_endstop_isr() { stepper_motors::mot_yaw.endstop_hit(); }

void mot_z_endstop_isr() { stepper_motors::mot_z.endstop_hit(); }

void mot_u_endstop_isr() { stepper_motors::mot_u.endstop_hit(); }

void attach_interrupts() {

  pinMode(stepper_motors::mot_x.encoder_n_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(stepper_motors::mot_x.encoder_n_pin),
                  mot_x_enc_isr, RISING);

  pinMode(stepper_motors::mot_yaw.encoder_n_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(stepper_motors::mot_yaw.encoder_n_pin),
                  mot_yaw_enc_isr, RISING);

  // pinMode(stepper_motors::mot_z.encoder_n_pin, INPUT_PULLUP);
  // ttachInterrupt(digitalPinToInterrupt(stepper_motors::mot_z.encoder_n_pin),
  //                 mot_z_enc_isr, RISING);

  pinMode(stepper_motors::mot_u.encoder_n_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(stepper_motors::mot_u.encoder_n_pin),
                  mot_u_enc_isr, RISING);

  pinMode(stepper_motors::mot_x.endstop_pin, INPUT);
  if (stepper_motors::mot_x.invert_endstop) {
    attachInterrupt(digitalPinToInterrupt(stepper_motors::mot_x.endstop_pin),
                    mot_x_endstop_isr, FALLING);
  } else {
    attachInterrupt(digitalPinToInterrupt(stepper_motors::mot_x.endstop_pin),
                    mot_x_endstop_isr, RISING);
  }

  pinMode(stepper_motors::mot_yaw.endstop_pin, INPUT);
  if (stepper_motors::mot_yaw.invert_endstop) {
    attachInterrupt(digitalPinToInterrupt(stepper_motors::mot_yaw.endstop_pin),
                    mot_yaw_endstop_isr, FALLING);
  } else {
    attachInterrupt(digitalPinToInterrupt(stepper_motors::mot_yaw.endstop_pin),
                    mot_yaw_endstop_isr, RISING);
  }
  pinMode(stepper_motors::mot_z.endstop_pin, INPUT);
  if (stepper_motors::mot_z.invert_endstop) {
    attachInterrupt(digitalPinToInterrupt(stepper_motors::mot_z.endstop_pin),
                    mot_z_endstop_isr, FALLING);
  } else {
    attachInterrupt(digitalPinToInterrupt(stepper_motors::mot_z.endstop_pin),
                    mot_z_endstop_isr, RISING);
  }

  pinMode(stepper_motors::mot_u.endstop_pin, INPUT);
  if (stepper_motors::mot_u.invert_endstop) {
    attachInterrupt(digitalPinToInterrupt(stepper_motors::mot_u.endstop_pin),
                    mot_u_endstop_isr, FALLING);
  } else {
    attachInterrupt(digitalPinToInterrupt(stepper_motors::mot_u.endstop_pin),
                    mot_u_endstop_isr, RISING);
  }
}

void gather_feedback(void) {
  for (size_t i = 0; i < stepper_setup.size(); i++) {
    if (false) { // This can be used to ignore encoders, also requires disabling
                 // of emergency stop
      current_feedback.stepper_positions[i] =
          stepper_setup[i]->steps_to_unit(stepper_setup[i]->curr_steps);
    } else {
      current_feedback.stepper_positions[i] =
          stepper_setup[i]->steps_to_unit(stepper_setup[i]->get_absolute_pos());
    }
    current_feedback.stepper_emergency_stops[i] =
        stepper_setup[i]->emergency_stop;
    current_feedback.stepper_directions[i] = stepper_setup[i]->direction;
    current_feedback.stepper_endstops[i] =
        stepper_setup[i]->invert_endstop
            ? !static_cast<bool>(digitalRead(stepper_setup[i]->endstop_pin))
            : static_cast<bool>(digitalRead(stepper_setup[i]->endstop_pin));
  }

  for (size_t i = 0; i < servo_setup.size(); i++) {
    current_feedback.servo_positions[i] = servo_setup[i]->approx_angle;
  }
}

void send_feedback(void) {
  current_feedback.command_index = current_command.command_index;
  put_to_m4(&current_feedback, sizeof(Feedback));
}

void read_command(void) {
  m4_data_read = get_from_m4(&current_command, sizeof(Command));
  if (m4_data_read > 0) {
    new_command_received = true;
    current_feedback.busy = true;
    leave_endstop_phase = false;
    second_endstop_hit = false;
    current_feedback.current_status = NO_FAILURE;
    for (const auto &mot : stepper_setup) {
      mot->emergency_stop = false; // reset prior detected step loss
      mot->curr_steps =
          mot->get_absolute_pos(); // sync up step count with encoder
    }
  }
}

inline void calibrate(void) {
  bool calibrate_busy = false;
  for (size_t i = 0; i < stepper_setup.size(); i++) {
    if (current_command.stepper_mask & (1 << i)) {
      const bool endstop_active = current_feedback.stepper_endstops[i];
      const bool axis_busy =
          leave_endstop_phase ? endstop_active : !endstop_active;
      calibrate_busy |= axis_busy;
    }
  }
  if (!calibrate_busy && leave_endstop_phase) {
    new_command_received = true;
    leave_endstop_phase = false;
    second_endstop_hit = true;
    Serial.println("Calibrate entersecond_endstop_hit");
    return;
  }
  if (!calibrate_busy && !leave_endstop_phase) {
    if (second_endstop_hit) {
      current_feedback.busy = false;
      current_feedback.referenced = true;
      new_command_received = false;
      second_endstop_hit = false;
#ifdef SERIAL_OUTPUT
      Serial.println("Calibrate done");
#endif
      return;
    }

    Serial.println("Calibrate enter leave_endstop_phase");
    new_command_received = true;
    leave_endstop_phase = true;
    return;
  }
  if (new_command_received) {
    new_command_received = false;

    for (size_t i = 0; i < stepper_setup.size(); i++) {
      if (current_command.stepper_mask & (1 << i)) {
        if (leave_endstop_phase) {
          stepper_setup[i]->out_of_endstop();
        } else {
          if (!current_feedback.stepper_endstops[i]) {
            stepper_setup[i]->reference();
          }
        }
      }
    }
  }
}

inline bool servo_approx_angle(float dt) {
  bool target_reached = true;
  for (size_t i = 0; i < servo_setup.size(); i++) {
    if (current_command.servo_mask & (1 << i)) {
      if (current_command.servo_positions[i] != servo_setup[i]->approx_angle) {
        target_reached = false;
        if (current_command.servo_positions[i] < servo_setup[i]->approx_angle) {
          servo_setup[i]->approx_angle =
              max(current_command.servo_positions[i],
                  servo_setup[i]->approx_angle - servo_setup[i]->speed * dt);
        } else {
          servo_setup[i]->approx_angle =
              min(current_command.servo_positions[i],
                  servo_setup[i]->approx_angle + servo_setup[i]->speed * dt);
        }
      } else {
        continue;
      }
    }
  }
  return target_reached;
}

inline bool servo_move_const(float dt) {
  bool target_reached = true;
  for (size_t i = 0; i < servo_setup.size(); i++) {
    if (current_command.servo_mask & (1 << i)) {
      if (current_command.servo_positions[i] != servo_setup[i]->approx_angle) {
        target_reached = false;
        if (current_command.servo_positions[i] > servo_setup[i]->approx_angle) {
          servo_setup[i]->approx_angle =
              max(current_command.servo_positions[i],
                  servo_setup[i]->approx_angle + servo_setup[i]->speed * dt);
        } else {
          servo_setup[i]->approx_angle =
              min(current_command.servo_positions[i],
                  servo_setup[i]->approx_angle - servo_setup[i]->speed * dt);
        }
        servo_setup[i]->set_position(servo_setup[i]->approx_angle);
      } else {
        continue;
      }
    }
  }
  return target_reached;
}

inline bool stepper_move_const(float dt) {
  bool target_reached = true;
  for (size_t i = 0; i < stepper_setup.size(); i++) {
    if (current_command.stepper_mask & (1 << i)) {
      if (abs(current_command.stepper_positions[i] -
              current_feedback.stepper_positions[i]) >
              stepper_setup[i]->precision_threshold &&
          !stepper_setup[i]->emergency_stop) {
        target_reached = false;
      } else {
        stepper_setup[i]->set_speed(0);
        continue;
      }
      bool positive_dir = current_command.stepper_positions[i] -
                          current_feedback.stepper_positions[i];
      if (positive_dir) {
        stepper_setup[i]->set_dir(stepper_setup[i]->positive_dir);
      } else {
        stepper_setup[i]->set_dir(!stepper_setup[i]->positive_dir);
      }
      stepper_setup[i]->set_speed(
          stepper_setup[i]->unit_to_steps(stepper_setup[i]->reference_speed));
    }
  }
  return target_reached;
}

inline bool stepper_move(float dt) {
  bool target_reached = true;
  for (size_t i = 0; i < stepper_setup.size(); i++) {
    if (current_command.stepper_mask & (1 << i)) {
      if (abs(current_command.stepper_positions[i] -
              current_feedback.stepper_positions[i]) >
              stepper_setup[i]->precision_threshold &&
          !stepper_setup[i]->emergency_stop) {
        target_reached = false;
      } else {
        stepper_setup[i]->set_speed(0);
        continue;
      }
      float speed = stepper_setup[i]->motion_controller.compute(
          current_command.stepper_positions[i],
          current_feedback.stepper_positions[i], dt);
      if (speed > 0.f) {
        stepper_setup[i]->set_dir(stepper_setup[i]->positive_dir);
      } else {
        stepper_setup[i]->set_dir(!stepper_setup[i]->positive_dir);
      }
      speed = abs(speed);
      speed = stepper_setup[i]->unit_to_steps(speed);

      stepper_setup[i]->set_speed(speed);
    }
  }
  return target_reached;
}

void execute_command(void) {
  if (current_feedback.busy) {
    // check if encoder pos and counted steps are matching,
    // if not block motor from moving until next command is issued
    for (const auto &mot : stepper_setup) {
      if (abs(mot->curr_steps - mot->get_absolute_pos()) >
          mot->step_loss_threshold) {
        mot->emergency_stop = true;
        mot->set_speed(0.0);
      }
    }

    switch (current_command.command_id) {
    case CommandID::NO_COMMAND:
      current_feedback.busy = false;
      break;
    case CommandID::CALIBRATE:
      // observe whether all endstops are hit which stops calibration
      calibrate();
      break;

    case CommandID::STOP:
#ifdef SERIAL_OUTPUT
      Serial.println("Stop");
#endif
      for (size_t i = 0; i < stepper_setup.size(); i++) {
        if (current_command.stepper_mask & (1 << i)) {
          stepper_setup[i]->set_speed(0);
        }
      }
      current_feedback.busy = false;
      new_command_received = false;
      break;
    case CommandID::MOVE: {
      if (new_command_received) {
#ifdef SERIAL_OUTPUT
        Serial.println("Move start");
#endif
        for (size_t i = 0; i < servo_setup.size(); i++) {
          if (current_command.servo_mask & (1 << i)) {
            servo_setup[i]->set_position(current_command.servo_positions[i]);
          }
        }
        stepper_positions_reached =
            static_cast<bool>(current_command.stepper_mask);
        servo_positions_reached = static_cast<bool>(current_command.servo_mask);

        for (size_t i = 0; i < stepper_setup.size(); i++) {
          stepper_setup[i]->motion_controller.plan_curve(
              current_feedback.stepper_positions[i],
              current_command.stepper_positions[i]);
        }
        new_command_received = false;
      }

      float dt_sec = loop_delta / 1000.0f;
      stepper_positions_reached = stepper_move(dt_sec);
      servo_positions_reached = servo_approx_angle(dt_sec);
      if (stepper_positions_reached && servo_positions_reached) {
#ifdef SERIAL_OUTPUT
        Serial.println("Move done");
#endif
        current_feedback.busy = false;
        for (const auto &mot : stepper_setup) {
          mot->pid_controller.reset();

          mot->motion_controller.reset();
        }
      }
      if (current_feedback.current_status == NO_FAILURE) {
        for (size_t i = 0; i < stepper_setup.size(); i++) {
          if ((stepper_setup[i]->curr_steps_per_sec == 0) &&
              stepper_positions_reached == false) {
            current_feedback.current_status = GENERIC_FAILURE;
            return;
          }
        }
      }
      break;
    }
    case CommandID::PID_UPDATE:
      new_command_received = false;
      for (size_t i = 0; i < stepper_setup.size(); i++) {
        stepper_setup[i]->set_speed(0);
        if (current_command.pid_motor_id == i) {
#ifdef SERIAL_OUTPUT
          Serial.print("PID Update for motor ");
          Serial.print(i);
#endif
          if (current_command.pid_param_mask & 1) {
#ifdef SERIAL_OUTPUT
            Serial.print(" New P: ");
#endif
            stepper_setup[i]->pid_controller.Kp_ =
                current_command.pid_params[0];
            Serial.print(stepper_setup[i]->pid_controller.Kp_);
          }
          if (current_command.pid_param_mask & (1 << 1)) {
#ifdef SERIAL_OUTPUT
            Serial.print(" New I: ");
#endif
            stepper_setup[i]->pid_controller.Ki_ =
                current_command.pid_params[1];
            Serial.print(stepper_setup[i]->pid_controller.Ki_);
          }
          if (current_command.pid_param_mask & (1 << 2)) {
#ifdef SERIAL_OUTPUT
            Serial.print(" New D: ");
#endif
            stepper_setup[i]->pid_controller.Kd_ =
                current_command.pid_params[2];
#ifdef SERIAL_OUTPUT
            Serial.print(stepper_setup[i]->pid_controller.Kd_);
#endif
          }
          stepper_setup[i]->pid_controller.integral_ = 0;
          stepper_setup[i]->pid_controller.prev_error_ = 0;
          stepper_setup[i]->pid_controller.prev_speed_ = 0;
        }
      }
#ifdef SERIAL_OUTPUT
      Serial.println("PID Update");
#endif
      new_command_received = false;
      current_feedback.busy = false;
      break;

    case CommandID::S_CONTROLLER_UPDATE:
      new_command_received = false;
      for (size_t i = 0; i < stepper_setup.size(); i++) {
        stepper_setup[i]->set_speed(0);
        if (current_command.motion_controller_motor_id == i) {
#ifdef SERIAL_OUTPUT
          Serial.print("S Controller Update for motor ");
          Serial.print(i);
#endif
          if (current_command.pid_param_mask & (1 << 0)) {

            stepper_setup[i]->motion_controller.min_speed_ =
                current_command.motion_controller_params[0];
#ifdef SERIAL_OUTPUT
            Serial.print(" New min speed: ");
            Serial.print(stepper_setup[i]->motion_controller.min_speed_);
#endif
          }
          if (current_command.pid_param_mask & (1 << 1)) {

            stepper_setup[i]->motion_controller.max_speed_ =
                current_command.motion_controller_params[1];
#ifdef SERIAL_OUTPUT
            Serial.print(" New max speed: ");
            Serial.print(stepper_setup[i]->motion_controller.max_speed_);
#endif
          }
          if (current_command.pid_param_mask & (1 << 2)) {
            stepper_setup[i]->motion_controller.max_accel_ =
                current_command.motion_controller_params[2];
#ifdef SERIAL_OUTPUT
            Serial.print(" New accel: ");
            Serial.print(stepper_setup[i]->motion_controller.max_accel_);

#endif
          }
          if (current_command.pid_param_mask & (1 << 3)) {

            stepper_setup[i]->motion_controller.max_jerk_ =
                current_command.motion_controller_params[3];
#ifdef SERIAL_OUTPUT
            Serial.print(" New jerk: ");
            Serial.print(stepper_setup[i]->motion_controller.max_jerk_);
#endif
          }
          if (current_command.pid_param_mask & (1 << 4)) {

            stepper_setup[i]->motion_controller.short_dist_ =
                current_command.motion_controller_params[4];
#ifdef SERIAL_OUTPUT
            Serial.print(" New short dist: ");
            Serial.print(stepper_setup[i]->motion_controller.short_dist_);
#endif
          }
        }
      }
#ifdef SERIAL_OUTPUT
      Serial.println(" Motion Controller Update");
#endif
      new_command_received = false;
      current_feedback.busy = false;
      break;
    case CommandID::CONST_SPEED: {
      if (new_command_received) {
#ifdef SERIAL_OUTPUT
        Serial.println("Const speed start");
#endif
        stepper_positions_reached =
            static_cast<bool>(current_command.stepper_mask);
        servo_positions_reached = static_cast<bool>(current_command.servo_mask);
        new_command_received = false;
      }
      float dt_sec = loop_delta / 1000.0f;
      stepper_positions_reached = stepper_move_const(dt_sec);
      servo_positions_reached = servo_move_const(dt_sec);
      if (stepper_positions_reached && servo_positions_reached) {
#ifdef SERIAL_OUTPUT
        Serial.println("Move done");
#endif
        current_feedback.busy = false;
      }
      break;
    }
    }
  } else {
    for (size_t i = 0; i < stepper_setup.size(); i++) {
      stepper_setup[i]->set_speed(0);
    }
  }
}

void setup() {
#ifdef SERIAL_OUTPUT
  Serial.begin(115200);
#endif
  /**
   * We need to boot the M4 right away, for some reason it resets some registers
   * that it should not be resetting.
   * In Particular, TIM2 (used for mot_u encoder) is affected by this...
   */
  HSEM_Init();
  MPU_Config();
  core_share_init();
  bootM4();

  GPIOA->AFR[1] = 0x0; // was on non-default values for some reason

  delay(4000); // arbirtrary, mainly to give some time before bricking the
               // firmware

  current_command.command_id = CommandID::NO_COMMAND;

  for (const auto &mot : stepper_setup) {
    mot->init();
  }
  for (const auto &servo : servo_setup) {
    servo->init();
  }
#ifdef SERIAL_OUTPUT
  Serial.println("motor setup done");
#endif
#ifdef DEBUG_VIA_RPC
  RPC.begin();
#endif
  // gather feedback once before attaching interrupts to have sensible values
  gather_feedback();

  attach_interrupts();
#ifdef SERIAL_OUTPUT
  Serial.println("interrupts done");
#endif

  last_loop = millis();
}

/**
 * The following steps are executed with a fixed time step of CONTROL_DT:
 * 1. gather feedback
 * 2. send feedback to m4 (ethernet)
 * 1. check for new command from m4 (ethernet)
 * 2. execute current command
 */
void loop() {
  if (millis() < last_loop + CONTROL_DT) {
    return;
  }
  loop_delta = millis() - last_loop;
  last_loop = millis();
  for (const auto &mot : stepper_setup) {
    mot->update_curr_steps(loop_delta / 1000.f);
  }
#ifdef DEBUG_VIA_RPC
  // step 0: write RPC buffer to serial
  String buffer = "";
  while (RPC.available()) {

    buffer += (char)RPC.read(); // Fill the buffer with characters
  }
  if (buffer.length() > 0) {

    Serial.print(buffer);
  }
#endif

  // step 1: check feedback and write it in current_feedback
  gather_feedback();
  // step 2: send feedback:
  send_feedback();
  // step 3: check for new command
  read_command();

#ifdef SERIAL_OUTPUT
  static unsigned long last_called = millis();
  if (last_called + 1000 < millis()) {

    last_called = millis();
    if (new_command_received) {
      Serial.println("new command received");
    }
    for (size_t i = 0; i < 3; i++) {
      Serial.print("mot ");
      Serial.print(i);
      Serial.print(" enc: ");
      Serial.print(stepper_setup[i]->encoder_a.get_timer()->CNT);
      // Serial.print(" psc: ");
      // Serial.print(stepper_setup[i]->encoder_a.get_timer()->PSC);
      // Serial.print(" stop: ");
      // Serial.print(current_feedback.stepper_endstops[i]);
      Serial.print(" offset: ");
      Serial.print(stepper_setup[i]->abs_position_step_offset);
      Serial.print(" pos: ");
      Serial.print(current_feedback.stepper_positions[i]);
      Serial.print(" target: ");
      Serial.print(current_command.stepper_positions[i]);
      Serial.print(" peak: ");
      Serial.print(stepper_setup[i]->motion_controller.peak_speed_);
      Serial.print(" accel: ");
      Serial.print(stepper_setup[i]->motion_controller.accel_);
      Serial.print(" delta: ");
      Serial.print(loop_delta, 8);
      Serial.print(" phase: ");
      Serial.print(stepper_setup[i]->motion_controller.curr_phase_);
      Serial.print(" speed: ");
      Serial.println(stepper_setup[i]->motion_controller.speed_);
      Serial.print(" p1: ");
      Serial.print(stepper_setup[i]->motion_controller.dist_phase_1_);
      Serial.print(" p2: ");
      Serial.print(stepper_setup[i]->motion_controller.dist_phase_2_);
      Serial.print(" p3: ");
      Serial.print(stepper_setup[i]->motion_controller.dist_phase_3_);
      Serial.print(" p4: ");
      Serial.print(stepper_setup[i]->motion_controller.dist_phase_4_);
      Serial.print(" p5: ");
      Serial.print(stepper_setup[i]->motion_controller.dist_phase_5_);
      Serial.print(" p6: ");
      Serial.print(stepper_setup[i]->motion_controller.dist_phase_6_);
      Serial.print(" p7: ");
      Serial.println(stepper_setup[i]->motion_controller.dist_phase_7_);
    }
  }
#endif

  execute_command();
}
