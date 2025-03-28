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

#ifdef DEBUG_VIA_RPC
#include <RPC.h>
#endif

// TODO: undef stuff probably not needed
#undef PIN_SPI_SS
#define PIN_SPI_SS 18u
#include <SPI.h>

// TODO: undef stuff probably not needed
#undef PIN_SPI_SS
#define PIN_SPI_SS 18u
#include <ArduinoJson.h>
#include <Ethernet3.h>
#include <EthernetUdp3.h>

#undef UDP_TX_PACKET_MAX_SIZE
#define UDP_TX_PACKET_MAX_SIZE 512

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 3, 199);
IPAddress remote_ip(192, 168, 3, 230); // where to send to
EthernetUDP Udp;
unsigned int localPort = 8888; // local port to listen on

unsigned int remote_port = 8889;
// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; // buffer to hold incoming packet,

char serialized_out[512];

Command current_command;
bool new_command_received = false;
bool current_command_sent = false;
Feedback current_feedback;

#define CONTROL_DT 10 // milliseconds
unsigned long last_loop = 0;

// Function to serialize Feedback struct into MessagePack
size_t serialize_feedback() {
  JsonDocument doc; // Adjust size based on your data

  // Convert arrays to JSON arrays
  JsonArray stepper_positions = doc["stepper_positions"].to<JsonArray>();
  for (int i = 0; i < 4; i++)
    stepper_positions.add(current_feedback.stepper_positions[i]);

  JsonArray servo_positions = doc["servo_positions"].to<JsonArray>();
  for (int i = 0; i < 2; i++)
    servo_positions.add(current_feedback.servo_positions[i]);

  JsonArray stepper_directions = doc["stepper_directions"].to<JsonArray>();
  for (int i = 0; i < 4; i++)
    stepper_directions.add(current_feedback.stepper_directions[i]);

  JsonArray stepper_endstops = doc["stepper_endstops"].to<JsonArray>();
  for (int i = 0; i < 4; i++)
    stepper_endstops.add(current_feedback.stepper_endstops[i]);

  JsonArray stepper_emergency_stops =
      doc["stepper_emergency_stops"].to<JsonArray>();
  for (int i = 0; i < 4; i++)
    stepper_endstops.add(current_feedback.stepper_emergency_stops[i]);

  // Store native values
  doc["wp_sensor"] = current_feedback.wp_sensor;
  doc["busy"] = current_feedback.busy;
  doc["referenced"] = current_feedback.referenced;
  doc["command_index"] = current_feedback.command_index;

  // Serialize to MessagePack and send to output stream
  return serializeMsgPack(doc, serialized_out, 512);
}

inline void fill_stepper_mask(JsonDocument &doc) {
  current_command.stepper_mask = 0;
  int counter = 0;
  for (auto &val :
       {"target_mot_x", "target_mot_yaw", "target_mot_z", "target_mot_u"}) {
    if (doc[val].is<float>()) {
      current_command.stepper_mask |= 1 << counter;
      current_command.stepper_positions[counter] = doc[val];
    }
    counter++;
  }
}

inline void fill_servo_mask(JsonDocument &doc) {
  current_command.servo_mask = 0;
  int counter = 0;
  for (auto &val : {"target_servo_gripper", "target_servo_rotation"}) {
    if (doc[val].is<float>()) {
      current_command.servo_mask |= 1 << counter;
      current_command.servo_positions[counter] = doc[val];
    }
    counter++;
  }
}

bool load_command_from_json(JsonDocument &doc) {
#ifdef DEBUG_VIA_RPC
  RPC.println("Load json");
#endif

  if (!doc["command"].is<const char *>()) {
#ifdef DEBUG_VIA_RPC
    RPC.println("Load json failed, no command");
#endif
    return false;
  }
  if (doc["command_index"].is<int>()) {
    current_command.command_index = doc["command_index"];
  }
  if (doc["command"] == "MOVE") {
    current_command.command_id = CommandID::MOVE;
    fill_stepper_mask(doc);
    fill_servo_mask(doc);
  } else if (doc["command"] == "CALIBRATE") {
    current_command.command_id = CommandID::CALIBRATE;
    fill_stepper_mask(doc);
    // default to calibrating everything
    if (current_command.stepper_mask == 0) {
      current_command.stepper_mask = 0b1111;
    }
  } else if (doc["command"] == "STOP") {
    current_command.command_id = CommandID::STOP;
    fill_stepper_mask(doc);
    // default to calibrating everything
    if (current_command.stepper_mask == 0) {
      current_command.stepper_mask = 0b1111;
    }
  } else if (doc["command"] == "PID_UPDATE") {
    current_command.command_id = CommandID::PID_UPDATE;
    if (!doc["motor"].is<const char *>()) {
      return false;
    }
    uint8_t mot_index = 0;
    const char *motor = doc["motor"];
    if (strcmp(motor, "mot_x") == 0) {
      mot_index = 0;
    } else if (strcmp(motor, "mot_yaw") == 0) {
      mot_index = 1;
    } else if (strcmp(motor, "mot_z") == 0) {
      mot_index = 2;
    } else if (strcmp(motor, "mot_u") == 0) {
      mot_index = 3;
    } else {
      return false;
    }
    current_command.pid_motor_id = mot_index;
    current_command.pid_param_mask = 0;
    int counter = 0;
    for (auto &val : {"P", "I", "D"}) {
      if (doc[val].is<float>()) {
        current_command.pid_param_mask |= 1 << counter;
        current_command.pid_params[counter] = doc[val];
      }
      counter++;
    }
  } else if (doc["command"] == "S_CONTROLLER_UPDATE") {
    current_command.command_id = CommandID::S_CONTROLLER_UPDATE;
    if (!doc["motor"].is<const char *>()) {
      return false;
    }
    uint8_t mot_index = 0;
    const char *motor = doc["motor"];
    if (strcmp(motor, "mot_x") == 0) {
      mot_index = 0;
    } else if (strcmp(motor, "mot_yaw") == 0) {
      mot_index = 1;
    } else if (strcmp(motor, "mot_z") == 0) {
      mot_index = 2;
    } else if (strcmp(motor, "mot_u") == 0) {
      mot_index = 3;
    } else {
      return false;
    }
    current_command.motion_controller_motor_id = mot_index;
    current_command.motion_controller_param_mask = 0;
    int counter = 0;
    for (auto &val :
         {"min_speed", "max_speed", "max_accel", "max_jerk", "short_dist"}) {
      if (doc[val].is<float>()) {
        current_command.motion_controller_param_mask |= 1 << counter;
        current_command.motion_controller_params[counter] = doc[val];
      }
      counter++;
    }
  } else if (doc["command"] == "CONST_SPEED") {
    current_command.command_id = CommandID::CONST_SPEED;
    fill_stepper_mask(doc);
    fill_servo_mask(doc);
  } else {
#ifdef DEBUG_VIA_RPC
    RPC.println("Load json failed, unknown command");
#endif

    return false;
  }
  return true;
}

void setup() {
  memset(&current_command, 0, sizeof(Command)); // init command
#ifdef DEBUG_VIA_RPC
  RPC.begin();
#endif
  HSEM_Init();
  MPU_Config();

#ifdef DEBUG_VIA_RPC
  RPC.print("M4: mem_m4_data: ");
  RPC.println(mem_m4_data(), HEX);
#endif

  // Set the CS pin to high
  pinMode(PIN_SPI_SS, OUTPUT);
  digitalWrite(PIN_SPI_SS, HIGH);
  // Ethernet.setSS(PIN_SPI_SS);
  // Ethernet.initSS();
  //  Start the Ethernet connection and the server
  Ethernet.setCsPin(PIN_SPI_SS);
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);
  delay(1000);

#ifdef DEBUG_VIA_RPC
  RPC.print("IP Address: ");
  RPC.println(Ethernet.localIP());
#endif
  /**
   *
  #ifdef DEBUG_VIA_RPC
    RPC.print("Sending first command ");
    RPC.println(current_command.command_id);
  #endif
         int res = 0;
            res = put_to_m7(&current_command, sizeof(Command));
  #ifdef DEBUG_VIA_RPC
            RPC.println("done sending");
            RPC.print("M4 has data: ");
            RPC.println(res);
  #endif
          if(res > 0) {
            new_command_received = false; // command was processed and relayed
            }

   */
}

void read_incoming() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Udp.read(packetBuffer, packetSize);
#ifdef DEBUG_VIA_RPC
    RPC.print("Recieved  ");
    RPC.println(packetSize);
    RPC.print("Received ");
    RPC.println(packetBuffer);
#endif
    Udp.flush();
    JsonDocument doc; // Adjust size as needed
    deserializeMsgPack(doc, packetBuffer);
    new_command_received = load_command_from_json(doc);

  } else {
  }
}

void send_command_to_m7() {
#ifdef DEBUG_VIA_RPC
  RPC.print("Sending command ");
  RPC.println(current_command.command_id);
#endif
  if (put_to_m7(&current_command, sizeof(Command)) > 0) {

#ifdef DEBUG_VIA_RPC
    RPC.println("Sending done");
#endif
    new_command_received = false; // command was processed and relayed
  } else {
#ifdef DEBUG_VIA_RPC
    RPC.println("Sending problem");
#endif
  }
}

void send_feedback() {
#ifdef DEBUG_VIA_RPC
  RPC.print("Sending feedback via ethernet");
#endif
  if (get_from_m7(&current_feedback, sizeof(Feedback)) > 0) {
#ifdef DEBUG_VIA_RPC
    RPC.println("Feedback received");
#endif
    size_t data_length = serialize_feedback();
    Udp.beginPacket(remote_ip, remote_port);
    Udp.write(serialized_out, data_length);
    Udp.endPacket();
  }
}

void loop() {
  if (millis() < last_loop + CONTROL_DT) {
    return;
  }

  last_loop = millis();
  /**
      if(get_from_m7(&current_feedback, sizeof(Feedback)) > 0) {
        size_t bytes_to_send = serialize_feedback();
        Udp.beginPacket(remote_ip, remote_port);
        Udp.write(serialized_out, bytes_to_send);
        Udp.endPacket();
    }
     */
  static unsigned long last_called = millis();

  if (last_called + 2000 < millis()) {
    if (last_called + 2100 < millis()) {
      digitalWrite(LEDR, HIGH);
    } else {
      digitalWrite(LEDR, LOW);
    }
    if (last_called + 6000 < millis()) {
      last_called = millis();
    }
  }
  read_incoming();
  if (new_command_received) {
    send_command_to_m7();
  }
  send_feedback();
}
