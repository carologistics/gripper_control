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
#include <ArduinoJson.h>
#include <shared_mem_data.h>

#define DEBUG_VIA_RPC 1

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
#define UDP_TX_PACKET_MAX_SIZE 128

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 1, 100);
IPAddress remote_ip(192, 168, 1, 2); // where to send to
EthernetUDP Udp;
unsigned int localPort = 8888; // local port to listen on

unsigned int remote_port = 8889;
// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; // buffer to hold incoming packet,

char serialized_out[128];

Command current_command = {.command_id = CommandID::CALIBRATE,
                           .stepper_positions = {0.0f, 0.0f, 0.0f},
                           .stepper_mask = 0,
                           .servo_positions = {0.0f, 0.0f},
                           .servo_mask = 0};
bool new_command_received = true;
bool current_command_sent = false;
Feedback current_feedback;

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

  // Store boolean values
  doc["wp_sensor"] = current_feedback.wp_sensor;
  doc["busy"] = current_feedback.busy;
  doc["referenced"] = current_feedback.referenced;

  // Serialize to MessagePack and send to output stream
  return serializeMsgPack(doc, serialized_out, 128);
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
  if (doc["command"] == "MOVE") {
    current_command.command_id = CommandID::MOVE;
    current_command.stepper_mask = 0;
    current_command.servo_mask = 0;
    int counter = 0;
    for (auto &val :
         {"target_mot_x", "target_mot_yaw", "target_mot_z", "target_mot_u"}) {
      if (doc[val].is<float>()) {
        current_command.stepper_mask |= 1 << counter;
        current_command.stepper_positions[counter] = doc[val];
      }
      counter++;
    }
    counter = 0;
    for (auto &val : {"target_servo_gripper", "target_servo_rotation"}) {
      if (doc[val].is<float>()) {
        current_command.servo_mask |= 1 << counter;
        current_command.servo_positions[counter] = doc[val];
      }
      counter++;
    }
  } else if (doc["command"] == "CALIBRATE") {
    current_command.command_id = CommandID::CALIBRATE;
  } else if (doc["command"] == "STOP") {
    current_command.command_id = CommandID::STOP;
  } else {
#ifdef DEBUG_VIA_RPC
    RPC.println("Load json failed, unknown command");
#endif

    return false;
  }
  return true;
}

void setup() {
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

#ifdef DEBUG_VIA_RPC
  RPC.print("Sending first command ");
  RPC.println(current_command.command_id);
#endif
  int res = 0;
  res = put_to_m7(&current_command, sizeof(Command));
  digitalWrite(LEDG, LOW);
  delay(500);
  digitalWrite(LEDG, HIGH);

  delay(500);
  for (int i = 0; i < res; i++) {
    digitalWrite(LEDB, LOW);
    delay(100);
    digitalWrite(LEDB, HIGH);
    delay(100);
  }

#ifdef DEBUG_VIA_RPC

  RPC.println("done sending");
  RPC.print("M4 has data: ");
  RPC.print(res);
#endif
  if (res > 0) {
    new_command_received = false; // command was processed and relayed
  }
}

void loop() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
#ifdef DEBUG_VIA_RPC
    RPC.print("Recieved  ");
    RPC.println(packetSize);
#endif
    Udp.read(packetBuffer, packetSize);

#ifdef DEBUG_VIA_RPC
    RPC.print("Recieved  ");
    RPC.println(packetBuffer);
#endif
  }
}
