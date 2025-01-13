#include "commands.h"
#include "pinout.h"
#include <Ethernet.h>
#include <SPI.h>

// Network settings
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 1, 100);
EthernetServer server(12345);

void setup() {
  // Set the CS pin to high
  pinMode(PIN_SPI_SS, OUTPUT);
  digitalWrite(PIN_SPI_SS, HIGH);
  // Ethernet.setSS(PIN_SPI_SS);
  // Ethernet.initSS();
  // Start the Ethernet connection and the server
  Ethernet.begin(mac, ip);
  server.begin();
  Serial.begin(9600);
  delay(1000);
  Serial.println("Arduino Giga Ethernet server started");

  // Print the IP address
  Serial.print("IP Address: ");
  Serial.println(Ethernet.localIP());
}

void loop() {
  // Listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
    Serial.println("Client connected");
    String command = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (c == TERMINATOR) {
          handleCommand(command);
          command = "";
        } else {
          command += c;
        }
      }
    }
    client.stop();
    Serial.println("Client disconnected");
  }
}

void handleCommand(const String &command) {
  Serial.print("Received command: ");
  Serial.println(command);

  if (command.startsWith(String(AT) + CMD_STATUS_REQ)) {
    sendStatus();
  } else if (command.startsWith(String(AT) + CMD_STOP)) {
    Serial.println("Stopping");
  } else if (command.startsWith(String(AT) + CMD_X_NEW_POS)) {
    Serial.println("Moving X to new position");
  } else if (command.startsWith(String(AT) + CMD_Y_NEW_POS)) {
    Serial.println("Moving Y to new position");
  } else if (command.startsWith(String(AT) + CMD_Z_NEW_POS)) {
    Serial.println("Moving Z to new position");
  } else if (command.startsWith(String(AT) + CMD_CLOSE)) {
    Serial.println("Closing gripper");
  } else if (command.startsWith(String(AT) + CMD_OPEN)) {
    Serial.println("Opening gripper");
  } else if (command.startsWith(String(AT) + CMD_CALIBRATE)) {
    Serial.println("Calibrating");
  } else if (command.startsWith(String(AT) + CMD_DOUBLE_CALIBRATE)) {
    Serial.println("Double calibrating");
  }
}

void sendStatus() {
  Serial.println("Sending status");
  // Dummy status message
  String status = "I 0.0 0.0 0.0 OPEN";
  EthernetClient client = server.available();
  if (client) {
    client.println(status);
  }
}
