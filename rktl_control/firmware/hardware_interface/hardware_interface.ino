/*********************************************************************
Arduino-Pico sketch used to control a car over a WebSocket.
License:
  BSD 3-Clause License
  Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
*********************************************************************/

#include <ArduinoHttpClient.h>
#include <WiFi.h>
#include <ros.h>

#define PACKET_SIZE 16

char ssid[] = "SSID GOES HERE";
char pass[] = "PASSWORD GOES HERE";

char serverAddress[] = "IP ADDRESS OF WEBSOCKET GOES HERE";  // server address
int port = WEBSOCKET PORT GOES HERE; // this is meant to cause a compilation error if it's not properly set

WiFiClient wifi;
WebSocketClient client = WebSocketClient(wifi, serverAddress, port);
int status = WL_IDLE_STATUS;

typedef struct {
  double throttle;
  double steering;
} packet_t;


typedef union {
  packet_t packet_obj;
  byte array[16];
} packet_union_t;

packet_union_t my_packet = {0.0, 0.0};

void setup() {
  Serial.begin(9600);
  while (status != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
  }

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}

void loop() {
  client.begin();

  while (client.connected()) {
    Serial.println("Connected to socket");

    int messageSize = client.parseMessage();

    if (messageSize >= PACKET_SIZE) {
      Serial.println("Receiving packet");
      client.read(my_packet.array, PACKET_SIZE);
      Serial.print("Throttle: ");
      Serial.println(my_packet.packet_obj.throttle);
      Serial.print("Steering: ");
      Serial.println(my_packet.packet_obj.steering);
    } else if (messageSize != 0) {
      Serial.println("Bad message size"); // don't know if it receives multiple bytes between repitions
    }
  }
}
