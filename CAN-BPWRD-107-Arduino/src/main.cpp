#include <Arduino.h>

#include "107-Arduino-CriticalSection.h"
#include "107-Arduino-Cyphal.h"

#include "SimpleCan.h"

void setup() {
  Serial2.begin(115200);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  uavcan::node::Heartbeat_1_0 msg;
}

