 /***  Includes  ***/
#include <Wire.h>


/***  Defines   ***/
#define SDA   A4
#define SDL   A5

#define ADDRESS   0xAA


uint8_t data = 0x00;


/***  Functions ***/
void receiveEvent(int num) {
  Serial.println("Received");
  while (Wire.available() >= 1) {
    data = Wire.read();
    Serial.println(data);
  }
}

void requestEvent() {
  Serial.print("Requested: ");
  Serial.println(data);
  Wire.write(data);
}

void setup() {
  Wire.begin(ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Serial.begin(115200);
}

void loop() {
  delay(10);
}
