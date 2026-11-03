#include <Wire.h>

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {}  // wait for Serial Monitor
  Wire.begin();
  Serial.println("Scanning...");
}

void loop() {
  int found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Device at 0x");
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) Serial.println("No devices found");
  Serial.println("---");
  delay(2000);
}