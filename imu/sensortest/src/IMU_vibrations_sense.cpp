#include <Arduino.h>
#include <Wire.h>

#include "../sensortest/src/mpu6500.h"

// Compile the local driver implementation with this sketch.
#include "../sensortest/src/invensense_imu.cpp"
#include "../sensortest/src/mpu6500.cpp"

namespace {

constexpr uint32_t kSamplePeriodUs = 2000;  // 500 Hz
constexpr uint8_t kSampleRateDivider = 1;   // MPU-6500: 1 kHz / (1 + 1)

bfs::Mpu6500 imu;
uint32_t nextSampleUs = 0;

void stopWithError(const char *message) {
  Serial.println(message);
  while (true) {
    delay(1000);
  }
}

void publishSample() {
  Serial.print(micros());
  Serial.print(',');
  Serial.print(imu.accel_x_mps2(), 4);
  Serial.print(',');
  Serial.print(imu.accel_y_mps2(), 4);
  Serial.print(',');
  Serial.print(imu.accel_z_mps2(), 4);
  Serial.print(',');
  Serial.print(imu.gyro_x_radps(), 4);
  Serial.print(',');
  Serial.print(imu.gyro_y_radps(), 4);
  Serial.print(',');
  Serial.println(imu.gyro_z_radps(), 4);
}

}  // namespace

void setup() {
  Serial.begin(921600);
  const uint32_t serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 3000) {}

  Wire.begin();
  Wire.setClock(400000);

  imu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
  if (!imu.Begin()) {
    stopWithError("MPU-6500 initialization failed; check address and wiring");
  }
  if (!imu.ConfigAccelRange(bfs::Mpu6500::ACCEL_RANGE_8G) ||
      !imu.ConfigGyroRange(bfs::Mpu6500::GYRO_RANGE_2000DPS) ||
      !imu.ConfigDlpfBandwidth(bfs::Mpu6500::DLPF_BANDWIDTH_184HZ) ||
      !imu.ConfigSrd(kSampleRateDivider)) {
    stopWithError("MPU-6500 configuration failed");
  }

  Serial.println("timestamp_us,ax_mps2,ay_mps2,az_mps2,gx_radps,gy_radps,gz_radps");
  nextSampleUs = micros();
}

void loop() {
  const uint32_t now = micros();
  if (static_cast<int32_t>(now - nextSampleUs) < 0) {
    return;
  }

  nextSampleUs += kSamplePeriodUs;
  if (static_cast<int32_t>(now - nextSampleUs) >= 0) {
    nextSampleUs = now + kSamplePeriodUs;
  }

  if (imu.Read()) {
    publishSample();
  }
}
