#include <Arduino.h>
#include <Wire.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "../sensortest/src/mpu6500.h"

// Compile the local driver implementation with this sketch.
#include "../sensortest/src/invensense_imu.cpp"
#include "../sensortest/src/mpu6500.cpp"

namespace {

constexpr uint32_t kSamplePeriodMs = 2;  // 500 Hz; requires a 1 kHz RTOS tick.
constexpr size_t kWindowSize = 256;      // 512 ms of samples per decision.
constexpr UBaseType_t kQueueLength = 2 * kWindowSize;
constexpr float kVibrationOnRmsMps2 = 1.0f;
constexpr float kVibrationOffRmsMps2 = 0.7f;

struct ImuSample {
  double ax;
  double ay;
  double az;
  double gx;
  double gy;
  double gz;
  uint32_t timestampUs;
};

static_assert(pdMS_TO_TICKS(kSamplePeriodMs) > 0,
              "FreeRTOS tick rate is too low for the requested sample period");

QueueHandle_t sampleQueue = nullptr;
bfs::Mpu6500 imu;
uint32_t droppedSamples = 0;

void stopWithError(const char *message) {
  Serial.println(message);
  while (true) {
    delay(1000);
  }
}

void imuSamplerTask(void *) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  constexpr TickType_t periodTicks = pdMS_TO_TICKS(kSamplePeriodMs);

  for (;;) {
    if (imu.Read()) {
      const ImuSample sample{
          imu.accel_x_mps2(),
          imu.accel_y_mps2(),
          imu.accel_z_mps2(),
          imu.gyro_x_radps(), // or 0.0 if not reading gyro
          imu.gyro_y_radps(), 
          imu.gyro_z_radps(), 
          micros(),
      };

      if (xQueueSend(sampleQueue, &sample, 0) != pdPASS) {
        taskENTER_CRITICAL();
        ++droppedSamples;
        taskEXIT_CRITICAL();
      }
    }

    vTaskDelayUntil(&lastWakeTime, periodTicks);
  }
}

float calculateAccelerationAcRms(const ImuSample *samples) {
  float meanX = 0.0f;
  float meanY = 0.0f;
  float meanZ = 0.0f;

  for (size_t i = 0; i < kWindowSize; ++i) {
    meanX += samples[i].ax;
    meanY += samples[i].ay;
    meanZ += samples[i].az;
  }
  meanX /= kWindowSize;
  meanY /= kWindowSize;
  meanZ /= kWindowSize;

  float sumSquaredDeviation = 0.0f;
  for (size_t i = 0; i < kWindowSize; ++i) {
    const float dx = samples[i].ax - meanX;
    const float dy = samples[i].ay - meanY;
    const float dz = samples[i].az - meanZ;
    sumSquaredDeviation += dx * dx + dy * dy + dz * dz;
  }

  return sqrtf(sumSquaredDeviation / kWindowSize);
}

void vibrationAnalysisTask(void *) {
  ImuSample samples[kWindowSize];
  bool vibrationActive = false;
  uint32_t reportedDropCount = 0;

  for (;;) {
    for (size_t i = 0; i < kWindowSize; ++i) {
      if (xQueueReceive(sampleQueue, &samples[i], portMAX_DELAY) != pdPASS) {
        stopWithError("IMU sample queue receive failed");
      }
    }

    const float accelerationAcRms = calculateAccelerationAcRms(samples);
    const bool nextVibrationState =
        vibrationActive
            ? accelerationAcRms >= kVibrationOffRmsMps2
            : accelerationAcRms >= kVibrationOnRmsMps2;

    if (nextVibrationState != vibrationActive) {
      vibrationActive = nextVibrationState;
      Serial.printf("Vibration %s; acceleration AC RMS = %.3f m/s^2\n",
                    vibrationActive ? "detected" : "cleared",
                    accelerationAcRms);
    }

    taskENTER_CRITICAL();
    const uint32_t currentDropCount = droppedSamples;
    taskEXIT_CRITICAL();
    if (currentDropCount != reportedDropCount) {
      Serial.printf("Warning: %lu IMU sample(s) dropped\n",
                    static_cast<unsigned long>(currentDropCount -
                                               reportedDropCount));
      reportedDropCount = currentDropCount;
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
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
      !imu.ConfigSrd(1)) {
    stopWithError("MPU-6500 configuration failed");
  }

  sampleQueue = xQueueCreate(kQueueLength, sizeof(ImuSample));
  if (sampleQueue == nullptr) {
    stopWithError("Could not create IMU sample queue");
  }

  if (xTaskCreate(imuSamplerTask, "imuSampler", 2048, nullptr,
                  tskIDLE_PRIORITY + 2, nullptr) != pdPASS) {
    stopWithError("Could not create IMU sampler task");
  }
  if (xTaskCreate(vibrationAnalysisTask, "vibration", 2048, nullptr,
                  tskIDLE_PRIORITY + 1, nullptr) != pdPASS) {
    stopWithError("Could not create vibration analysis task");
  }

  Serial.println("MPU-6500 vibration sampling started");
  vTaskStartScheduler();
  stopWithError("FreeRTOS scheduler failed to start");
}

void loop() {}
