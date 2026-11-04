#include <Arduino.h>
#include <Wire.h>

#include "../sensortest/src/mpu6500.h"

// Compile the local driver implementation with this sketch.
#include "../sensortest/src/invensense_imu.cpp"
#include "../sensortest/src/mpu6500.cpp"

namespace {

constexpr uint32_t kSampleRateHz = 500;
constexpr uint32_t kSamplePeriodUs = 1000000UL / kSampleRateHz;
constexpr uint8_t kSampleRateDivider = 1;  // 1 kHz / (1 + 1)
constexpr size_t kWindowSize = 256;        // 1.953 Hz bin spacing
constexpr size_t kSpectrumBins = kWindowSize / 2;
constexpr uint8_t kBaselineWindows = 8;
constexpr float kMinAxisRmsMps2 = 0.05f;
constexpr float kFrequencyShiftWarningHz = 3.0f;
constexpr float kPi = 3.14159265358979323846f;

struct AccelSample {
  float axis[3];
  uint32_t timestampUs;
};

bfs::Mpu6500 imu;
AccelSample samples[kWindowSize];
float fftReal[kWindowSize];
float fftImag[kWindowSize];
float magnitudes[kSpectrumBins + 1];
float baselineMean[3] = {0.0f, 0.0f, 0.0f};
float baselineFrequency[3] = {0.0f, 0.0f, 0.0f};
uint8_t baselineMeanCount = 0;
uint8_t baselineFrequencyCount[3] = {0, 0, 0};
uint32_t nextSampleUs = 0;
size_t sampleCount = 0;

void stopWithError(const char *message) {
  Serial.println(message);
  while (true) {
    delay(1000);
  }
}

void calculateFft() {
  size_t reversed = 0;
  for (size_t i = 1; i < kWindowSize; ++i) {
    size_t bit = kWindowSize >> 1;
    while ((reversed & bit) != 0) {
      reversed ^= bit;
      bit >>= 1;
    }
    reversed ^= bit;
    if (i < reversed) {
      const float real = fftReal[i];
      fftReal[i] = fftReal[reversed];
      fftReal[reversed] = real;
      const float imag = fftImag[i];
      fftImag[i] = fftImag[reversed];
      fftImag[reversed] = imag;
    }
  }

  for (size_t length = 2; length <= kWindowSize; length <<= 1) {
    const size_t halfLength = length / 2;
    const float angle = -2.0f * kPi / length;
    const float stepReal = cosf(angle);
    const float stepImag = sinf(angle);
    for (size_t start = 0; start < kWindowSize; start += length) {
      float twiddleReal = 1.0f;
      float twiddleImag = 0.0f;
      for (size_t offset = 0; offset < halfLength; ++offset) {
        const size_t even = start + offset;
        const size_t odd = even + halfLength;
        const float oddReal =
            fftReal[odd] * twiddleReal - fftImag[odd] * twiddleImag;
        const float oddImag =
            fftReal[odd] * twiddleImag + fftImag[odd] * twiddleReal;
        fftReal[odd] = fftReal[even] - oddReal;
        fftImag[odd] = fftImag[even] - oddImag;
        fftReal[even] += oddReal;
        fftImag[even] += oddImag;

        const float nextTwiddleReal =
            twiddleReal * stepReal - twiddleImag * stepImag;
        twiddleImag =
            twiddleReal * stepImag + twiddleImag * stepReal;
        twiddleReal = nextTwiddleReal;
      }
    }
  }
}

float dominantFrequency(size_t axis, float &axisRms, float &axisMean) {
  axisMean = 0.0f;
  for (size_t i = 0; i < kWindowSize; ++i) {
    axisMean += samples[i].axis[axis];
  }
  axisMean /= kWindowSize;

  float sumSquares = 0.0f;
  for (size_t i = 0; i < kWindowSize; ++i) {
    const float centered = samples[i].axis[axis] - axisMean;
    sumSquares += centered * centered;
    const float hann =
        0.5f - 0.5f * cosf(2.0f * kPi * i / (kWindowSize - 1));
    fftReal[i] = centered * hann;
    fftImag[i] = 0.0f;
  }
  axisRms = sqrtf(sumSquares / kWindowSize);

  calculateFft();
  for (size_t bin = 0; bin <= kSpectrumBins; ++bin) {
    magnitudes[bin] =
        sqrtf(fftReal[bin] * fftReal[bin] + fftImag[bin] * fftImag[bin]);
  }

  size_t peakBin = 1;
  for (size_t bin = 2; bin < kSpectrumBins; ++bin) {
    if (magnitudes[bin] > magnitudes[peakBin]) {
      peakBin = bin;
    }
  }

  float binOffset = 0.0f;
  const float left = magnitudes[peakBin - 1];
  const float center = magnitudes[peakBin];
  const float right = magnitudes[peakBin + 1];
  const float denominator = left - 2.0f * center + right;
  if (fabsf(denominator) > 1e-12f) {
    binOffset = 0.5f * (left - right) / denominator;
    if (binOffset < -0.5f) binOffset = -0.5f;
    if (binOffset > 0.5f) binOffset = 0.5f;
  }

  return (peakBin + binOffset) * kSampleRateHz / kWindowSize;
}

void publishWindow() {
  float rms[3];
  float mean[3];
  float frequency[3];
  for (size_t axis = 0; axis < 3; ++axis) {
    frequency[axis] = dominantFrequency(axis, rms[axis], mean[axis]);
  }

  if (baselineMeanCount < kBaselineWindows) {
    for (size_t axis = 0; axis < 3; ++axis) {
      baselineMean[axis] += mean[axis];
    }
    ++baselineMeanCount;
    if (baselineMeanCount == kBaselineWindows) {
      for (size_t axis = 0; axis < 3; ++axis) {
        baselineMean[axis] /= kBaselineWindows;
      }
    }
  }

  for (size_t axis = 0; axis < 3; ++axis) {
    if (rms[axis] >= kMinAxisRmsMps2 &&
        baselineFrequencyCount[axis] < kBaselineWindows) {
      baselineFrequency[axis] += frequency[axis];
      ++baselineFrequencyCount[axis];
      if (baselineFrequencyCount[axis] == kBaselineWindows) {
        baselineFrequency[axis] /= kBaselineWindows;
      }
    }
  }

  const bool baselining = baselineMeanCount < kBaselineWindows ||
                          baselineFrequencyCount[0] < kBaselineWindows ||
                          baselineFrequencyCount[1] < kBaselineWindows ||
                          baselineFrequencyCount[2] < kBaselineWindows;
  Serial.print(samples[kWindowSize - 1].timestampUs);
  Serial.print(',');
  Serial.print(baselining ? "CAL" : "MONITOR");

  for (size_t axis = 0; axis < 3; ++axis) {
    Serial.print(',');
    Serial.print(rms[axis], 4);
    Serial.print(',');
    Serial.print(frequency[axis], 2);
    Serial.print(',');
    if (baselineFrequencyCount[axis] == kBaselineWindows) {
      const float offset = frequency[axis] - baselineFrequency[axis];
      Serial.print(baselineFrequency[axis], 2);
      Serial.print(',');
      Serial.print(offset, 2);
      Serial.print(',');
      Serial.print(fabsf(offset) >= kFrequencyShiftWarningHz ? 1 : 0);
    } else {
      Serial.print("nan,nan,0");
    }

    Serial.print(',');
    if (baselineMeanCount == kBaselineWindows) {
      Serial.print(mean[axis] - baselineMean[axis], 4);
    } else {
      Serial.print("nan");
    }
  }
  Serial.println();
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

  Serial.println(
      "timestamp_us,state,"
      "rms_x_mps2,freq_x_hz,baseline_x_hz,freq_offset_x_hz,shift_x,mean_offset_x_mps2,"
      "rms_y_mps2,freq_y_hz,baseline_y_hz,freq_offset_y_hz,shift_y,mean_offset_y_mps2,"
      "rms_z_mps2,freq_z_hz,baseline_z_hz,freq_offset_z_hz,shift_z,mean_offset_z_mps2");
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

  if (!imu.Read()) {
    return;
  }

  AccelSample &sample = samples[sampleCount++];
  sample.axis[0] = imu.accel_x_mps2();
  sample.axis[1] = imu.accel_y_mps2();
  sample.axis[2] = imu.accel_z_mps2();
  sample.timestampUs = micros();

  if (sampleCount == kWindowSize) {
    publishWindow();
    sampleCount = 0;
  }
}
