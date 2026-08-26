/**
 * @file    max30102_module.cpp
 * @brief   Triển khai module MAX30102 — FreeRTOS task đo HR & SpO2.
 *
 * Luồng xử lý:
 *  1. Chờ ngón tay đặt lên (IR > FINGER_THRESHOLD)
 *  2. Thu đủ BUFFER_LENGTH (100) mẫu IR/Red
 *  3. Tính SpO2 ban đầu bằng maxim_heart_rate_and_oxygen_saturation()
 *  4. Liên tục thu thêm NEW_SAMPLES_PER_CYCLE (25) mẫu, dịch buffer, tính lại
 *  5. Song song: phát hiện nhịp tim bằng RR-interval trên kênh IR
 */

#include "max30102_module.h"
#include "shared_state.h"
#include "config.h"

#include <Wire.h>
#include <string.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

// ─────────────────────────────────────────────
// Đối tượng phần cứng (chỉ dùng trong file này)
// ─────────────────────────────────────────────
static MAX30105 particleSensor;

// ─────────────────────────────────────────────
// Hàm nội bộ
// ─────────────────────────────────────────────

/** Kiểm tra ngón tay có đặt lên cảm biến không. */
static bool isFingerPresent(uint32_t ir) {
  return ir > FINGER_THRESHOLD;
}

/**
 * @brief Đọc một mẫu IR và Red từ FIFO của MAX30102 (thread-safe qua i2cMutex).
 * @param[out] ir  Giá trị kênh IR
 * @param[out] red Giá trị kênh Red
 * @return true nếu đọc được mẫu mới.
 */
static bool readOneSample(uint32_t &ir, uint32_t &red) {
  bool got = false;
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    particleSensor.check();
    if (particleSensor.available()) {
      ir  = particleSensor.getIR();
      red = particleSensor.getRed();
      particleSensor.nextSample();
      got = true;
    }
    xSemaphoreGive(i2cMutex);
  }
  return got;
}

/** Reset dữ liệu HR/SpO2 về trạng thái chưa đo (khi mất ngón tay). */
static void resetHealthData() {
  if (xSemaphoreTake(healthMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
    healthData.heartRate = 0;
    healthData.spo2      = 0;
    healthData.hrValid   = false;
    healthData.spo2Valid = false;
    xSemaphoreGive(healthMutex);
  }
}

/**
 * @brief Lọc median 5 phần tử cho nhịp tim — giảm nhiễu giá trị đột biến.
 * @param newHr Giá trị HR mới (BPM)
 * @return Median của 5 giá trị gần nhất.
 */
static int medianHR(int newHr) {
  static int hist[5] = {0, 0, 0, 0, 0};
  static int cnt = 0;

  // Shift lịch sử và thêm giá trị mới
  for (int i = 4; i > 0; i--) hist[i] = hist[i - 1];
  hist[0] = newHr;
  if (cnt < 5) cnt++;

  // Sắp xếp bản sao để lấy median
  int tmp[5], n = cnt;
  for (int i = 0; i < n; i++) tmp[i] = hist[i];
  for (int i = 0; i < n - 1; i++)
    for (int j = 0; j < n - 1 - i; j++)
      if (tmp[j] > tmp[j + 1]) { int t = tmp[j]; tmp[j] = tmp[j + 1]; tmp[j + 1] = t; }
  return tmp[n / 2];
}

static const int RR_BEAT_SIZE = 5;

/**
 * @brief Phát hiện nhịp đập và tính HR từ RR-interval.
 *
 * Dùng thư viện heartRate của SparkFun (checkForBeat).
 * HR hợp lệ (40–180 BPM) được ghi vào healthData qua healthMutex.
 */
static void processBeat(uint32_t       ir,
                         unsigned long  &lastBeatMs,
                         long           (&rrBuf)[RR_BEAT_SIZE],
                         int            &rrHead,
                         int            &rrN) {
  if (!checkForBeat(ir)) return;

  unsigned long now = millis();
  if (lastBeatMs > 0) {
    long rr = (long)(now - lastBeatMs);
    // RR hợp lệ: 300–1500 ms (tương đương 40–200 BPM)
    if (rr >= 300 && rr <= 1500) {
      rrBuf[rrHead] = rr;
      rrHead = (rrHead + 1) % RR_BEAT_SIZE;
      if (rrN < RR_BEAT_SIZE) rrN++;

      if (rrN >= 2) {
        int  n = (rrN < RR_BEAT_SIZE) ? rrN : RR_BEAT_SIZE;
        long s = 0;
        for (int j = 0; j < n; j++) s += rrBuf[j];
        int hrB = (int)(60000L / (s / n));

        if (hrB >= 40 && hrB <= 180) {
          if (xSemaphoreTake(healthMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            healthData.heartRate = hrB;
            healthData.hrValid   = true;
            xSemaphoreGive(healthMutex);
          }
        }
      }
    }
  }
  lastBeatMs = now;
}

// ─────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────

bool max30102Init() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("[MAX30102] CANH BAO: Khong tim thay cam bien.");
    return false;
  }

  particleSensor.setup(LED_BRIGHTNESS, SAMPLE_AVERAGE, LED_MODE,
                       SAMPLE_RATE, PULSE_WIDTH, ADC_RANGE);
  particleSensor.setPulseAmplitudeRed(0x18);
  particleSensor.setPulseAmplitudeIR(0x18);
  particleSensor.setPulseAmplitudeGreen(0);

  max30102Ready = true;
  Serial.println("[MAX30102] OK");
  return true;
}

void TaskMAX30102(void *pvParameters) {
  uint32_t irBuffer[BUFFER_LENGTH];
  uint32_t redBuffer[BUFFER_LENGTH];
  int32_t  spo2, heartRate;
  int8_t   validSPO2, validHeartRate;

  while (true) {
    // ── Trạng thái ban đầu: chờ ngón tay ──────────────────────────────
    unsigned long lastBeatMs          = 0;
    long          rrBuf[RR_BEAT_SIZE] = {};
    int           rrHead = 0, rrN     = 0;

    Serial.println("[MAX30102] Cho ngon tay...");

    // ── Thu BUFFER_LENGTH mẫu để khởi động ────────────────────────────
    int collected = 0;
    while (collected < BUFFER_LENGTH) {
      uint32_t ir, red;
      if (readOneSample(ir, red)) {
        if (!isFingerPresent(ir)) {
          // Mất ngón tay → reset toàn bộ
          collected = 0;
          memset(irBuffer,  0, sizeof(irBuffer));
          memset(redBuffer, 0, sizeof(redBuffer));
          resetHealthData();
          lastBeatMs = 0; rrN = 0; rrHead = 0;
          memset(rrBuf, 0, sizeof(rrBuf));
          vTaskDelay(pdMS_TO_TICKS(200));
          continue;
        }
        processBeat(ir, lastBeatMs, rrBuf, rrHead, rrN);
        irBuffer[collected]  = ir;
        redBuffer[collected] = red;
        collected++;
      } else {
        vTaskDelay(pdMS_TO_TICKS(5));
      }
    }

    // ── Tính SpO2 ban đầu ─────────────────────────────────────────────
    Serial.println("[MAX30102] Du 100 mau, bat dau tinh SpO2...");
    maxim_heart_rate_and_oxygen_saturation(
      irBuffer, BUFFER_LENGTH, redBuffer,
      &spo2, &validSPO2, &heartRate, &validHeartRate
    );
    if (xSemaphoreTake(healthMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      if (validSPO2 && spo2 >= 70 && spo2 <= 100) {
        healthData.spo2      = (int)spo2;
        healthData.spo2Valid = true;
      }
      xSemaphoreGive(healthMutex);
    }

    // ── Vòng lặp liên tục: sliding window ─────────────────────────────
    while (true) {
      // Dịch buffer: bỏ NEW_SAMPLES_PER_CYCLE mẫu cũ nhất
      for (int i = NEW_SAMPLES_PER_CYCLE; i < BUFFER_LENGTH; i++) {
        irBuffer[i  - NEW_SAMPLES_PER_CYCLE] = irBuffer[i];
        redBuffer[i - NEW_SAMPLES_PER_CYCLE] = redBuffer[i];
      }

      // Thu thêm NEW_SAMPLES_PER_CYCLE mẫu mới
      int  newCollected = 0;
      bool fingerLost   = false;

      while (newCollected < NEW_SAMPLES_PER_CYCLE) {
        uint32_t ir, red;
        if (readOneSample(ir, red)) {
          if (!isFingerPresent(ir)) { fingerLost = true; break; }

          processBeat(ir, lastBeatMs, rrBuf, rrHead, rrN);

          irBuffer[BUFFER_LENGTH  - NEW_SAMPLES_PER_CYCLE + newCollected] = ir;
          redBuffer[BUFFER_LENGTH - NEW_SAMPLES_PER_CYCLE + newCollected] = red;
          newCollected++;
        } else {
          vTaskDelay(pdMS_TO_TICKS(5));
        }
      }

      if (fingerLost) {
        Serial.println("[MAX30102] Mat ngon tay, reset...");
        resetHealthData();
        lastBeatMs = 0; rrN = 0; rrHead = 0;
        memset(rrBuf, 0, sizeof(rrBuf));
        break;  // Quay về vòng ngoài → chờ ngón tay
      }

      // Tính lại HR & SpO2
      maxim_heart_rate_and_oxygen_saturation(
        irBuffer, BUFFER_LENGTH, redBuffer,
        &spo2, &validSPO2, &heartRate, &validHeartRate
      );

      if (xSemaphoreTake(healthMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (validSPO2 && spo2 >= 70 && spo2 <= 100) {
          healthData.spo2      = (int)spo2;
          healthData.spo2Valid = true;
        }
        if (validHeartRate && heartRate >= 40 && heartRate <= 180) {
          healthData.heartRate = medianHR((int)heartRate);
          healthData.hrValid   = true;
        }
        xSemaphoreGive(healthMutex);
      }

      Serial.printf("[MAX30102] HR=%d BPM | SPO2=%d%%\n",
                    healthData.hrValid  ? healthData.heartRate : -1,
                    healthData.spo2Valid ? healthData.spo2     : -1);
    }
  }
}
