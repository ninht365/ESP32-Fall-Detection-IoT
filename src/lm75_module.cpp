/**
 * @file    lm75_module.cpp
 * @brief   Triển khai module LM75 — đọc nhiệt độ qua I2C.
 *
 * Giao tiếp trực tiếp với thanh ghi 0x00 của LM75 theo định dạng
 * 11-bit two's complement, độ phân giải 0.125°C/LSB.
 */

#include "lm75_module.h"
#include "shared_state.h"
#include "config.h"

#include <Wire.h>

// ─────────────────────────────────────────────
// Hàm nội bộ
// ─────────────────────────────────────────────

/**
 * @brief Đọc nhiệt độ thô từ thanh ghi LM75_REG_TEMP.
 * @return Nhiệt độ (°C), hoặc -100.0f nếu có lỗi I2C.
 */
static float lm75ReadTemp() {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) != pdTRUE) return -100.0f;

  Wire.beginTransmission(lm75Addr);
  Wire.write(LM75_REG_TEMP);
  if (Wire.endTransmission(true) != 0)   { xSemaphoreGive(i2cMutex); return -100.0f; }

  Wire.requestFrom(lm75Addr, (uint8_t)2);
  if (Wire.available() < 2)              { xSemaphoreGive(i2cMutex); return -100.0f; }

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  xSemaphoreGive(i2cMutex);

  // 11-bit two's complement, MSB-first, dịch phải 5 bit
  int16_t raw = ((int16_t)msb << 8) | lsb;
  raw >>= 5;
  return raw * 0.125f;  // 0.125°C / LSB
}

// ─────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────

bool lm75Init() {
  lm75Addr = 0x00;
  Serial.print("[LM75] Quet 0x48..0x4F: ");

  for (uint8_t a = 0x48; a <= 0x4F; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("0x%02X ", a);
      if (lm75Addr == 0x00) lm75Addr = a;  // Lấy địa chỉ đầu tiên tìm thấy
    }
  }
  Serial.println();

  if (lm75Addr != 0x00) {
    lm75Ready = true;
    float t = lm75ReadTemp();
    if (t > -50.0f) {
      if (xSemaphoreTake(healthMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        healthData.bodyTemp = t;
        xSemaphoreGive(healthMutex);
      }
    }
    Serial.printf("[LM75] OK @ 0x%02X - %.2f C\n", lm75Addr, t);
    return true;
  }

  Serial.println("[LM75] CANH BAO: Khong thay trong dai 0x48..0x4F.");
  return false;
}

void handleLM75() {
  if (!lm75Ready) return;

  static unsigned long lastRead = 0;
  if (millis() - lastRead < LM75_READ_INTERVAL_MS) return;
  lastRead = millis();

  float t = lm75ReadTemp();
  if (t > -50.0f) {
    if (xSemaphoreTake(healthMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      healthData.bodyTemp = t;
      xSemaphoreGive(healthMutex);
    }
    Serial.printf("[LM75] %.2f C\n", t);
  } else {
    Serial.println("[LM75] ERROR");
  }
}
