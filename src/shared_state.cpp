/**
 * @file    shared_state.cpp
 * @brief   Định nghĩa (allocation) các biến toàn cục dùng chung.
 */

#include "shared_state.h"

// ─────────────────────────────────────────────
// Định nghĩa biến toàn cục
// ─────────────────────────────────────────────

HealthData   healthData;
SensorData   sensorData;
FilteredData filteredData;

SemaphoreHandle_t healthMutex = NULL;
SemaphoreHandle_t i2cMutex   = NULL;

bool    wifiOk        = false;
bool    mqttOk        = false;
bool    max30102Ready = false;
bool    lm75Ready     = false;
uint8_t lm75Addr      = 0x00;

// ─────────────────────────────────────────────
// Hàm tiện ích
// ─────────────────────────────────────────────

void yieldDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) { delay(10); }
}
