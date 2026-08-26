/**
 * @file    shared_state.h
 * @brief   Trạng thái dùng chung giữa các module (struct, biến toàn cục, mutex).
 *
 * Các module chỉ cần `#include "shared_state.h"` để truy cập dữ liệu cảm biến,
 * dữ liệu sức khỏe và các primitive đồng bộ FreeRTOS.
 */

#pragma once

#include <Arduino.h>
#include "freertos/semphr.h"
#include <math.h>

// ─────────────────────────────────────────────
// Struct dữ liệu
// ─────────────────────────────────────────────

/** Dữ liệu sức khỏe tổng hợp (cập nhật bởi MAX30102 và LM75). */
struct HealthData {
  int   heartRate = 0;     ///< Nhịp tim (BPM)
  int   spo2      = 0;     ///< Độ bão hoà oxy (%)
  bool  hrValid   = false; ///< Nhịp tim có hợp lệ không
  bool  spo2Valid = false; ///< SpO2 có hợp lệ không
  float bodyTemp  = -1.0f; ///< Nhiệt độ cơ thể (°C), -1 = chưa đọc được
};

/** Dữ liệu thô từ MPU6050 sau khi quy đổi đơn vị. */
struct SensorData {
  float accX,  accY,  accZ;  ///< Gia tốc 3 trục (g)
  float gyroX, gyroY, gyroZ; ///< Tốc độ góc 3 trục (°/s)
  float accMag;              ///< Độ lớn gia tốc tổng (g)
  float gyroMag;             ///< Độ lớn tốc độ góc tổng (°/s)
};

/** Dữ liệu đã lọc EMA — dùng cho logic phát hiện té ngã. */
struct FilteredData {
  float accMag  = 1.0f; ///< accMag sau lọc (khởi tạo = 1g khi đứng yên)
  float gyroMag = 0.0f; ///< gyroMag sau lọc
};

// ─────────────────────────────────────────────
// Biến trạng thái toàn cục (định nghĩa trong shared_state.cpp)
// ─────────────────────────────────────────────

extern HealthData   healthData;   ///< Dữ liệu sức khỏe hiện tại
extern SensorData   sensorData;   ///< Dữ liệu cảm biến thô hiện tại
extern FilteredData filteredData; ///< Dữ liệu cảm biến sau lọc EMA

extern SemaphoreHandle_t healthMutex; ///< Bảo vệ healthData giữa các task
extern SemaphoreHandle_t i2cMutex;   ///< Bảo vệ bus I2C giữa các task

extern bool    wifiOk;        ///< WiFi đang kết nối
extern bool    mqttOk;        ///< MQTT đang kết nối
extern bool    max30102Ready; ///< MAX30102 đã khởi tạo thành công
extern bool    lm75Ready;     ///< LM75 đã tìm thấy trên bus I2C
extern uint8_t lm75Addr;      ///< Địa chỉ I2C của LM75 (0x48..0x4F)

// ─────────────────────────────────────────────
// Hàm tiện ích chung
// ─────────────────────────────────────────────

/**
 * @brief Delay không chặn hoàn toàn — nhường CPU mỗi 10ms.
 * @param ms Thời gian chờ (ms)
 */
void yieldDelay(unsigned long ms);

/**
 * @brief Căn bậc hai an toàn (trả về 0 nếu v <= 0).
 * @param v Giá trị đầu vào
 * @return  sqrtf(v) hoặc 0.0f
 */
static inline float safeSqrt(float v) {
  return (v > 0.0f) ? sqrtf(v) : 0.0f;
}
