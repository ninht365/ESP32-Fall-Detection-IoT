/**
 * @file    mpu6050_module.cpp
 * @brief   Triển khai module MPU6050.
 *
 * Đối tượng MPU6050 được giữ là static (file-scope) để che giấu
 * chi tiết phần cứng khỏi các module khác.
 */

#include "mpu6050_module.h"
#include "shared_state.h"
#include "config.h"

#include <Wire.h>
#include "MPU6050.h"

// ─────────────────────────────────────────────
// Đối tượng phần cứng (chỉ dùng trong file này)
// ─────────────────────────────────────────────
static MPU6050 mpu;

// ─────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────

bool mpu6050Init() {
  mpu.initialize();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);  // ±8g  → LSB = 4096
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);  // ±500°/s → LSB = 65.5
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);             // Low-pass 42 Hz

  if (!mpu.testConnection()) {
    Serial.println("[MPU6050] THAT BAI!");
    return false;
  }
  Serial.println("[MPU6050] OK");

  // Warm-up: đọc 10 mẫu để bộ lọc EMA ổn định
  for (int i = 0; i < 10; i++) { readSensor(); delay(10); }
  Serial.printf("[MPU6050] Khoi tao: acc=%.2fg gyro=%.1f\n",
                filteredData.accMag, filteredData.gyroMag);
  return true;
}

void readSensor() {
  int16_t ax, ay, az, gx, gy, gz;

  // Lấy mutex I2C trước khi giao tiếp với bus
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  xSemaphoreGive(i2cMutex);

  // Quy đổi đơn vị
  sensorData.accX  = (float)ax / 4096.0f;   // g
  sensorData.accY  = (float)ay / 4096.0f;
  sensorData.accZ  = (float)az / 4096.0f;
  sensorData.gyroX = (float)gx / 65.5f;     // °/s
  sensorData.gyroY = (float)gy / 65.5f;
  sensorData.gyroZ = (float)gz / 65.5f;

  // Độ lớn vector
  sensorData.accMag = safeSqrt(
    sensorData.accX * sensorData.accX +
    sensorData.accY * sensorData.accY +
    sensorData.accZ * sensorData.accZ
  );
  sensorData.gyroMag = safeSqrt(
    sensorData.gyroX * sensorData.gyroX +
    sensorData.gyroY * sensorData.gyroY +
    sensorData.gyroZ * sensorData.gyroZ
  );

  // Lọc EMA (α=0.6 cho giá trị mới — phản ứng nhanh)
  filteredData.accMag  = 0.4f * filteredData.accMag  + 0.6f * sensorData.accMag;
  filteredData.gyroMag = 0.4f * filteredData.gyroMag + 0.6f * sensorData.gyroMag;
}
