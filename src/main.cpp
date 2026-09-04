/**
 * @file    main.cpp
 * @brief   Điểm vào chính của firmware Elder Care (ESP32-S3).
 *
 * File này chỉ chứa setup() và loop(). Mọi logic được uỷ thác
 * cho các module riêng biệt:
 *
 *  ┌─────────────────┐  ┌──────────────────┐  ┌──────────────────┐
 *  │  mpu6050_module │  │   lm75_module    │  │ max30102_module  │
 *  │  (gia tốc/gyro) │  │  (nhiệt độ cơ    │  │  (HR + SpO2)     │
 *  └────────┬────────┘  │   thể)           │  │  FreeRTOS Task   │
 *           │           └────────┬─────────┘  └────────┬─────────┘
 *           │                    │                     │
 *           ▼                    ▼                     ▼
 *      ┌────────────────────────────────────────────────────┐
 *      │               shared_state (mutex, structs)        │
 *      └──────────────────────────┬─────────────────────────┘
 *                                 │
 *           ┌─────────────────────┴──────────────────┐
 *           ▼                                        ▼
 *  ┌─────────────────┐                    ┌──────────────────┐
 *  │ fall_detection  │──sendDataToCloud──►│  mqtt_manager    │
 *  │ (state machine) │                    │  (WiFi + MQTT)   │
 *  └─────────────────┘                    └──────────────────┘
 */

#include <Wire.h>
#include "config.h"
#include "shared_state.h"
#include "mpu6050_module.h"
#include "lm75_module.h"
#include "max30102_module.h"
#include "fall_detection.h"
#include "mqtt_manager.h"

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(1000);
  Serial.println("\n=== KHOI DONG HE THONG ===");

  // 1. Khởi tạo mutex FreeRTOS
  healthMutex = xSemaphoreCreateMutex();
  i2cMutex    = xSemaphoreCreateMutex();
  if (!healthMutex || !i2cMutex) {
    Serial.println("[FATAL] Khong tao duoc mutex!");
    while (1) delay(1000);
  }

  // 2. Khởi tạo I2C bus
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);

  // 3. Khởi tạo cảm biến MPU6050 (bắt buộc — dừng nếu lỗi)
  if (!mpu6050Init()) {
    while (1) delay(1000);
  }

  // 4. Khởi tạo cảm biến nhiệt độ LM75 (không bắt buộc)
  lm75Init();

  // 5. Khởi tạo cảm biến nhịp tim MAX30102 (không bắt buộc)
  max30102Init();

  // 6. Kết nối WiFi & MQTT
#if ENABLE_WIFI
  mqttManagerInit();
  if (connectWiFi()) {
    yieldDelay(1000);
    connectMQTT();
  } else {
    Serial.println("[Setup] Khong co WiFi - chay offline");
  }
#else
  Serial.println("[Setup] WiFi/MQTT DANG TAT (ENABLE_WIFI=0)");
#endif

  // 7. Tạo FreeRTOS task MAX30102 trên core 0 (tách biệt khỏi loop() ở core 1)
  if (max30102Ready) {
    xTaskCreatePinnedToCore(TaskMAX30102, "MAX30102",
                            MAX30102_STACK_SIZE, NULL, 1, NULL, 0);
  }

  Serial.println("\n[System] Bat dau giam sat...\n");
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // ── Quản lý kết nối mạng ──────────────────────────────────────────────────
#if ENABLE_WIFI
  handleMQTT();

  static unsigned long lastWiFiCheck = 0;
  if (now - lastWiFiCheck > WIFI_CHECK_INTERVAL) {
    lastWiFiCheck = now;
    handleWiFi();
  }
#endif

  // ── Đọc cảm biến và xử lý thuật toán ────────────────────────────────────
  readSensor();
  handleFallDetection();
  handleLM75();

  // ── Gửi dữ liệu định kỳ khi bình thường (1 Hz) ───────────────────────────
  static unsigned long lastNormalSend = 0;
  if (getFallPhase() == PHASE_IDLE && now - lastNormalSend > 1000) {
    lastNormalSend = now;
    sendDataToCloud("Binh thuong");
  }

  // ── In thông số sức khỏe ra Serial mỗi 3 giây ────────────────────────────
  static unsigned long lastHealthPrint = 0;
  if (now - lastHealthPrint > 3000) {
    lastHealthPrint = now;
    int hr = -1, sp = -1;
    float temp = -1.0f;
    if (xSemaphoreTake(healthMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      hr   = healthData.hrValid   ? healthData.heartRate : -1;
      sp   = healthData.spo2Valid ? healthData.spo2      : -1;
      temp = healthData.bodyTemp;
      xSemaphoreGive(healthMutex);
    }
    Serial.printf("[HEALTH] Temp=%.2fC | HR=%d | SPO2=%d%%\n", temp, hr, sp);
  }

  delay(10);
}
