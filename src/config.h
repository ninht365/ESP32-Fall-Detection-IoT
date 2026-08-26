/**
 * @file    config.h
 * @brief   Tập trung toàn bộ hằng số cấu hình của hệ thống Elder Care.
 *
 * Mọi ngưỡng thuật toán, thông số phần cứng, thông tin kết nối WiFi/MQTT
 * đều được khai báo tại đây để dễ tra cứu và chỉnh sửa một chỗ.
 */

#pragma once

// ─────────────────────────────────────────────
// I2C Bus
// ─────────────────────────────────────────────
#define SDA_PIN 8
#define SCL_PIN 9

// ─────────────────────────────────────────────
// WiFi
// ─────────────────────────────────────────────
#define ENABLE_WIFI 1

#define WIFI_SSID           "SSID"
#define WIFI_PASSWORD       "PASSWORD"
#define WIFI_TIMEOUT_MS     20000UL
#define WIFI_CHECK_INTERVAL 30000UL

// ─────────────────────────────────────────────
// MQTT (HiveMQ Cloud — TLS 8883)
// ─────────────────────────────────────────────
#define MQTT_HOST           "7c533fddea754db19e6afd1e297cf2f8.s1.eu.hivemq.cloud"
#define MQTT_PORT           8883
#define MQTT_USER           "eldercare_device"
#define MQTT_PASS           "Nhom1@123456"
#define MQTT_CLIENT         "ESP32_FallDetect"
#define TOPIC_FALL          "eldercare/test_nga"
#define MQTT_RETRY_DELAY    3000UL
#define MQTT_MAX_RETRY      2
#define MQTT_KEEPALIVE_SEC  60
#define MQTT_SOCKET_TIMEOUT 5

// ─────────────────────────────────────────────
// Phát hiện té ngã — Ngưỡng & Thời gian
// ─────────────────────────────────────────────
/** Gia tốc tổng (g) nhỏ hơn giá trị này → nghi ngờ rơi tự do */
#define FREE_FALL_THRESHOLD  0.3f
/** Gia tốc tổng (g) lớn hơn giá trị này → xác nhận va chạm */
#define IMPACT_THRESHOLD     2.5f
/** Độ lệch gia tốc so với 1g để xác định đứng yên */
#define STILL_ACC_THRESHOLD  0.15f
/** Tốc độ góc (°/s) tối đa để xác định đứng yên */
#define STILL_GYRO_THRESHOLD 20.0f

/** Thời gian đứng yên tối thiểu để xác nhận té ngã (ms) */
#define STILL_DURATION_MS   2000UL
/** Thời gian rơi tự do tối thiểu (ms) — dưới ngưỡng này là nhiễu */
#define FREE_FALL_MIN_MS     100UL
/** Timeout chờ va chạm sau rơi tự do (ms) */
#define IMPACT_TIMEOUT_MS   1000UL
/** Timeout chờ đứng yên sau va chạm (ms) */
#define STILL_TIMEOUT_MS    3000UL
/** Thời gian nghỉ giữa hai lần phát hiện (ms) */
#define FALL_COOLDOWN_MS    3000UL

// ─────────────────────────────────────────────
// MAX30102 — Đo nhịp tim & SpO2
// ─────────────────────────────────────────────
/** Ngưỡng IR tối thiểu để xác nhận có ngón tay */
#define FINGER_THRESHOLD       30000UL
/** Stack size cho FreeRTOS task của MAX30102 (bytes) */
#define MAX30102_STACK_SIZE    16384
/** Số mẫu trong buffer SpO2 */
#define BUFFER_LENGTH          100
/** Số mẫu mới thu thêm mỗi chu kỳ tính toán */
#define NEW_SAMPLES_PER_CYCLE  25
/** Độ sáng LED (0–255) */
#define LED_BRIGHTNESS         60
/** Số mẫu trung bình phần cứng */
#define SAMPLE_AVERAGE         4
/** Chế độ LED (1=Red only, 2=Red+IR, 3=Red+IR+Green) */
#define LED_MODE               2
/** Tần số lấy mẫu (Hz) */
#define SAMPLE_RATE            100
/** Độ rộng xung LED (μs): 69, 118, 215, 411 */
#define PULSE_WIDTH            411
/** Dải ADC: 2048, 4096, 8192, 16384 */
#define ADC_RANGE              4096

// ─────────────────────────────────────────────
// LM75 — Đo nhiệt độ cơ thể
// ─────────────────────────────────────────────
/** Thanh ghi nhiệt độ của LM75 */
#define LM75_REG_TEMP         0x00
/** Chu kỳ đọc nhiệt độ (ms) */
#define LM75_READ_INTERVAL_MS 5000UL
