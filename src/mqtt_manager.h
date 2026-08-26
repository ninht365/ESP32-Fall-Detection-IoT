/**
 * @file    mqtt_manager.h
 * @brief   Interface module quản lý WiFi và MQTT.
 *
 * Cung cấp:
 *  - mqttManagerInit():  cấu hình TLS client, gọi 1 lần trong setup().
 *  - connectWiFi():      scan và kết nối WiFi.
 *  - connectMQTT():      kết nối MQTT broker với retry.
 *  - handleWiFi():       kiểm tra và reconnect WiFi khi mất kết nối.
 *  - handleMQTT():       keepalive MQTT loop và tự động reconnect.
 *  - sendDataToCloud():  serialize và publish JSON payload lên broker.
 */

#pragma once

#include <Arduino.h>

/**
 * @brief Cấu hình WiFiClientSecure (bỏ qua xác thực certificate).
 *        Phải gọi trước connectWiFi() / connectMQTT().
 */
void mqttManagerInit();

/**
 * @brief Scan WiFi và kết nối tới WIFI_SSID.
 * @return true nếu kết nối thành công trong WIFI_TIMEOUT_MS.
 */
bool connectWiFi();

/**
 * @brief Kết nối tới MQTT broker (retry tối đa MQTT_MAX_RETRY lần).
 * @return true nếu kết nối thành công.
 */
bool connectMQTT();

/**
 * @brief Kiểm tra trạng thái WiFi, tự kết nối lại nếu mất.
 *        Gọi định kỳ trong loop() theo WIFI_CHECK_INTERVAL.
 */
void handleWiFi();

/**
 * @brief Chạy vòng lặp MQTT (keepalive) và reconnect khi mất kết nối.
 *        Gọi mỗi vòng loop().
 */
void handleMQTT();

/**
 * @brief Đóng gói dữ liệu sức khỏe + cảm biến thành JSON và publish lên TOPIC_FALL.
 * @param alert_status Chuỗi trạng thái cảnh báo (e.g., "Binh thuong", "Phat hien te nga khan cap").
 */
void sendDataToCloud(const String &alert_status);
