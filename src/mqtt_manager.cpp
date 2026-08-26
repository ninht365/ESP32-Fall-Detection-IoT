/**
 * @file    mqtt_manager.cpp
 * @brief   Triển khai module WiFi + MQTT — kết nối TLS và publish dữ liệu.
 *
 * espClient và mqttClient được giữ là static (file-scope) để ẩn chi tiết
 * thư viện khỏi các module khác. Chỉ giao tiếp qua API công khai.
 */

#include "mqtt_manager.h"
#include "shared_state.h"
#include "config.h"

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "esp_wifi.h"
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ─────────────────────────────────────────────
// Đối tượng phần cứng / thư viện (file-scope)
// ─────────────────────────────────────────────
static WiFiClientSecure espClient;
static PubSubClient     mqttClient(espClient);

// ─────────────────────────────────────────────
// Hàm nội bộ
// ─────────────────────────────────────────────

/** Quét và liệt kê các mạng WiFi 2.4 GHz trong vùng phủ sóng. */
static void scanWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  Serial.println("[WiFi] Dang quet mang 2.4GHz...");

  int n = WiFi.scanNetworks();
  if (n == 0) {
    Serial.println("[WiFi] KHONG thay mang nao.");
  } else {
    Serial.printf("[WiFi] Thay %d mang:\n", n);
    for (int i = 0; i < n; i++) {
      Serial.printf("   %2d) '%s'  (RSSI %d)%s\n", i + 1,
                    WiFi.SSID(i).c_str(), WiFi.RSSI(i),
                    WiFi.SSID(i) == WIFI_SSID ? "  <<<" : "");
    }
  }
  WiFi.scanDelete();
}

// ─────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────

void mqttManagerInit() {
  // Bỏ qua xác minh certificate (phù hợp cho dev/prototype)
  espClient.setInsecure();
}

bool connectWiFi() {
  wifiOk = false;

  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  delay(200);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);

  scanWiFi();

  // Ép dùng 802.11b/g/n (tránh vấn đề tương thích với một số AP cũ)
  esp_wifi_set_protocol(WIFI_IF_STA,
      WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  Serial.printf("[WiFi] Dang ket noi: %s (TXpwr=%d)\n",
                WIFI_SSID, WiFi.getTxPower());
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  esp_wifi_set_ps(WIFI_PS_NONE);  // Tắt power-save để giảm latency

  unsigned long start = millis();
  int lastSt = -1;
  while (WiFi.status() != WL_CONNECTED) {
    int st = WiFi.status();
    if (st != lastSt) { Serial.printf("\n[WiFi] status=%d ", st); lastSt = st; }
    if (millis() - start > WIFI_TIMEOUT_MS) {
      Serial.printf("\n[WiFi] Timeout! (status cuoi=%d)\n", st);
      return false;
    }
    delay(300);
    Serial.print(".");
  }
  Serial.printf("\n[WiFi] Ket noi thanh cong! IP: %s\n",
                WiFi.localIP().toString().c_str());
  wifiOk = true;
  return true;
}

bool connectMQTT() {
  if (!wifiOk || WiFi.status() != WL_CONNECTED) return false;

  mqttOk = false;
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setBufferSize(1024);
  mqttClient.setKeepAlive(MQTT_KEEPALIVE_SEC);
  mqttClient.setSocketTimeout(MQTT_SOCKET_TIMEOUT);

  // Client ID ngẫu nhiên để tránh xung đột khi có nhiều thiết bị cùng MQTT_CLIENT
  char clientId[48];
  snprintf(clientId, sizeof(clientId), "%s_%04x",
           MQTT_CLIENT, (unsigned)random(0xffff));

  Serial.print("[MQTT] Dang ket noi");
  for (int i = 0; i < MQTT_MAX_RETRY; i++) {
    if (mqttClient.connect(clientId, MQTT_USER, MQTT_PASS)) {
      mqttOk = true;
      Serial.println(" -> OK!");
      return true;
    }
    Serial.printf(" -> That bai (ma %d), thu lai %d/%d\n",
                  mqttClient.state(), i + 1, MQTT_MAX_RETRY);
    yieldDelay(MQTT_RETRY_DELAY);
  }
  Serial.println("[MQTT] Ket noi that bai sau nhieu lan thu!");
  return false;
}

void handleWiFi() {
  if (WiFi.status() == WL_CONNECTED) { wifiOk = true; return; }

  wifiOk = false;
  mqttOk = false;
  Serial.println("[WiFi] Mat ket noi, thu lai...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > WIFI_TIMEOUT_MS) {
      Serial.println("\n[WiFi] Reconnect timeout.");
      return;
    }
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\n[WiFi] Ket noi lai! IP: %s\n",
                WiFi.localIP().toString().c_str());
  wifiOk = true;
}

void handleMQTT() {
  if (!wifiOk || WiFi.status() != WL_CONNECTED) return;

  if (!mqttClient.connected()) {
    mqttOk = false;
    static unsigned long lastReconn = 0;
    unsigned long now = millis();
    if (now - lastReconn < 5000UL) return;  // Throttle reconnect attempts
    lastReconn = now;
    Serial.printf("[MQTT] Mat ket noi (ma %d), thu lai...\n", mqttClient.state());
    connectMQTT();
  }

  if (mqttOk) mqttClient.loop();
}

void sendDataToCloud(const String &alert_status) {
#if ENABLE_WIFI
  if (!mqttOk || !mqttClient.connected()) return;
#endif

  // Đọc snapshot dữ liệu sức khỏe (thread-safe)
  int   hr   = -1;
  int   sp   = -1;
  float temp = -1.0f;
  if (xSemaphoreTake(healthMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    hr   = healthData.hrValid   ? healthData.heartRate : -1;
    sp   = healthData.spo2Valid ? healthData.spo2      : -1;
    temp = healthData.bodyTemp;
    xSemaphoreGive(healthMutex);
  }

  // Bảo vệ khỏi NaN/Inf trước khi serialize JSON
  auto safeFloat = [](float v, float fb = -1.0f) -> float {
    return (isnan(v) || isinf(v)) ? fb : v;
  };

  JsonDocument doc;
  doc["ten"]      = "Ong Nguyen Van A";
  doc["nhip_tim"] = hr;
  doc["spo2"]     = sp;
  doc["nhiet_do"] = safeFloat(temp);
  doc["canh_bao"] = alert_status;
  doc["ax"]       = safeFloat(sensorData.accX,  0.0f);
  doc["ay"]       = safeFloat(sensorData.accY,  0.0f);
  doc["az"]       = safeFloat(sensorData.accZ,  0.0f);
  doc["gx"]       = safeFloat(sensorData.gyroX, 0.0f);
  doc["gy"]       = safeFloat(sensorData.gyroY, 0.0f);
  doc["gz"]       = safeFloat(sensorData.gyroZ, 0.0f);

  char buffer[600];
  serializeJson(doc, buffer);

#if ENABLE_WIFI
  bool ok = mqttClient.publish(TOPIC_FALL, buffer);
  Serial.printf("[MQTT] publish=%s (conn=%d, state=%d, len=%d) topic=%s\n",
                ok ? "OK" : "FAIL", mqttClient.connected(), mqttClient.state(),
                (int)strlen(buffer), TOPIC_FALL);
#else
  Serial.printf("[DATA] %s\n", buffer);
#endif
}
