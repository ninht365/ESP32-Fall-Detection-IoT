#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "esp_wifi.h"
#include "MPU6050.h"
#include <PubSubClient.h>
#include <math.h>
#include <ArduinoJson.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include "freertos/semphr.h"
#include <math.h>
#include <stdarg.h>

#define SDA_PIN 8
#define SCL_PIN 9

#define ENABLE_WIFI 1

#define WIFI_SSID        "iot"
#define WIFI_PASSWORD    "abcd1234"
#define WIFI_TIMEOUT_MS  20000UL

#define MQTT_HOST             "7c533fddea754db19e6afd1e297cf2f8.s1.eu.hivemq.cloud"
#define MQTT_PORT             8883
#define MQTT_USER             "eldercare_device"
#define MQTT_PASS             "Nhom1@123456"
#define MQTT_CLIENT           "ESP32_FallDetect"
#define TOPIC_FALL            "eldercare/test_nga"
#define TOPIC_DEBUG           "eldercare/debug"
#define MQTT_RETRY_DELAY      3000UL
#define MQTT_MAX_RETRY        2
#define MQTT_KEEPALIVE_SEC    60
#define MQTT_SOCKET_TIMEOUT   5

#define FREE_FALL_THRESHOLD   0.6f
#define IMPACT_THRESHOLD      1.7f
#define STILL_ACC_THRESHOLD   0.15f
#define STILL_GYRO_THRESHOLD  44.0f

#define STILL_DURATION_MS     1500UL
#define FREE_FALL_MIN_MS       100UL
#define IMPACT_TIMEOUT_MS     1000UL
#define STILL_TIMEOUT_MS      3000UL
#define FALL_COOLDOWN_MS      3000UL
#define WIFI_CHECK_INTERVAL   30000UL

#define FINGER_THRESHOLD      30000UL
#define MAX30102_STACK_SIZE   16384
#define BUFFER_LENGTH         100
#define NEW_SAMPLES_PER_CYCLE  25
#define LED_BRIGHTNESS        60
#define SAMPLE_AVERAGE        4
#define LED_MODE              2
#define SAMPLE_RATE           100
#define PULSE_WIDTH           411
#define ADC_RANGE             4096

#define LM75_REG_TEMP         0x00
#define LM75_READ_INTERVAL_MS 5000UL

static WiFiClientSecure espClient;
static PubSubClient     mqttClient(espClient);
static MPU6050          mpu;
static MAX30105         particleSensor;

static bool wifiOk        = false;
static bool mqttOk        = false;
static bool max30102Ready = false;
static bool lm75Ready     = false;
static uint8_t lm75Addr   = 0x00;

static SemaphoreHandle_t healthMutex = NULL;
static SemaphoreHandle_t i2cMutex   = NULL;

struct HealthData {
  int   heartRate = 0;
  int   spo2      = 0;
  bool  hrValid   = false;
  bool  spo2Valid = false;
  float bodyTemp  = -1.0f;
};

static HealthData healthData;
static void mqttDebug(const char* msg)
{
  Serial.println(msg);

#if ENABLE_WIFI
  if (mqttOk && mqttClient.connected()) {
    mqttClient.publish(TOPIC_DEBUG, msg);
  }
#endif
}

static void mqttDebugf(const char *fmt, ...)
{
  char buffer[200];

  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  mqttDebug(buffer);
}

static void yieldDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) { delay(10); }
}

static inline float safeSqrt(float v) {
  return (v > 0.0f) ? sqrtf(v) : 0.0f;
}

struct SensorData {
  float accX, accY, accZ;
  float gyroX, gyroY, gyroZ;
  float accMag;
  float gyroMag;
};

struct FilteredData {
  float accMag  = 1.0f;
  float gyroMag = 0.0f;
  float accX    = 0.0f;  
  float accY    = 0.0f;  
  float accZ    = 1.0f;
};

static SensorData   sensorData;
static FilteredData filteredData;

static void readSensor() {
  int16_t ax, ay, az, gx, gy, gz;

  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  xSemaphoreGive(i2cMutex);

  sensorData.accX  = (float)ax / 4096.0f;
  sensorData.accY  = (float)ay / 4096.0f;
  sensorData.accZ  = (float)az / 4096.0f;
  sensorData.gyroX = (float)gx / 65.5f;
  sensorData.gyroY = (float)gy / 65.5f;
  sensorData.gyroZ = (float)gz / 65.5f;

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

filteredData.accX    = 0.4f * filteredData.accX    + 0.6f * sensorData.accX;
filteredData.accY    = 0.4f * filteredData.accY    + 0.6f * sensorData.accY;
filteredData.accZ    = 0.4f * filteredData.accZ    + 0.6f * sensorData.accZ;
filteredData.accMag  = 0.4f * filteredData.accMag  + 0.6f * sensorData.accMag;
filteredData.gyroMag = 0.4f * filteredData.gyroMag + 0.6f * sensorData.gyroMag;
}

static float lm75ReadTemp() {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) != pdTRUE) return -100.0f;

  Wire.beginTransmission(lm75Addr);
  Wire.write(LM75_REG_TEMP);
  if (Wire.endTransmission(true) != 0)  { xSemaphoreGive(i2cMutex); return -100.0f; }
  Wire.requestFrom(lm75Addr, (uint8_t)2);
  if (Wire.available() < 2)            { xSemaphoreGive(i2cMutex); return -100.0f; }

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  xSemaphoreGive(i2cMutex);

  int16_t raw = ((int16_t)msb << 8) | lsb;
  raw >>= 5;
  return raw * 0.125f;
}

static void handleLM75() {
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

static bool isFingerPresent(uint32_t ir) { return ir > FINGER_THRESHOLD; }

static const int RR_BEAT_SIZE = 5;

static void processBeat(uint32_t        ir,
                         unsigned long  &lastBeatMs,
                         long           (&rrBuf)[RR_BEAT_SIZE],
                         int            &rrHead,
                         int            &rrN) {
  if (!checkForBeat(ir)) return;

  unsigned long now = millis();
  if (lastBeatMs > 0) {
    long rr = (long)(now - lastBeatMs);
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

static int medianHR(int newHr) {
  static int hist[5] = {0,0,0,0,0};
  static int cnt = 0;
  for (int i = 4; i > 0; i--) hist[i] = hist[i-1];
  hist[0] = newHr;
  if (cnt < 5) cnt++;
  int tmp[5], n = cnt;
  for (int i = 0; i < n; i++) tmp[i] = hist[i];
  for (int i = 0; i < n-1; i++)
    for (int j = 0; j < n-1-i; j++)
      if (tmp[j] > tmp[j+1]) { int t = tmp[j]; tmp[j] = tmp[j+1]; tmp[j+1] = t; }
  return tmp[n/2];
}

static void resetHealthData() {
  if (xSemaphoreTake(healthMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
    healthData.heartRate = 0;
    healthData.spo2      = 0;
    healthData.hrValid   = false;
    healthData.spo2Valid = false;
    xSemaphoreGive(healthMutex);
  }
}

void TaskMAX30102(void *pvParameters) {
  uint32_t irBuffer[BUFFER_LENGTH];
  uint32_t redBuffer[BUFFER_LENGTH];
  int32_t  spo2, heartRate;
  int8_t   validSPO2, validHeartRate;

  while (true) {

    unsigned long lastBeatMs          = 0;
    long          rrBuf[RR_BEAT_SIZE] = {};
    int           rrHead = 0, rrN     = 0;

    Serial.println("[MAX30102] Cho ngon tay...");

    int collected = 0;
    while (collected < BUFFER_LENGTH) {
      uint32_t ir, red;
      if (readOneSample(ir, red)) {
        if (!isFingerPresent(ir)) {
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

    while (true) {

      for (int i = NEW_SAMPLES_PER_CYCLE; i < BUFFER_LENGTH; i++) {
        irBuffer[i  - NEW_SAMPLES_PER_CYCLE] = irBuffer[i];
        redBuffer[i - NEW_SAMPLES_PER_CYCLE] = redBuffer[i];
      }

      int  newCollected = 0;
      bool fingerLost   = false;

      while (newCollected < NEW_SAMPLES_PER_CYCLE) {
        uint32_t ir, red;
        if (readOneSample(ir, red)) {
          if (!isFingerPresent(ir)) { fingerLost = true; break; }

          processBeat(ir, lastBeatMs, rrBuf, rrHead, rrN);

          irBuffer[BUFFER_LENGTH - NEW_SAMPLES_PER_CYCLE + newCollected]  = ir;
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
        break;
      }

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
                    healthData.hrValid ? healthData.heartRate : -1,
                    healthData.spo2Valid ? healthData.spo2 : -1);
    }
  }
}

static void sendDataToCloud(const String& alert_status) {
#if ENABLE_WIFI
  if (!mqttOk || !mqttClient.connected()) return;
#endif

  int   hr   = -1;
  int   sp   = -1;
  float temp = -1.0f;

  if (xSemaphoreTake(healthMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    hr   = healthData.hrValid   ? healthData.heartRate : -1;
    sp   = healthData.spo2Valid ? healthData.spo2      : -1;
    temp = healthData.bodyTemp;
    xSemaphoreGive(healthMutex);
  }

  auto safeFloat = [](float v, float fb = -1.0f) -> float {
    return (isnan(v) || isinf(v)) ? fb : v;
  };

  JsonDocument doc;
  doc["nhip_tim"] = hr;
  doc["spo2"]     = sp;
  doc["nhiet_do"] = safeFloat(temp);
  doc["canh_bao"] = alert_status;
  doc["ax"] = safeFloat(sensorData.accX, 0.0f);
  doc["ay"] = safeFloat(sensorData.accY, 0.0f);
  doc["az"] = safeFloat(sensorData.accZ, 0.0f);
  doc["gx"] = safeFloat(sensorData.gyroX, 0.0f);
  doc["gy"] = safeFloat(sensorData.gyroY, 0.0f);
  doc["gz"] = safeFloat(sensorData.gyroZ, 0.0f);

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

static bool connectMQTT() {
  if (!wifiOk || WiFi.status() != WL_CONNECTED) return false;

  mqttOk = false;
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setBufferSize(1024);
  mqttClient.setKeepAlive(MQTT_KEEPALIVE_SEC);
  mqttClient.setSocketTimeout(MQTT_SOCKET_TIMEOUT);

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

static void handleMQTT() {
  if (!wifiOk || WiFi.status() != WL_CONNECTED) return;
  if (!mqttClient.connected()) {
    mqttOk = false;
    static unsigned long lastReconn = 0;
    unsigned long now = millis();
    if (now - lastReconn < 5000UL) return;
    lastReconn = now;
    Serial.printf("[MQTT] Mat ket noi (ma %d), thu lai...\n", mqttClient.state());
    connectMQTT();
  }
  if (mqttOk) mqttClient.loop();
}

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

static bool connectWiFi() {
  wifiOk = false;

  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  delay(200);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);

  scanWiFi();

  esp_wifi_set_protocol(WIFI_IF_STA,
      WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  Serial.printf("[WiFi] Dang ket noi: %s (TXpwr=%d)\n", WIFI_SSID, WiFi.getTxPower());
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  esp_wifi_set_ps(WIFI_PS_NONE);

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

static void handleWiFi() {
  if (WiFi.status() == WL_CONNECTED) { wifiOk = true; return; }
  wifiOk = false; mqttOk = false;
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

enum FallPhase : uint8_t {
  PHASE_IDLE, PHASE_FREE_FALL, PHASE_IMPACT,
  PHASE_WAITING_STILL, PHASE_COOLDOWN
};

static FallPhase     fallPhase     = PHASE_IDLE;
static unsigned long phaseStart    = 0;
static unsigned long stillStart    = 0;
static unsigned long freeFallStart = 0;
static unsigned long cooldownStart = 0;
static int           fallCount     = 0;
static float preFallAx = 0.0f;
static float preFallAy = 0.0f;
static float preFallAz = 1.0f;

static void handleFallDetection() {
  unsigned long now = millis();

  switch (fallPhase) {
    case PHASE_IDLE:
      if(sensorData.accMag < FREE_FALL_THRESHOLD)
{
    preFallAx = filteredData.accX;
    preFallAy = filteredData.accY;
    preFallAz = filteredData.accZ;

    fallPhase = PHASE_FREE_FALL;
    phaseStart = now;

    mqttDebugf("[PHA1] ROI TU DO (acc=%.2fg)",
               sensorData.accMag);
}
      break;

    case PHASE_FREE_FALL:
      if (sensorData.accMag >= FREE_FALL_THRESHOLD) {
        unsigned long dur = now - freeFallStart;
        if (dur < FREE_FALL_MIN_MS) {
          fallPhase = PHASE_IDLE;
          //Serial.println("[Reset] Roi tu do qua ngan (nhieu)");
        } else {
          fallPhase = PHASE_IMPACT; phaseStart = now;
          mqttDebugf("[PHA1] XAC NHAN ROI TU DO (%lums)", dur);
        }
        break;
      }
      if (now - freeFallStart > 1000UL) {
        fallPhase = PHASE_IDLE;
        //Serial.println("[Reset] Roi tu do qua lau (timeout)");
      }
      break;

    case PHASE_IMPACT:
      if (now - phaseStart > IMPACT_TIMEOUT_MS) {
        fallPhase = PHASE_IDLE;
        //Serial.println("[Reset] Khong co va cham (timeout)");
        break;
      }
      if (sensorData.accMag > IMPACT_THRESHOLD) {
        mqttDebugf("[PHA2] VA CHAM (acc=%.2fg)", sensorData.accMag);
        fallPhase = PHASE_WAITING_STILL; phaseStart = now; stillStart = now;
      }
      break;

    case PHASE_WAITING_STILL:
{
    static unsigned long lastWaitLog = 0;

    if (now - lastWaitLog > 200) {
        lastWaitLog = now;

        mqttDebugf("[WAIT] acc=%.2f gyro=%.2f",
                   filteredData.accMag,
                   filteredData.gyroMag);
    }

    if (now - phaseStart > STILL_TIMEOUT_MS) {
        fallPhase = PHASE_IDLE;
        break;
    }

    if (fabsf(filteredData.accMag - 1.0f) < STILL_ACC_THRESHOLD &&
        filteredData.gyroMag < STILL_GYRO_THRESHOLD) {
            if (now - stillStart >= STILL_DURATION_MS)
{
    float postureChange =
        fabsf(preFallAx - filteredData.accX) +
        fabsf(preFallAy - filteredData.accY) +
        fabsf(preFallAz - filteredData.accZ);

    mqttDebugf("[POSTURE] change=%.2f",
               postureChange);

    if(postureChange > 0.8f)
    {
        mqttDebug("[PHA3] BAT DONG DU THOI GIAN");

        fallCount++;

        mqttDebugf("[FALL] TE NGA PHAT HIEN (lan #%d)",
                   fallCount);

        sendDataToCloud("Phat hien te nga khan cap");

        fallPhase = PHASE_COOLDOWN;
        cooldownStart = now;
    }
    else
    {
        mqttDebugf(
          "[FALSE ALARM] posture change=%.2f",
          postureChange);

        fallPhase = PHASE_IDLE;
    }
}
        
    }
    else {
        stillStart = now;
    }

    break;
}

    case PHASE_COOLDOWN:
      if (now - cooldownStart >= FALL_COOLDOWN_MS) {
        fallPhase = PHASE_IDLE;
        //Serial.println("[System] San sang giam sat lai...");
      }
      break;

    default: fallPhase = PHASE_IDLE; break;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(1000);
  Serial.println("\n=== KHOI DONG HE THONG ===");

  healthMutex = xSemaphoreCreateMutex();
  i2cMutex    = xSemaphoreCreateMutex();
  if (!healthMutex || !i2cMutex) {
    Serial.println("[FATAL] Khong tao duoc mutex!");
    while (1) delay(1000);
  }

  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);

  mpu.initialize();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);
  if (mpu.testConnection()) {
    Serial.println("[MPU6050] OK");
  } else {
    Serial.println("[MPU6050] THAT BAI!");
    while (1) delay(1000);
  }
  for (int i = 0; i < 10; i++) { readSensor(); delay(10); }
  Serial.printf("[MPU6050] Khoi tao: acc=%.2fg gyro=%.1f\n",
                filteredData.accMag, filteredData.gyroMag);

  lm75Addr = 0x00;
  Serial.print("[LM75] Quet 0x48..0x4F: ");
  for (uint8_t a = 0x48; a <= 0x4F; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("0x%02X ", a);
      if (lm75Addr == 0x00) lm75Addr = a;
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
  } else {
    Serial.println("[LM75] CANH BAO: Khong thay trong dai 0x48..0x4F.");
  }

  if (particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    particleSensor.setup(LED_BRIGHTNESS, SAMPLE_AVERAGE, LED_MODE,
                         SAMPLE_RATE, PULSE_WIDTH, ADC_RANGE);
    particleSensor.setPulseAmplitudeRed(0x18);
    particleSensor.setPulseAmplitudeIR(0x18);
    particleSensor.setPulseAmplitudeGreen(0);
    max30102Ready = true;
    Serial.println("[MAX30102] OK");
  } else {
    Serial.println("[MAX30102] CANH BAO: Khong tim thay cam bien.");
  }

#if ENABLE_WIFI
  espClient.setInsecure();
  if (connectWiFi()) {
    yieldDelay(1000);
    connectMQTT();
  } else {
    Serial.println("[Setup] Khong co WiFi - chay offline");
  }
#else
  Serial.println("[Setup] WiFi/MQTT DANG TAT (ENABLE_WIFI=0)");
#endif

  if (max30102Ready) {
    xTaskCreatePinnedToCore(TaskMAX30102, "MAX30102",
                            MAX30102_STACK_SIZE, NULL, 1, NULL, 0);
  }

  Serial.println("\n[System] Bat dau giam sat...\n");
}

void loop() {
  unsigned long now = millis();

#if ENABLE_WIFI
  handleMQTT();

  static unsigned long lastWiFiCheck = 0;
  if (now - lastWiFiCheck > WIFI_CHECK_INTERVAL) {
    lastWiFiCheck = now;
    handleWiFi();
  }
#endif

  readSensor();
  handleFallDetection();
  handleLM75();

  static unsigned long lastNormalSend = 0;
  if (fallPhase == PHASE_IDLE && now - lastNormalSend > 1000) {
    lastNormalSend = now;
    sendDataToCloud("Binh thuong");
  }

  static unsigned long lastHealthPrint = 0;
  if (now - lastHealthPrint > 3000) {
    lastHealthPrint = now;
    int hr = -1, sp = -1; float temp = -1.0f;
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
