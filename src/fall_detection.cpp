/**
 * @file    fall_detection.cpp
 * @brief   Triển khai state machine phát hiện té ngã 5 pha.
 *
 * Sơ đồ chuyển trạng thái:
 *
 *  IDLE ──(accMag < FF_THRESH)──► FREE_FALL
 *         ◄─(dur < MIN_MS)────────────┤
 *                                     │(dur ≥ MIN_MS)
 *                                     ▼
 *                               IMPACT ──(timeout)──► IDLE
 *                                     │(accMag > IMP_THRESH)
 *                                     ▼
 *                          WAITING_STILL ──(timeout)──► IDLE
 *                                     │(still ≥ STILL_DURATION_MS)
 *                                     ▼
 *                   [TÉ NGÃ XÁC NHẬN] → sendDataToCloud()
 *                                     ▼
 *                               COOLDOWN ──(elapsed)──► IDLE
 */

#include "fall_detection.h"
#include "shared_state.h"
#include "config.h"
#include "mqtt_manager.h"

#include <Arduino.h>
#include <math.h>

// ─────────────────────────────────────────────
// Trạng thái nội bộ
// ─────────────────────────────────────────────
static FallPhase     fallPhase     = PHASE_IDLE;
static unsigned long phaseStart    = 0;  ///< Thời điểm bắt đầu pha hiện tại
static unsigned long stillStart    = 0;  ///< Thời điểm bắt đầu đứng yên
static unsigned long freeFallStart = 0;  ///< Thời điểm bắt đầu rơi tự do
static unsigned long cooldownStart = 0;  ///< Thời điểm bắt đầu cooldown
static int           fallCount     = 0;  ///< Tổng số lần phát hiện té ngã

// ─────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────

FallPhase getFallPhase() { return fallPhase; }

void handleFallDetection() {
  unsigned long now = millis();

  switch (fallPhase) {

    // ── Pha 0: Bình thường ────────────────────────────────────────────
    case PHASE_IDLE:
      if (sensorData.accMag < FREE_FALL_THRESHOLD) {
        fallPhase = PHASE_FREE_FALL;
        freeFallStart = now;
        Serial.printf("[Pha 1] Bat dau ROI TU DO (acc=%.2fg)\n", sensorData.accMag);
      }
      break;

    // ── Pha 1: Rơi tự do ─────────────────────────────────────────────
    case PHASE_FREE_FALL:
      if (sensorData.accMag >= FREE_FALL_THRESHOLD) {
        // Gia tốc đã phục hồi → kiểm tra thời gian rơi
        unsigned long dur = now - freeFallStart;
        if (dur < FREE_FALL_MIN_MS) {
          fallPhase = PHASE_IDLE;
          Serial.println("[Reset] Roi tu do qua ngan (nhieu)");
        } else {
          fallPhase = PHASE_IMPACT;
          phaseStart = now;
          Serial.printf("[Pha 1] Xac nhan ROI TU DO (%lums)\n", dur);
        }
        break;
      }
      // Rơi quá lâu → bất thường, reset
      if (now - freeFallStart > 1000UL) {
        fallPhase = PHASE_IDLE;
        Serial.println("[Reset] Roi tu do qua lau (timeout)");
      }
      break;

    // ── Pha 2: Chờ va chạm ───────────────────────────────────────────
    case PHASE_IMPACT:
      if (now - phaseStart > IMPACT_TIMEOUT_MS) {
        fallPhase = PHASE_IDLE;
        Serial.println("[Reset] Khong co va cham (timeout)");
        break;
      }
      if (sensorData.accMag > IMPACT_THRESHOLD) {
        Serial.printf("[Pha 2] VA CHAM phat hien (acc=%.2fg)\n", sensorData.accMag);
        fallPhase  = PHASE_WAITING_STILL;
        phaseStart = now;
        stillStart = now;
      }
      break;

    // ── Pha 3: Chờ đứng yên ──────────────────────────────────────────
    case PHASE_WAITING_STILL:
      if (now - phaseStart > STILL_TIMEOUT_MS) {
        fallPhase = PHASE_IDLE;
        Serial.println("[Reset] Van di chuyen sau va cham (timeout)");
        break;
      }
      if (fabsf(filteredData.accMag - 1.0f) < STILL_ACC_THRESHOLD &&
          filteredData.gyroMag < STILL_GYRO_THRESHOLD) {
        // Đang đứng yên — kiểm tra đủ thời gian chưa
        if (now - stillStart >= STILL_DURATION_MS) {
          fallCount++;
          Serial.println("=================================");
          Serial.printf(">>>    TE NGA PHAT HIEN! (lan #%d)    <<<\n", fallCount);
          Serial.println("=================================");
          sendDataToCloud("Phat hien te nga khan cap");
          fallPhase     = PHASE_COOLDOWN;
          cooldownStart = now;
        }
      } else {
        // Vẫn đang di chuyển → reset bộ đếm đứng yên
        stillStart = now;
      }
      break;

    // ── Pha 4: Cooldown ───────────────────────────────────────────────
    case PHASE_COOLDOWN:
      if (now - cooldownStart >= FALL_COOLDOWN_MS) {
        fallPhase = PHASE_IDLE;
        Serial.println("[System] San sang giam sat lai...");
      }
      break;

    default:
      fallPhase = PHASE_IDLE;
      break;
  }
}
