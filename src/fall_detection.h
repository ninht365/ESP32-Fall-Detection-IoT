/**
 * @file    fall_detection.h
 * @brief   Interface module phát hiện té ngã — state machine 5 pha.
 *
 * Pha chuyển tiếp:
 *   IDLE → FREE_FALL → IMPACT → WAITING_STILL → COOLDOWN → IDLE
 *
 * Cung cấp:
 *  - handleFallDetection(): gọi mỗi vòng loop() sau readSensor().
 *  - getFallPhase():        truy vấn trạng thái hiện tại từ bên ngoài.
 */

#pragma once

#include <stdint.h>

/**
 * @brief Các pha của state machine phát hiện té ngã.
 */
enum FallPhase : uint8_t {
  PHASE_IDLE,          ///< Bình thường — đang theo dõi
  PHASE_FREE_FALL,     ///< Phát hiện rơi tự do (accMag < FREE_FALL_THRESHOLD)
  PHASE_IMPACT,        ///< Chờ va chạm (accMag > IMPACT_THRESHOLD)
  PHASE_WAITING_STILL, ///< Chờ đứng yên sau va chạm
  PHASE_COOLDOWN       ///< Cooldown sau khi phát hiện té ngã
};

/**
 * @brief Chạy một bước của state machine phát hiện té ngã.
 *
 * Đọc ::sensorData và ::filteredData (không cần mutex vì chỉ loop() ghi).
 * Gọi sendDataToCloud() khi xác nhận té ngã.
 */
void handleFallDetection();

/**
 * @brief Trả về pha hiện tại của state machine.
 * @return FallPhase hiện tại.
 */
FallPhase getFallPhase();
