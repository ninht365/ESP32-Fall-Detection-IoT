/**
 * @file    lm75_module.h
 * @brief   Interface module LM75 — cảm biến nhiệt độ qua I2C.
 *
 * Cung cấp hai hàm công khai:
 *  - lm75Init():   quét bus I2C để tìm LM75, gọi 1 lần trong setup().
 *  - handleLM75(): đọc nhiệt độ định kỳ, gọi trong loop().
 */

#pragma once

/**
 * @brief Quét địa chỉ 0x48–0x4F để tìm LM75 và đọc nhiệt độ ban đầu.
 * @return true nếu tìm thấy thiết bị, false nếu không.
 */
bool lm75Init();

/**
 * @brief Đọc nhiệt độ từ LM75 theo chu kỳ LM75_READ_INTERVAL_MS.
 *
 * Cập nhật ::healthData.bodyTemp qua healthMutex.
 * Không làm gì nếu lm75Ready == false.
 */
void handleLM75();
