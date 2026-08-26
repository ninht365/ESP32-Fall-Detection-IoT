/**
 * @file    max30102_module.h
 * @brief   Interface module MAX30102 — đo nhịp tim (HR) và SpO2.
 *
 * Cung cấp:
 *  - max30102Init():  khởi tạo phần cứng, gọi 1 lần trong setup().
 *  - TaskMAX30102():  FreeRTOS task chạy liên tục trên core 0.
 */

#pragma once

/**
 * @brief Khởi tạo MAX30102 với thông số LED và lấy mẫu từ config.h.
 * @return true nếu tìm thấy và cấu hình thành công, false nếu không.
 */
bool max30102Init();

/**
 * @brief FreeRTOS task — thu thập mẫu IR/Red, tính HR và SpO2 liên tục.
 *
 * Task này nên được tạo bằng xTaskCreatePinnedToCore() trên core 0
 * (tránh xung đột với loop() trên core 1).
 *
 * @param pvParameters Không dùng (truyền NULL).
 */
void TaskMAX30102(void *pvParameters);
