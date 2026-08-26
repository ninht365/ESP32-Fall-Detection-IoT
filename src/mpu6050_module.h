/**
 * @file    mpu6050_module.h
 * @brief   Interface module MPU6050 — cảm biến gia tốc và con quay hồi chuyển.
 *
 * Cung cấp hai hàm công khai:
 *  - mpu6050Init(): khởi tạo phần cứng, gọi 1 lần trong setup().
 *  - readSensor():  đọc và lọc dữ liệu, gọi định kỳ trong loop().
 */

#pragma once

/**
 * @brief Khởi tạo MPU6050 (range, DLPF, warm-up 10 mẫu).
 * @return true nếu kết nối thành công, false nếu không tìm thấy thiết bị.
 */
bool mpu6050Init();

/**
 * @brief Đọc một mẫu từ MPU6050, quy đổi đơn vị và áp lọc EMA.
 *
 * Kết quả được ghi vào ::sensorData và ::filteredData trong shared_state.h.
 * Hàm này sử dụng i2cMutex nên thread-safe với các task khác trên I2C.
 */
void readSensor();
