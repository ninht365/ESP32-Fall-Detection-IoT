# ESP32-Fall-Detection-IoT
Wearable IoT Fall Detection &amp; Health Monitoring system using ESP32-S3, MAX30102, and MPU6050.
## 🌟 Key Features
*   **Real-time Fall Detection:** Utilizes MPU6050 kinematics data and filtering algorithms to accurately identify falls.
*   **Vital Signs Monitoring:** Measures SpO2 & Heart Rate (MAX30102) and Body Temperature (LM75).
*   **Low Latency Alerts:** Publishes critical alerts instantly via **MQTT (HiveMQ)**.
*   **Cloud Storage:** Logs historical data and events securely on **Firebase** for remote access.

## 🛠️ Hardware Components
*   Microcontroller: **ESP32-S3 Supermini**
*   Accelerometer & Gyroscope: **MPU6050** (I2C)
*   Pulse Oximeter & Heart-Rate Sensor: **MAX30102** (I2C)
*   Temperature Sensor: **LM75** (I2C)

## 💻 Software & Technologies
*   **Framework/IDE:** PlatformIO / Arduino C++
*   **Communication Protocols:** I2C, WiFi, MQTT
*   **Cloud/Backend:** HiveMQ Broker, Firebase

## 🎥 Video Demo
[![Watch the video](https://drive.google.com/file/d/1pKgG8LQ315uCQ7WLP3eOXYklbpnuRJRS/view?usp=sharing)
