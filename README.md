# iot-cloud-step-analysis

This repository contains the code and documentation for an Internet of Things (IoT) project focused on real-time step and jump detection using a microcontroller and cloud-based analysis. The project explores the latency trade-offs between on-device processing and remote cloud computation for motion recognition.

## Overview

The project involves the following key components:

1.  **Embedded System (Microcontroller):**
    * Utilizes an ESP32-based LilyGO T-Display microcontroller to interface with a Gyroscope/Accelerometer (LSM6DSO) sensor via the I2C protocol.
    * Implements real-time data acquisition and initial processing of sensor data.
    * Performs local motion detection with an ultra-low latency of approximately 60-100 microseconds.
    * Establishes wireless connectivity (Bluetooth Low Energy - BLE and Wi-Fi).
    * Transmits timestamped sensor data to a cloud-based backend via HTTP requests to a RESTful API.

2.  **Cloud Backend:**
    * Deployed on an AWS EC2 instance.
    * Built using C++ and the Mongoose embedded web server library.
    * Exposes a RESTful API endpoint to receive sensor data from the microcontroller.
    * Implements a robust peak detection algorithm in C++ to identify steps and jumps from the received gyroscope data.
    * Calculates and returns the processing latency, demonstrating a microcontroller-to-cloud response time of approximately 450-500 milliseconds.

3.  **Latency Analysis:**
    * The project includes a detailed analysis comparing the latency of on-device motion detection (60-100µs) versus cloud-based analysis (450-500ms).
    * This comparison provides insights into the trade-offs between immediate local responsiveness and more complex remote processing for IoT applications.

## Key Technologies Used

* **Languages:** C++, Arduino
* **Microcontroller:** ESP32 (LilyGO T-Display)
* **Sensors:** LSM6DSO Gyroscope/Accelerometer
* **Communication Protocols:** I2C, Bluetooth Low Energy (BLE), Wi-Fi, HTTP
* **Web Server/API:** Mongoose (C++)
* **Cloud Platform:** Amazon Web Services (AWS EC2)
* **API Interaction:** RESTful APIs
* **Data Format:** JSON (for API responses)
* **Development Framework:** ESP-IDF (Espressif IoT Development Framework)
* **Version Control:** Git
