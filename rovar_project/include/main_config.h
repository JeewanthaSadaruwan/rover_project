/*
 * Main Robot Configuration
 * Pin definitions and settings specific to main robot
 */

#ifndef MAIN_CONFIG_H
#define MAIN_CONFIG_H

// ========== WATER SENSOR PINS ==========
#define WATER_TEMP_PIN      A0  // 36
#define WATER_PH_PIN        A3  // 39
#define WATER_TURBIDITY_PIN A6  // 34
#define WATER_DO_PIN        A7  // 35

// ========== SOIL SENSOR PINS ==========
#define SOIL_MOISTURE_PIN   A4  // 32
#define SOIL_TEMP_PIN       A5  // 33
#define SOIL_PH_PIN         25
#define SOIL_NPK_RX         16
#define SOIL_NPK_TX         17

// ========== AIR SENSOR PINS ==========
#define AIR_DHT_PIN         4   // DHT22 data pin
#define AIR_BMP_SDA         21  // I2C SDA for BMP280
#define AIR_BMP_SCL         22  // I2C SCL for BMP280

// ========== GPS PINS ==========
#define GPS_RX              13
#define GPS_TX              15

// ========== MOTOR PINS ==========
#define MOTOR_LEFT_PWM      12
#define MOTOR_LEFT_IN1      14
#define MOTOR_LEFT_IN2      27
#define MOTOR_RIGHT_PWM     26
#define MOTOR_RIGHT_IN1     25
#define MOTOR_RIGHT_IN2     33

// ========== ULTRASONIC SENSOR PINS ==========
#define ULTRASONIC_FRONT_TRIG   5
#define ULTRASONIC_FRONT_ECHO   18
#define ULTRASONIC_LEFT_TRIG    19
#define ULTRASONIC_LEFT_ECHO    21
#define ULTRASONIC_RIGHT_TRIG   22
#define ULTRASONIC_RIGHT_ECHO   23

// ========== BATTERY MONITOR ==========
#define BATTERY_PIN         35  // ADC pin

// ========== LED INDICATOR ==========
#define STATUS_LED          2

// ========== MOTOR SETTINGS ==========
#define MOTOR_SPEED_NORMAL  200
#define MOTOR_SPEED_SLOW    100
#define OBSTACLE_THRESHOLD  30  // cm

// ========== BATTERY SETTINGS ==========
#define BATTERY_MAX_VOLTAGE 12.6
#define BATTERY_MIN_VOLTAGE 9.0
#define LOW_BATTERY_PERCENT 20.0

// ========== HOME POSITION ==========
#define HOME_LATITUDE       0.0
#define HOME_LONGITUDE      0.0
#define HOME_RADIUS         5.0  // meters

// ========== WiFi CREDENTIALS ==========
#define WIFI_SSID          "your_wifi_ssid"
#define WIFI_PASSWORD      "your_wifi_password"

// ========== MQTT SETTINGS ==========
#define MQTT_SERVER        "mqtt.example.com"
#define MQTT_PORT          1883
#define MQTT_USER          "rover_user"
#define MQTT_PASSWORD      "rover_password"
#define MQTT_TOPIC_DATA    "rover/main/data"
#define MQTT_TOPIC_STATUS  "rover/main/status"

#endif // MAIN_CONFIG_H
