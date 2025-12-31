#ifndef SENSOR_INIT_H
#define SENSOR_INIT_H

#include <Arduino.h>

// Sensor data structure
struct SensorReadings {
  float temperature;      // DHT11 temperature (°C)
  float humidity;         // DHT11 humidity (%)
  float airQuality;       // MQ2 gas level (%)
  float soilMoisture;     // Soil moisture (%) - DUMMY DATA FOR NOW
  float soilPH;           // Soil pH (5-7) - DUMMY DATA FOR NOW
  float latitude;         // GPS latitude
  float longitude;        // GPS longitude
  float batteryVoltage;   // Battery voltage (V)
};

// Function declarations
bool initSensors();
bool checkSensorsReady();
SensorReadings readAllSensors();
void printSensorStatus();

#endif // SENSOR_INIT_H
