/*
 * Firebase Handler for Base Station
 * Handles WiFi connection and Firebase data upload
 */

#ifndef FIREBASE_HANDLER_H
#define FIREBASE_HANDLER_H

#include <Arduino.h>

// Sensor data structure for Firebase
struct FirebaseSensorData {
  float temperature;
  float humidity;
  float gasLevel;
  float soilMoisture;
  float soilPH;
  float latitude;
  float longitude;
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  unsigned long timestamp;
};

// Function declarations
bool initWiFi();
bool initFirebase();
bool isWiFiConnected();
bool isFirebaseReady();
bool sendSensorDataToFirebase(FirebaseSensorData& data);
bool sendRawDataToFirebase(const char* sensorData);
void checkWiFiConnection();

#endif // FIREBASE_HANDLER_H
