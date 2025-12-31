/*
 * Firebase Handler Implementation
 * Connects to WiFi and uploads sensor data to Firebase Realtime Database
 */

#include "firebase_handler.h"
#include "firebase_config.h"
#include <WiFi.h>
#include <Firebase_ESP_Client.h>

// Firebase helpers
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Status flags
bool wifiConnected = false;
bool firebaseReady = false;
unsigned long lastWiFiCheck = 0;
unsigned long dataCount = 0;

// WiFi connection timeout
#define WIFI_TIMEOUT_MS 10000

bool initWiFi() {
  Serial.println("\n[WiFi] Connecting to WiFi...");
  Serial.print("[WiFi] SSID: ");
  Serial.println(WIFI_SSID);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  unsigned long startTime = millis();
  int dots = 0;
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    dots++;
    if (dots % 20 == 0) Serial.println();
    
    if (millis() - startTime > WIFI_TIMEOUT_MS) {
      Serial.println("\n[WiFi] Connection FAILED - Timeout!");
      Serial.println("[WiFi] Check SSID and password in firebase_config.h");
      wifiConnected = false;
      return false;
    }
  }
  
  Serial.println("\n[WiFi] Connected!");
  Serial.print("[WiFi] IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("[WiFi] Signal Strength (RSSI): ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
  
  wifiConnected = true;
  return true;
}

bool initFirebase() {
  if (!wifiConnected) {
    Serial.println("[Firebase] Cannot initialize - WiFi not connected!");
    return false;
  }
  
  Serial.println("\n[Firebase] Initializing...");
  
  // Configure Firebase
  config.api_key = FIREBASE_API_KEY;
  config.database_url = FIREBASE_DATABASE_URL;
  
  // Set up authentication
  auth.user.email = FIREBASE_USER_EMAIL;
  auth.user.password = FIREBASE_USER_PASSWORD;
  
  // Token status callback
  config.token_status_callback = tokenStatusCallback;
  
  // Initialize Firebase
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  
  // Set database read timeout
  fbdo.setResponseSize(4096);
  
  // Wait for token
  Serial.println("[Firebase] Getting auth token...");
  unsigned long startTime = millis();
  while (!Firebase.ready() && (millis() - startTime < 15000)) {
    delay(100);
  }
  
  if (Firebase.ready()) {
    Serial.println("[Firebase] Connected and authenticated!");
    firebaseReady = true;
    
    // Send initial status
    Firebase.RTDB.setString(&fbdo, String(FIREBASE_STATUS_PATH) + "/status", "online");
    Firebase.RTDB.setInt(&fbdo, String(FIREBASE_STATUS_PATH) + "/lastBoot", millis());
    
    return true;
  } else {
    Serial.println("[Firebase] Authentication FAILED!");
    Serial.println("[Firebase] Check API Key and credentials in firebase_config.h");
    firebaseReady = false;
    return false;
  }
}

bool isWiFiConnected() {
  return WiFi.status() == WL_CONNECTED;
}

bool isFirebaseReady() {
  return firebaseReady && Firebase.ready();
}

void checkWiFiConnection() {
  // Check WiFi every 30 seconds
  if (millis() - lastWiFiCheck > 30000) {
    lastWiFiCheck = millis();
    
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] Connection lost! Reconnecting...");
      wifiConnected = false;
      firebaseReady = false;
      initWiFi();
      if (wifiConnected) {
        initFirebase();
      }
    }
  }
}

bool sendSensorDataToFirebase(FirebaseSensorData& data) {
  if (!isFirebaseReady()) {
    Serial.println("[Firebase] Not ready - skipping upload");
    return false;
  }
  
  Serial.println("[Firebase] Uploading sensor data...");
  
  // Get current timestamp (seconds since epoch - approximate)
  unsigned long timestamp = data.timestamp / 1000;  // Convert ms to seconds
  
  // Determine gas status
  const char* gasStatus = (data.gasLevel < 50) ? "SAFE" : "WARNING";
  
  // ===== BUILD LATEST DATA JSON =====
  FirebaseJson latestJson;
  latestJson.set("device_id", FIREBASE_ROVER_ID);
  latestJson.set("timestamp", (int)timestamp);
  
  // GPS
  latestJson.set("gps/lat", data.latitude);
  latestJson.set("gps/lng", data.longitude);
  
  // DHT sensor
  latestJson.set("sensors/dht/temp", data.temperature);
  latestJson.set("sensors/dht/humidity", data.humidity);
  
  // Gas sensor
  latestJson.set("sensors/gas/level", data.gasLevel);
  latestJson.set("sensors/gas/status", gasStatus);
  
  // Soil sensor
  latestJson.set("sensors/soil/moisture", data.soilMoisture);
  latestJson.set("sensors/soil/ph", data.soilPH);
  
  // IMU sensor
  latestJson.set("sensors/imu/accel/x", data.accelX);
  latestJson.set("sensors/imu/accel/y", data.accelY);
  latestJson.set("sensors/imu/accel/z", data.accelZ);
  latestJson.set("sensors/imu/gyro/x", data.gyroX);
  latestJson.set("sensors/imu/gyro/y", data.gyroY);
  latestJson.set("sensors/imu/gyro/z", data.gyroZ);
  
  // Upload to "latest" - always overwritten with newest data
  if (Firebase.RTDB.setJSON(&fbdo, FIREBASE_LATEST_PATH, &latestJson)) {
    Serial.println("[Firebase] Latest data updated!");
  } else {
    Serial.print("[Firebase] Error: ");
    Serial.println(fbdo.errorReason());
    return false;
  }
  
  // ===== BUILD HISTORY ENTRY JSON =====
  dataCount++;
  FirebaseJson historyJson;
  historyJson.set("timestamp", (int)timestamp);
  
  // GPS
  historyJson.set("gps/lat", data.latitude);
  historyJson.set("gps/lng", data.longitude);
  
  // Sensors
  historyJson.set("sensors/dht/temp", data.temperature);
  historyJson.set("sensors/dht/humidity", data.humidity);
  historyJson.set("sensors/gas/level", data.gasLevel);
  historyJson.set("sensors/gas/status", gasStatus);
  historyJson.set("sensors/soil/moisture", data.soilMoisture);
  historyJson.set("sensors/soil/ph", data.soilPH);
  historyJson.set("sensors/imu/accel/x", data.accelX);
  historyJson.set("sensors/imu/accel/y", data.accelY);
  historyJson.set("sensors/imu/accel/z", data.accelZ);
  historyJson.set("sensors/imu/gyro/x", data.gyroX);
  historyJson.set("sensors/imu/gyro/y", data.gyroY);
  historyJson.set("sensors/imu/gyro/z", data.gyroZ);
  
  // Push to history with auto-generated key
  if (Firebase.RTDB.pushJSON(&fbdo, FIREBASE_HISTORY_PATH, &historyJson)) {
    Serial.print("[Firebase] Data pushed to history! (Count: ");
    Serial.print(dataCount);
    Serial.println(")");
  }
  
  // ===== UPDATE STATUS =====
  FirebaseJson statusJson;
  statusJson.set("is_online", true);
  statusJson.set("last_seen", (int)timestamp);
  statusJson.set("battery", 85);  // Placeholder - add battery sensor later
  Firebase.RTDB.setJSON(&fbdo, FIREBASE_STATUS_PATH, &statusJson);
  
  return true;
}

bool sendRawDataToFirebase(const char* sensorData) {
  // Parse the raw sensor data string
  // Format: TEMP:27.4,HUM:78.7,GAS:0.0,SOIL:45.6,PH:6.2,LAT:0.000000,LNG:0.000000,AX:-2.04,AY:-0.44,AZ:9.92,GX:-0.01,GY:0.03,GZ:-0.00
  
  FirebaseSensorData data = {0};
  
  int parsed = sscanf(sensorData,
    "TEMP:%f,HUM:%f,GAS:%f,SOIL:%f,PH:%f,LAT:%f,LNG:%f,AX:%f,AY:%f,AZ:%f,GX:%f,GY:%f,GZ:%f",
    &data.temperature, &data.humidity, &data.gasLevel, &data.soilMoisture, &data.soilPH,
    &data.latitude, &data.longitude,
    &data.accelX, &data.accelY, &data.accelZ,
    &data.gyroX, &data.gyroY, &data.gyroZ);
  
  if (parsed < 13) {
    Serial.print("[Firebase] Parse error - only got ");
    Serial.print(parsed);
    Serial.println(" fields");
    return false;
  }
  
  data.timestamp = millis();
  
  return sendSensorDataToFirebase(data);
}
