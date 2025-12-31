/*
 * MAIN ROBOT FIRMWARE
 * Environmental monitoring rover with sensors and navigation
 * 
 * Initialization Order:
 * 1. ESP-NOW receiver (remote control)
 * 2. LoRa communication
 * 3. Sensors initialization
 * 4. Start transmitting sensor data
 */

#include <Arduino.h>
#include <Arduino.h>
#include <LoRa.h>
#include "mainrobot_lora_init.h"
#include "protocol.h"
#include "app_config.h"
#include "main_config.h"
#include "motor_control.h"
#include "wifi_mqtt_handler.h"
#include "sensor_init.h"
#include "esp_now_receiver.h"

// Forward declarations
void handleLoRaPacket(LoRaPacket* packet);
bool sendLoRaPacket(uint8_t type, const char* data);
bool receiveLoRaPacket(LoRaPacket* packet);
void readIMUData(float& accelX, float& accelY, float& accelZ, float& gyroX, float& gyroY, float& gyroZ);

// LoRa communication
uint16_t loraPacketCounter = 0;
uint16_t loraCounter = 0;

// Timing variables
unsigned long lastSensorTransmit = 0;
#define SENSOR_TRANSMIT_INTERVAL 5000  // Transmit every 5 seconds

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n====================================");
  Serial.println("   MAIN ROBOT - Starting...");
  Serial.println("====================================\n");
  
  // Status LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);
  
  // Initialize motors
  initMotors();
  
  // ===== STEP 1: ESP-NOW RECEIVER =====
  Serial.println("[STEP 1] Initializing Remote Control (ESP-NOW)...");
  if (!initESPNowReceiver()) {
    Serial.println("[ERROR] ESP-NOW initialization failed!");
  }
  delay(500);
  
  // ===== STEP 2: LoRa INITIALIZATION =====
  Serial.println("[STEP 2] Initializing LoRa Communication...");
  if (!initMainRobotLoRa()) {
    Serial.println("[ERROR] LoRa initialization failed!");
    while (1) {
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
      delay(200);
    }
  }
  mainRobotLoRaStartReceive();
  Serial.println("[LoRa] Establishment done - Ready to send data");
  delay(500);
  
  // ===== STEP 3: SENSORS INITIALIZATION =====
  Serial.println("[STEP 3] Initializing Sensors...");
  if (!initSensors()) {
    Serial.println("[ERROR] Sensor initialization failed!");
  }
  printSensorStatus();
  delay(500);
  
  // All systems ready
  Serial.println("====================================");
  Serial.println("[READY] All systems initialized!");
  Serial.println("====================================\n");
  
  digitalWrite(STATUS_LED, LOW);
}

void loop() {
  unsigned long currentTime = millis();
  
  // Check if remote is connected
  static bool remoteStatusPrinted = false;
  if (isRemoteConnected() && !remoteStatusPrinted) {
    Serial.println("\n>>> Remote control is connected - Now you can control the robot <<<\n");
    remoteStatusPrinted = true;
  } else if (!isRemoteConnected() && remoteStatusPrinted) {
    Serial.println("\n>>> Remote control disconnected <<<\n");
    stopMotors();
    remoteStatusPrinted = false;
  }
  
  // ===== MOTOR CONTROL FROM JOYSTICK =====
  // Check if we have joystick data and remote is connected
  if (isRemoteConnected()) {
    JoystickData joyData = {0, 0, false};
    if (espNowReadJoystick(joyData)) {
      // Process joystick input and control motors
      handleJoystickInput(joyData);
    }
  } else {
    // No remote connection - ensure motors are stopped
    stopMotors();
  }
  
  // Handle incoming LoRa packets
  LoRaPacket packet;
  if (receiveLoRaPacket(&packet)) {
    handleLoRaPacket(&packet);
  }
  
  // Transmit sensor data periodically
  if (currentTime - lastSensorTransmit >= SENSOR_TRANSMIT_INTERVAL) {
    lastSensorTransmit = currentTime;
    
    // Read sensors
    SensorReadings readings = readAllSensors();
    
    // Read IMU data
    float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
    readIMUData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
    
    // Print sensor values to Serial
    Serial.print("[SENSORS] Temp: ");
    Serial.print(readings.temperature, 1);
    Serial.print("°C | Humidity: ");
    Serial.print(readings.humidity, 1);
    Serial.print("% | Gas: ");
    Serial.print(readings.airQuality, 1);
    Serial.print("% | Soil: ");
    Serial.print(readings.soilMoisture, 1);
    Serial.print("% | pH: ");
    Serial.print(readings.soilPH, 1);
    Serial.print(" | GPS: ");
    Serial.print(readings.latitude, 6);
    Serial.print(",");
    Serial.println(readings.longitude, 6);
    
    Serial.print("[IMU] Accel: ");
    Serial.print(accelX, 2);
    Serial.print(",");
    Serial.print(accelY, 2);
    Serial.print(",");
    Serial.print(accelZ, 2);
    Serial.print(" | Gyro: ");
    Serial.print(gyroX, 2);
    Serial.print(",");
    Serial.print(gyroY, 2);
    Serial.print(",");
    Serial.println(gyroZ, 2);
    
    // Transmit via LoRa (TEMP, HUM, GAS, SOIL, PH, GPS, and IMU)
    char loraPayload[256];
    snprintf(loraPayload, sizeof(loraPayload),
      "TEMP:%.1f,HUM:%.1f,GAS:%.1f,SOIL:%.1f,PH:%.1f,LAT:%.6f,LNG:%.6f,AX:%.2f,AY:%.2f,AZ:%.2f,GX:%.2f,GY:%.2f,GZ:%.2f",
      readings.temperature, readings.humidity, readings.airQuality, readings.soilMoisture, readings.soilPH,
      readings.latitude, readings.longitude,
      accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
    
    Serial.print("[LoRa TX] Transmitting: ");
    Serial.println(loraPayload);
    
    bool txSuccess = sendLoRaPacket(PKT_DATA, loraPayload);
    
    if (txSuccess) {
      Serial.println("[LoRa TX] SUCCESS");
    } else {
      Serial.println("[LoRa TX] FAILED");
    }
    
    // Blink LED on transmission
    digitalWrite(STATUS_LED, HIGH);
    delay(100);
    digitalWrite(STATUS_LED, LOW);
  }
  
  delay(20);
}

void handleLoRaPacket(LoRaPacket* packet) {
  switch (packet->type) {
    case PKT_STATUS:
      Serial.print("[LoRa RX] Sub-robot status: ");
      Serial.println(packet->data);
      break;
      
    case PKT_DATA:
      Serial.print("[LoRa RX] Sub-robot data: ");
      Serial.println(packet->data);
      break;
      
    default:
      Serial.print("[LoRa RX] Packet type 0x");
      Serial.print(packet->type, HEX);
      Serial.print(": ");
      Serial.println(packet->data);
  }
}

bool sendLoRaPacket(uint8_t type, const char* data) {
  uint8_t buffer[4 + MAX_DATA_SIZE];
  size_t dataLen = strlen(data);
  if (dataLen > MAX_DATA_SIZE) {
    dataLen = MAX_DATA_SIZE;
  }

  buffer[0] = type;
  buffer[1] = (uint8_t)(loraPacketCounter >> 8);
  buffer[2] = (uint8_t)(loraPacketCounter & 0xFF);
  buffer[3] = (uint8_t)dataLen;
  memcpy(&buffer[4], data, dataLen);

  Serial.print("[LoRa Internal] Sending packet type 0x");
  Serial.print(type, HEX);
  Serial.print(" len ");
  Serial.println(dataLen);
  
  int state = mainRobotLoRaTransmitPacket(buffer, 4 + dataLen);
  mainRobotLoRaStartReceive();
  
  if (!state) {
    Serial.print("[LoRa Internal] TX error");
    return false;
  }

  loraPacketCounter++;
  return true;
}

bool receiveLoRaPacket(LoRaPacket* packet) {
  if (!mainRobotLoRaAvailable()) {
    return false;
  }

  uint8_t buffer[MAX_PACKET_SIZE];
  int state = mainRobotLoRaRead(buffer, sizeof(buffer));
  mainRobotLoRaStartReceive();

  // LoRa library doesn't use state codes like RadioLib

  uint8_t dataLen = buffer[3];
  if (dataLen > MAX_DATA_SIZE) {
    Serial.println("[LoRa RX] Invalid data length");
    return false;
  }

  packet->type = buffer[0];
  packet->counter = (uint16_t)((buffer[1] << 8) | buffer[2]);
  packet->dataLen = dataLen;
  memcpy(packet->data, &buffer[4], dataLen);
  packet->data[dataLen] = '\0';
  return true;
}