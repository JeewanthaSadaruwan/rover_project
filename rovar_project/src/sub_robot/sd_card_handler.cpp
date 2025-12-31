/*
 * SD Card Handler Implementation
 * Logs sensor data to CSV files with timestamps
 */

#include "sd_card_handler.h"
#include <SPI.h>
#include <SD.h>

// SD Card SPI instance
SPIClass sdSPI(HSPI);

// Status variables
bool sdCardInitialized = false;
String currentLogFile = "";
unsigned long sessionStartTime = 0;
int dataCount = 0;

bool initSDCard() {
  Serial.println("\n[SD Card] Initializing...");
  Serial.print("[SD Card] Pins: CS=");
  Serial.print(SD_CS);
  Serial.print(" SCK=");
  Serial.print(SD_SCK);
  Serial.print(" MISO=");
  Serial.print(SD_MISO);
  Serial.print(" MOSI=");
  Serial.println(SD_MOSI);
  
  // Initialize SPI for SD card
  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  delay(100);
  
  // Initialize SD card
  if (!SD.begin(SD_CS, sdSPI)) {
    Serial.println("[SD Card] FAILED - Check card and connections!");
    sdCardInitialized = false;
    return false;
  }
  
  // Get card info
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.print("[SD Card] Card Size: ");
  Serial.print(cardSize);
  Serial.println(" MB");
  
  uint64_t usedBytes = SD.usedBytes() / (1024 * 1024);
  Serial.print("[SD Card] Used Space: ");
  Serial.print(usedBytes);
  Serial.println(" MB");
  
  // Create filename with session timestamp
  sessionStartTime = millis();
  currentLogFile = "/rover_data_" + String(sessionStartTime / 1000) + ".csv";
  
  Serial.print("[SD Card] Log file: ");
  Serial.println(currentLogFile);
  
  // Create CSV file with header
  File file = SD.open(currentLogFile.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("[SD Card] Failed to create log file!");
    sdCardInitialized = false;
    return false;
  }
  
  // Write CSV header
  file.println("Timestamp,Temperature,Humidity,Gas,SoilMoisture,SoilPH,Latitude,Longitude,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ");
  file.close();
  
  Serial.println("[SD Card] Ready! CSV file created with header");
  sdCardInitialized = true;
  dataCount = 0;
  
  return true;
}

bool storeLoRaData(const char* sensorData) {
  if (!sdCardInitialized) {
    Serial.println("[SD Card] Not initialized - skipping storage");
    return false;
  }
  
  // Parse the sensor data
  // Format: TEMP:27.4,HUM:78.7,GAS:0.0,SOIL:45.6,PH:6.2,LAT:0.000000,LNG:0.000000,AX:-2.04,AY:-0.44,AZ:9.92,GX:-0.01,GY:0.03,GZ:-0.00
  float temp = 0, hum = 0, gas = 0, soil = 0, ph = 0, lat = 0, lng = 0;
  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  
  // Parse the data using sscanf
  int parsed = sscanf(sensorData, 
    "TEMP:%f,HUM:%f,GAS:%f,SOIL:%f,PH:%f,LAT:%f,LNG:%f,AX:%f,AY:%f,AZ:%f,GX:%f,GY:%f,GZ:%f",
    &temp, &hum, &gas, &soil, &ph, &lat, &lng, &ax, &ay, &az, &gx, &gy, &gz);
  
  if (parsed < 13) {
    Serial.print("[SD Card] Parse error - only got ");
    Serial.print(parsed);
    Serial.println(" fields");
    return false;
  }
  
  // Get current timestamp (milliseconds since boot)
  unsigned long timestamp = millis();
  
  // Open file in append mode
  File file = SD.open(currentLogFile.c_str(), FILE_APPEND);
  if (!file) {
    Serial.println("[SD Card] Failed to open file for writing!");
    return false;
  }
  
  // Write CSV data
  file.print(timestamp);
  file.print(",");
  file.print(temp, 2);
  file.print(",");
  file.print(hum, 2);
  file.print(",");
  file.print(gas, 2);
  file.print(",");
  file.print(soil, 2);
  file.print(",");
  file.print(ph, 2);
  file.print(",");
  file.print(lat, 6);
  file.print(",");
  file.print(lng, 6);
  file.print(",");
  file.print(ax, 2);
  file.print(",");
  file.print(ay, 2);
  file.print(",");
  file.print(az, 2);
  file.print(",");
  file.print(gx, 3);
  file.print(",");
  file.print(gy, 3);
  file.print(",");
  file.println(gz, 3);
  
  file.close();
  
  dataCount++;
  
  Serial.print("[SD Card] ✓ Data saved! (Count: ");
  Serial.print(dataCount);
  Serial.println(")");
  
  return true;
}

void getSDCardStatus(char* statusBuffer, size_t bufferSize) {
  if (!sdCardInitialized) {
    snprintf(statusBuffer, bufferSize, "SD:DISABLED,LOGS:0");
    return;
  }
  
  snprintf(statusBuffer, bufferSize, "SD:ACTIVE,LOGS:%d,FILE:%s", 
           dataCount, currentLogFile.c_str());
}

String getCurrentLogFileName() {
  return currentLogFile;
}
