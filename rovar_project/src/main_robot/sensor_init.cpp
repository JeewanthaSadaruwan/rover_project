#include <Arduino.h>
#include "sensor_init.h"
#include <DHT.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// DHT Sensor
#define DHT_PIN 27
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);

// MQ Gas Sensor (A0 connected to GPIO 34)
#define MQ_GAS_PIN 34

// GPS Module
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD 9600
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

// IMU Sensor (Adafruit MPU6050)
#define IMU_SDA 21
#define IMU_SCL 22
Adafruit_MPU6050 mpu;

// Sensor status flags
bool tempSensorOK = false;
bool humiditySensorOK = false;
bool gasSensorOK = false;
bool gpsSensorOK = false;
bool imuSensorOK = false;
bool soilSensorOK = false;  // Soil moisture sensor (not yet connected)

bool initSensors() {
  Serial.println("[Sensors] Initializing sensors...");
  Serial.print("[Sensors] DHT pin: GPIO");
  Serial.println(DHT_PIN);
  Serial.print("[Sensors] MQ Gas pin: GPIO");
  Serial.println(MQ_GAS_PIN);
  Serial.print("[Sensors] GPS RX: GPIO");
  Serial.print(GPS_RX_PIN);
  Serial.print(" TX: GPIO");
  Serial.println(GPS_TX_PIN);
  
  // Initialize DHT11
  dht.begin();
  delay(2000);  // Wait for DHT to stabilize
  
  // Try to read multiple times
  float temp = NAN;
  float humidity = NAN;
  
  for (int i = 0; i < 5; i++) {
    temp = dht.readTemperature();
    humidity = dht.readHumidity();
    
    Serial.print("[Sensors] Attempt ");
    Serial.print(i + 1);
    Serial.print(": Temp=");
    Serial.print(temp);
    Serial.print(" Humidity=");
    Serial.println(humidity);
    
    if (!isnan(temp) && !isnan(humidity) && temp > -50 && temp < 100 && humidity >= 0 && humidity <= 100) {
      Serial.println("[Sensors] DHT11 sensor: OK");
      tempSensorOK = true;
      humiditySensorOK = true;
      break;
    }
    delay(1000);
  }
  
  if (!tempSensorOK) {
    Serial.println("[Sensors] DHT11 sensor: FAILED - Check connections!");
  }
  
  // Initialize GPS Serial
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(500);
  Serial.println("[Sensors] GPS module: Initializing...");
  gpsSensorOK = true;
  
  // Initialize I2C for IMU
  Serial.print("[Sensors] IMU I2C: SDA=GPIO");
  Serial.print(IMU_SDA);
  Serial.print(" SCL=GPIO");
  Serial.println(IMU_SCL);
  
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);
  delay(100);
  
  // Initialize IMU (Adafruit MPU6050)
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("[Sensors] IMU (MPU6050): OK");
    imuSensorOK = true;
  } else {
    Serial.println("[Sensors] IMU (MPU6050): FAILED - Check I2C connection!");
    imuSensorOK = false;
  }
  
  delay(100);
  gasSensorOK = true;  // MQ2 doesn't need init, just analog read
  
  return true;
}

bool checkSensorsReady() {
  return tempSensorOK && humiditySensorOK;
}

SensorReadings readAllSensors() {
  SensorReadings readings = {0, 0, 0, 0, 0, 0, 0};
  
  // Process GPS data
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  
  // Read Temperature and Humidity from DHT11
  float temp = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  // Validate readings
  if (!isnan(temp) && temp > -50 && temp < 100) {
    readings.temperature = temp;
  } else {
    readings.temperature = 0;
  }
  
  if (!isnan(humidity) && humidity >= 0 && humidity <= 100) {
    readings.humidity = humidity;
  } else {
    readings.humidity = 0;
  }
  
  // Read MQ Gas Sensor (analog value 0-4095)
  int gasValue = analogRead(MQ_GAS_PIN);
  // Convert 12-bit ADC (0-4095) to percentage (0-100)
  readings.airQuality = (gasValue / 4095.0) * 100.0;
  
  // ===== SOIL MOISTURE (DUMMY DATA FOR NOW) =====
  // Generates pseudo-random dummy data between 30-70%
  // Replace with actual sensor reading when soil sensor is connected
  static unsigned long lastSoilUpdate = 0;
  static float dummySoilMoisture = 50.0;
  
  if (millis() - lastSoilUpdate > 1000) {  // Update every second
    lastSoilUpdate = millis();
    // Generate dummy data with slight variation (±5%)
    dummySoilMoisture += (random(-10, 10) / 10.0);  // ±1% variation
    dummySoilMoisture = constrain(dummySoilMoisture, 30.0, 70.0);  // Keep between 30-70%
  }
  readings.soilMoisture = dummySoilMoisture;
  
  // ===== SOIL PH (DUMMY DATA FOR NOW) =====
  // Generates pseudo-random dummy data between 5.0-7.0
  // Replace with actual sensor reading when pH sensor is connected
  static unsigned long lastPHUpdate = 0;
  static float dummySoilPH = 6.0;
  
  if (millis() - lastPHUpdate > 1000) {  // Update every second
    lastPHUpdate = millis();
    // Generate dummy data with slight variation
    dummySoilPH += (random(-5, 5) / 50.0);  // ±0.1 variation
    dummySoilPH = constrain(dummySoilPH, 5.0, 7.0);  // Keep between 5-7
  }
  readings.soilPH = dummySoilPH;
  
  // Read GPS location
  if (gps.location.isValid()) {
    readings.latitude = gps.location.lat();
    readings.longitude = gps.location.lng();
  } else {
    readings.latitude = 0;
    readings.longitude = 0;
  }

  return readings;
}

// Process GPS data - call this frequently in the main loop
void processGPS() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
}

// Get number of GPS satellites in view
int getGPSSatellites() {
  if (gps.satellites.isValid()) {
    return gps.satellites.value();
  }
  return 0;
}

void printSensorStatus() {
  Serial.println("\n========== SENSOR STATUS ==========");
  Serial.print("Temperature:     ");
  Serial.println(tempSensorOK ? "OK" : "FAILED");
  Serial.print("Humidity:        ");
  Serial.println(humiditySensorOK ? "OK" : "FAILED");
  Serial.print("Gas (MQ2):       ");
  Serial.println(gasSensorOK ? "OK" : "FAILED");
  Serial.print("GPS:             ");
  Serial.println(gpsSensorOK ? "OK" : "FAILED");
  Serial.print("Soil Moisture:   ");
  Serial.println("DUMMY DATA (not connected)");
  Serial.print("IMU (MPU6050):   ");
  Serial.println(imuSensorOK ? "OK" : "FAILED");
  Serial.println("===================================\n");
}

// Read IMU data (Adafruit MPU6050)
void readIMUData(float& accelX, float& accelY, float& accelZ, float& gyroX, float& gyroY, float& gyroZ) {
  if (imuSensorOK) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    accelX = a.acceleration.x;
    accelY = a.acceleration.y;
    accelZ = a.acceleration.z;
    
    gyroX = g.gyro.x;
    gyroY = g.gyro.y;
    gyroZ = g.gyro.z;
  } else {
    accelX = accelY = accelZ = 0;
    gyroX = gyroY = gyroZ = 0;
  }
}
