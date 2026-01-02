/*
 * EXAMPLE MAIN ROBOT - Motor Control Receiver
 * Receives joystick data via ESP-NOW and controls motors
 * 
 * Controls:
 * - Left Joystick Y = Forward/Backward (both motors)
 * - Right Joystick X = Turn Left/Right (steering)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// ================================
// MOTOR PIN DEFINITIONS (L298 on Main Robot)
// ================================
// Left Motor (3 motors connected together)
#define MOTOR_LEFT_PWM      25    // ENA - Left motors speed
#define MOTOR_LEFT_IN1      13    // IN1 - Left motors direction
#define MOTOR_LEFT_IN2      33    // IN2 - Left motors direction

// Right Motor (3 motors connected together)
#define MOTOR_RIGHT_PWM     4     // ENB - Right motors speed
#define MOTOR_RIGHT_IN1     12    // IN3 - Right motors direction
#define MOTOR_RIGHT_IN2     15    // IN4 - Right motors direction

// PWM settings
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define PWM_CHANNEL_LEFT 0
#define PWM_CHANNEL_RIGHT 1

// ================================
// ESP-NOW DATA STRUCTURES
// ================================
// Packet types
#define PACKET_JOYSTICK 0x01
#define PACKET_ACK      0x02

// Joystick packet - received FROM controller
typedef struct {
  uint8_t type;
  int16_t leftX;
  int16_t leftY;
  int16_t rightX;
  int16_t rightY;
  bool leftBtn;
  bool rightBtn;
} JoystickPacket;

// ACK packet - sent TO controller
typedef struct {
  uint8_t type;
  uint8_t status;
} AckPacket;

JoystickPacket joystickData = {PACKET_JOYSTICK, 2048, 2048, 2048, 2048, false, false};
unsigned long lastDataTime = 0;
bool controllerConnected = false;
uint8_t controllerMac[6];
#define CONNECTION_TIMEOUT 500  // Stop motors if no data for 500ms

// ================================
// MOTOR CONTROL FUNCTIONS
// ================================
void initMotors() {
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);
  
  ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_LEFT_PWM, PWM_CHANNEL_LEFT);
  ledcAttachPin(MOTOR_RIGHT_PWM, PWM_CHANNEL_RIGHT);
  
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
  ledcWrite(PWM_CHANNEL_LEFT, 0);
  ledcWrite(PWM_CHANNEL_RIGHT, 0);
}

void setMotor(int channel, int in1Pin, int in2Pin, int speed) {
  if (speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
  ledcWrite(channel, abs(speed));
}

void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  setMotor(PWM_CHANNEL_LEFT, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, leftSpeed);
  setMotor(PWM_CHANNEL_RIGHT, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, rightSpeed);
}

void stopMotors() {
  setMotors(0, 0);
}

// ================================
// JOYSTICK TO MOTOR CONVERSION
// ================================
int mapJoystick(int value, int deadzone = 200) {
  int centered = value - 2048;
  if (abs(centered) < deadzone) {
    return 0;
  }
  if (centered > 0) {
    return map(centered, deadzone, 2047, 0, 255);
  } else {
    return map(centered, -2047, -deadzone, -255, 0);
  }
}

// ================================
// ARCADE DRIVE MIXING
// ================================
void arcadeDrive(int throttle, int steering, int& leftSpeed, int& rightSpeed) {
  leftSpeed = throttle + steering;
  rightSpeed = throttle - steering;
  
  int maxSpeed = max(abs(leftSpeed), abs(rightSpeed));
  if (maxSpeed > 255) {
    leftSpeed = (leftSpeed * 255) / maxSpeed;
    rightSpeed = (rightSpeed * 255) / maxSpeed;
  }
}

// ================================
// ESP-NOW CALLBACKS
// ================================
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // ACK sent callback
}

void onDataReceive(const uint8_t *mac, const uint8_t *data, int len) {
  if (len >= 1 && data[0] == PACKET_JOYSTICK && len == sizeof(JoystickPacket)) {
    memcpy(&joystickData, data, sizeof(JoystickPacket));
    lastDataTime = millis();
    
    // Save controller MAC and add as peer if not already
    if (!controllerConnected) {
      memcpy(controllerMac, mac, 6);
      
      // Add controller as peer to send ACK back
      esp_now_peer_info_t peer;
      memset(&peer, 0, sizeof(peer));
      memcpy(peer.peer_addr, mac, 6);
      peer.channel = 0;
      peer.encrypt = false;
      
      if (!esp_now_is_peer_exist(mac)) {
        esp_now_add_peer(&peer);
      }
      controllerConnected = true;
      Serial.println("\n*** CONTROLLER CONNECTED! ***\n");
    }
    
    // Send ACK back to controller
    AckPacket ack = {PACKET_ACK, 1};
    esp_now_send(controllerMac, (uint8_t*)&ack, sizeof(ack));
  }
}

// ================================
// SETUP
// ================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("  EXAMPLE MAIN ROBOT - Motor Receiver");
  Serial.println("========================================\n");
  
  // Initialize motors
  initMotors();
  Serial.println("[Motors] Initialized");
  Serial.println("  LEFT:  PWM=25, IN1=13, IN2=33 (3 motors)");
  Serial.println("  RIGHT: PWM=4,  IN1=12, IN2=15 (3 motors)");
  
  // Initialize WiFi for ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  Serial.print("\n[WiFi] MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.println("[WiFi] >>> Use this MAC in main_robot_control! <<<\n");
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init FAILED!");
    while (1) delay(1000);
  }
  Serial.println("[ESP-NOW] Initialized");
  
  // Register callbacks
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceive);
  Serial.println("[ESP-NOW] Waiting for controller...\n");
}

// ================================
// MAIN LOOP
// ================================
void loop() {
  unsigned long currentTime = millis();
  
  // Check for connection timeout
  if (currentTime - lastDataTime > CONNECTION_TIMEOUT) {
    stopMotors();
    
    if (controllerConnected) {
      controllerConnected = false;
      Serial.println("\n*** CONTROLLER DISCONNECTED! ***\n");
    }
    
    static unsigned long lastTimeoutPrint = 0;
    if (currentTime - lastTimeoutPrint > 2000) {
      Serial.println("[WAITING] No controller - motors stopped");
      lastTimeoutPrint = currentTime;
    }
    return;
  }
  
  // CONTROL: 
  // Left Y = Forward/Backward (both motors)
  // Right X = Turn (steering)
  int throttle = mapJoystick(joystickData.leftY);   // -255 to 255
  int steering = mapJoystick(joystickData.rightX);  // -255 to 255
  
  int leftSpeed, rightSpeed;
  arcadeDrive(throttle, steering, leftSpeed, rightSpeed);
  
  // Apply to BOTH motors
  setMotors(leftSpeed, rightSpeed);
  
  // Print status
  static unsigned long lastPrint = 0;
  if (currentTime - lastPrint > 100) {
    String direction = "STOP";
    if (throttle > 0 && steering == 0) direction = "FORWARD";
    else if (throttle < 0 && steering == 0) direction = "BACKWARD";
    else if (throttle == 0 && steering > 0) direction = "TURN RIGHT";
    else if (throttle == 0 && steering < 0) direction = "TURN LEFT";
    else if (throttle > 0) direction = "FWD+TURN";
    else if (throttle < 0) direction = "BWD+TURN";
    
    Serial.printf("[%s] Thr:%4d Str:%4d -> L:%4d R:%4d\n",
      direction.c_str(),
      throttle, steering,
      leftSpeed, rightSpeed);
    lastPrint = currentTime;
  }
}
