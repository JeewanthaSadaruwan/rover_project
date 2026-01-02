/*
 * SUB ROBOT CONTROL - Joystick Transmitter
 * Reads joysticks and sends data via ESP-NOW to example_sub_robot
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// -------- RECEIVER MAC ADDRESS --------
// Replace with MAC from example_sub_robot Serial output!
uint8_t receiverMac[] = {0x68, 0xFE, 0x71, 0x81, 0x88, 0x18};

// ================================
// JOYSTICK PIN DEFINITIONS
// ================================

// Left Joystick
#define JOY_L_X   34    // ADC1_CH6 (input only)
#define JOY_L_Y   35    // ADC1_CH7 (input only)
#define JOY_L_SW  27    // Button

// Right Joystick
#define JOY_R_X   33    // ADC1_CH5
#define JOY_R_Y   32    // ADC1_CH4
#define JOY_R_SW  14    // Button

// ================================
// ESP-NOW DATA STRUCTURES
// ================================
// Packet types
#define PACKET_JOYSTICK 0x01
#define PACKET_ACK      0x02

// Joystick packet - sent TO receiver
typedef struct {
  uint8_t type;
  int16_t leftX;
  int16_t leftY;
  int16_t rightX;
  int16_t rightY;
  bool leftBtn;
  bool rightBtn;
} JoystickPacket;

// ACK packet - received FROM receiver
typedef struct {
  uint8_t type;
  uint8_t status;  // 1 = alive
} AckPacket;

JoystickPacket txData;
bool espNowReady = false;
volatile bool lastSendOk = false;
volatile bool peerConnected = false;
volatile unsigned long lastAckTime = 0;
unsigned long lastReconnectAttempt = 0;
int failCount = 0;
#define RECONNECT_INTERVAL 3000  // Try reconnect every 3 seconds
#define FAIL_THRESHOLD 10        // Reconnect after 10 consecutive failures
#define ACK_TIMEOUT 1000         // Peer offline if no ACK for 1 second

// ================================
// ESP-NOW CALLBACKS
// ================================
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  lastSendOk = (status == ESP_NOW_SEND_SUCCESS);
  if (lastSendOk) {
    failCount = 0;
  } else {
    failCount++;
  }
}

// Receive ACK from receiver
void onDataReceive(const uint8_t *mac, const uint8_t *data, int len) {
  if (len >= 1 && data[0] == PACKET_ACK) {
    lastAckTime = millis();
    if (!peerConnected) {
      peerConnected = true;
      Serial.println("\n*** RECEIVER CONNECTED! ***\n");
    }
  }
}

// ================================
// ESP-NOW INITIALIZATION
// ================================
bool initESPNow() {
  Serial.println("\n[ESP-NOW] Initializing...");
  
  // Setup WiFi first
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  
  Serial.print("[WiFi] This MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.print("[ESP-NOW] Target MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", receiverMac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init FAILED!");
    return false;
  }
  Serial.println("[ESP-NOW] Initialized");

  // Register callbacks
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceive);

  // Add peer (receiver)
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, receiverMac, 6);
  peer.channel = 0;
  peer.encrypt = false;

  // Check if peer exists first
  if (esp_now_is_peer_exist(receiverMac)) {
    esp_now_del_peer(receiverMac);
  }
  
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ESP-NOW] Failed to add peer!");
    return false;
  }
  
  Serial.println("[ESP-NOW] Peer added - waiting for receiver...");
  failCount = 0;
  peerConnected = false;
  return true;
}

// ================================
// SETUP
// ================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("  SUB ROBOT CONTROL - Joystick TX");
  Serial.println("========================================\n");

  // Button pins
  pinMode(JOY_L_SW, INPUT_PULLUP);
  pinMode(JOY_R_SW, INPUT_PULLUP);

  // Initialize ESP-NOW (will retry in loop if fails)
  espNowReady = initESPNow();
  
  if (!espNowReady) {
    Serial.println("\n[WARNING] ESP-NOW not ready, will retry...");
  }

  Serial.println("\nControls:");
  Serial.println("  Left Joystick Y  = Forward/Backward");
  Serial.println("  Right Joystick X = Turn Left/Right");
  Serial.println("\nSending data...\n");
}

// ================================
// MAIN LOOP
// ================================
void loop() {
  unsigned long currentTime = millis();
  
  // ===== RECONNECTION LOGIC =====
  // Try to reconnect if not ready OR too many consecutive failures
  if (!espNowReady || failCount >= FAIL_THRESHOLD) {
    if (currentTime - lastReconnectAttempt >= RECONNECT_INTERVAL) {
      lastReconnectAttempt = currentTime;
      
      if (failCount >= FAIL_THRESHOLD) {
        Serial.println("\n[WARNING] Too many TX failures, reconnecting...");
      }
      
      espNowReady = initESPNow();
      
      if (!espNowReady) {
        Serial.println("[ESP-NOW] Reconnect failed, will retry in 3s...");
      }
    }
    
    // Show waiting status
    static unsigned long lastWaitPrint = 0;
    if (!espNowReady && currentTime - lastWaitPrint >= 1000) {
      Serial.println("[WAITING] ESP-NOW not connected... Retrying...");
      lastWaitPrint = currentTime;
    }
    
    if (!espNowReady) {
      delay(100);
      return;
    }
  }
  
  // Check if peer is still connected (ACK timeout)
  if (peerConnected && (currentTime - lastAckTime > ACK_TIMEOUT)) {
    peerConnected = false;
    Serial.println("\n*** RECEIVER DISCONNECTED! ***\n");
  }
  
  // Read joystick values (0-4095)
  txData.type = PACKET_JOYSTICK;
  txData.leftX = analogRead(JOY_L_X);
  txData.leftY = analogRead(JOY_L_Y);
  txData.rightX = analogRead(JOY_R_X);
  txData.rightY = analogRead(JOY_R_Y);

  // Read buttons (LOW = pressed)
  txData.leftBtn = (digitalRead(JOY_L_SW) == LOW);
  txData.rightBtn = (digitalRead(JOY_R_SW) == LOW);

  // Send data via ESP-NOW
  esp_now_send(receiverMac, (uint8_t*)&txData, sizeof(txData));

  // Print status
  static unsigned long lastPrint = 0;
  if (currentTime - lastPrint >= 200) {
    Serial.print(peerConnected ? "[CONNECTED] " : "[SEARCHING] ");
    Serial.print("LY:"); Serial.print(txData.leftY);
    Serial.print(" RX:"); Serial.print(txData.rightX);
    Serial.print(" | BTN:");
    Serial.print(txData.leftBtn ? "L" : "-");
    Serial.println(txData.rightBtn ? "R" : "-");
    lastPrint = currentTime;
  }

  delay(50);  // 20Hz update rate
}
