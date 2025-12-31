# 🤖 Environmental Monitoring Rover System

A multi-robot environmental monitoring system built with ESP32 microcontrollers. The system consists of a main robot for data collection and navigation, a sub-robot for relay/buffering, remote controllers, and a base station for data reception.

---

## 📋 Table of Contents

- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Project Structure](#project-structure)
- [Pin Configuration](#pin-configuration)
- [Building & Flashing](#building--flashing)
- [Features](#features)
- [Communication Protocol](#communication-protocol)
- [Configuration](#configuration)
- [Usage](#usage)

---

## 🏗️ System Architecture

```
┌─────────────────┐     ESP-NOW      ┌─────────────────┐
│  Main Robot     │◄────────────────►│ Main Controller │
│  Controller     │                  │   (Joystick)    │
└────────┬────────┘                  └─────────────────┘
         │
         │ LoRa 433MHz
         ▼
┌─────────────────┐     ESP-NOW      ┌─────────────────┐
│   Sub Robot     │◄────────────────►│  Sub Controller │
│  (Relay/Buffer) │                  │   (Joystick)    │
└────────┬────────┘                  └─────────────────┘
         │
         │ LoRa 433MHz
         ▼
┌─────────────────┐
│  Base Station   │
│ (Data Receiver) │
└─────────────────┘
```

---

## 🔧 Hardware Requirements

### Main Robot
| Component | Model | Purpose |
|-----------|-------|---------|
| Microcontroller | ESP32 DOIT DevKit V1 | Main processing |
| LoRa Module | SX1278 433MHz | Long-range communication |
| Temperature/Humidity | DHT11 | Air monitoring |
| Gas Sensor | MQ2 | Air quality detection |
| GPS Module | NEO-6M | Location tracking |
| IMU | MPU6050 | Orientation & motion |
| Motor Driver | L298N / BTS7960 | 6-wheel motor control |
| Battery | 3S LiPo (11.1V) | Power supply |

### Sub Robot
| Component | Model | Purpose |
|-----------|-------|---------|
| Microcontroller | ESP32 DOIT DevKit V1 | Main processing |
| LoRa Module | SX1278 433MHz | Long-range communication |
| SD Card Module | SPI SD Card | Data logging/buffering |

### Controllers (x2)
| Component | Model | Purpose |
|-----------|-------|---------|
| Microcontroller | ESP32 DOIT DevKit V1 | Controller processing |
| Joysticks | Dual analog joysticks | Robot control |

### Base Station
| Component | Model | Purpose |
|-----------|-------|---------|
| Microcontroller | ESP32 DOIT DevKit V1 | Data reception |
| LoRa Module | SX1278 433MHz | Long-range communication |

---

## 📁 Project Structure

```
rovar_project/
├── platformio.ini              # Build configurations for all environments
├── README.md                   # This file
│
├── include/                    # Header files
│   ├── app_config.h           # LoRa pins & timing constants
│   ├── main_config.h          # Main robot pin definitions
│   ├── protocol.h             # LoRa packet protocol definitions
│   ├── esp_now_receiver.h     # ESP-NOW joystick data structure
│   ├── motor_control.h        # Motor control definitions
│   ├── sensor_init.h          # Sensor initialization
│   ├── mainrobot_lora_init.h  # Main robot LoRa setup
│   ├── subrobot_lora_init.h   # Sub robot LoRa setup
│   ├── sd_card_handler.h      # SD card logging
│   ├── wifi_mqtt_handler.h    # WiFi/MQTT communication
│   ├── lora_link.h            # LoRa interface
│   ├── navigation.h           # Navigation functions
│   └── sensors.h              # Sensor definitions
│
└── src/
    ├── base_station/          # Base station firmware
    │   └── main.cpp           # LoRa receiver & data display
    │
    ├── main_robot/            # Main robot firmware
    │   ├── main.cpp           # Main entry point
    │   ├── esp_now_receiver.cpp   # Remote control receiver
    │   ├── lora_init.cpp      # LoRa initialization
    │   ├── motor_control.cpp  # 6-wheel motor control
    │   ├── sensor_init.cpp    # Sensor initialization
    │   ├── navigation.cpp     # GPS navigation
    │   └── wifi_mqtt_handler.cpp  # WiFi/MQTT handler
    │
    ├── main_robot_control/    # Main robot remote controller
    │   └── main.cpp           # Dual joystick ESP-NOW sender
    │
    ├── sub_robot/             # Sub robot firmware
    │   ├── main.cpp           # Relay & buffer entry point
    │   ├── esp_now_receiver.cpp   # Remote control receiver
    │   ├── lora_init.cpp      # LoRa initialization
    │   └── sd_card_handler.cpp    # SD card data logging
    │
    └── sub_robot_control/     # Sub robot remote controller
        └── main.cpp           # Dual joystick ESP-NOW sender
```

---

## 📌 Pin Configuration

### LoRa Module (All devices)
| Signal | GPIO |
|--------|------|
| SCK | 18 |
| MISO | 19 |
| MOSI | 23 |
| CS | 5 |
| RST | 14 |
| DIO0 | 2 (Main/Sub) / 26 (Base) |

### Main Robot - Sensors
| Sensor | GPIO | Notes |
|--------|------|-------|
| DHT11 (Temp/Humidity) | 27 | Digital |
| MQ2 (Gas) | 32 | Analog (ADC1) |
| GPS RX | 16 | UART2 |
| GPS TX | 17 | UART2 |
| MPU6050 SDA | 21 | I2C |
| MPU6050 SCL | 22 | I2C |

### Main Robot - Motors (6-wheel drive)
| Signal | GPIO | Notes |
|--------|------|-------|
| Left DIR1 | 13 | Direction control |
| Left DIR2 | 33 | Direction control |
| Left PWM | 25 | Speed control |
| Right DIR1 | 12 | Direction control |
| Right DIR2 | 15 | Direction control |
| Right PWM | 4 | Speed control |

### Sub Robot - SD Card
| Signal | GPIO |
|--------|------|
| CS | 5 |
| SCK | 18 |
| MISO | 19 |
| MOSI | 23 |

### Controllers - Joysticks
| Signal | GPIO | Notes |
|--------|------|-------|
| Left Joy X | 34 | ADC1_CH6 |
| Left Joy Y | 35 | ADC1_CH7 |
| Left Joy Button | 27 | Digital |
| Right Joy X | 33 | ADC1_CH5 |
| Right Joy Y | 32 | ADC1_CH4 |
| Right Joy Button | 14 | Digital |

---

## 🔨 Building & Flashing

### Prerequisites
- [PlatformIO](https://platformio.org/) installed (VS Code extension recommended)
- USB drivers for ESP32

### Build Environments

| Environment | Description | COM Port |
|-------------|-------------|----------|
| `main_robot` | Main robot with sensors & motors | COM5 |
| `sub_robot` | Sub robot relay & SD logging | COM5 |
| `main_robot_control` | Main robot joystick controller | COM5 |
| `sub_robot_control` | Sub robot joystick controller | COM5 |

### Commands

```bash
# Build specific environment
pio run -e main_robot

# Upload to device
pio run -e main_robot -t upload

# Monitor serial output
pio device monitor -e main_robot

# Build and upload in one command
pio run -e main_robot -t upload && pio device monitor -e main_robot
```

### All Environments Build Commands

```bash
# Main Robot (with sensors, motors, LoRa)
pio run -e main_robot -t upload

# Sub Robot (relay, SD card, LoRa)
pio run -e sub_robot -t upload

# Main Robot Controller (joystick)
pio run -e main_robot_control -t upload

# Sub Robot Controller (joystick)
pio run -e sub_robot_control -t upload
```

---

## ✨ Features

### 🚗 Main Robot
- **Environmental Sensors**: DHT11 (temperature/humidity), MQ2 (gas), GPS (location), MPU6050 (IMU)
- **6-Wheel Motor Control**: Differential drive with PWM speed control
- **ESP-NOW Remote Control**: Wireless joystick control via ESP-NOW protocol
- **LoRa Communication**: 433MHz long-range data transmission to sub-robot/base station
- **Real-time Data Transmission**: Sends sensor data every 5 seconds

### 📡 Sub Robot
- **LoRa Relay Mode**: Extends communication range by relaying data
- **SD Card Data Logging**: Stores received sensor data in CSV format
- **Buffer Mode**: Stores data when main robot is out of range
- **ESP-NOW Remote Control**: Wireless joystick control via ESP-NOW protocol
- **Status Reporting**: Periodic status updates including RSSI, data count

### 🎮 Controllers
- **Dual Joystick Input**: Left and right analog joysticks
- **ESP-NOW Transmission**: Low-latency wireless control
- **Button Support**: Emergency stop functionality

### 📊 Base Station
- **LoRa Data Reception**: Receives sensor data from robot network
- **Serial Data Display**: Real-time sensor data visualization
- **Status LED**: Visual indication of data reception

---

## 📡 Communication Protocol

### LoRa Configuration
| Parameter | Value |
|-----------|-------|
| Frequency | 433 MHz |
| TX Power | 20 dBm |
| Spreading Factor | 7 |
| Bandwidth | 125 kHz |
| Coding Rate | 4/5 |
| Sync Word | 0x12 |

### Packet Types
| Type | Code | Description |
|------|------|-------------|
| PING | 0x01 | Connection test |
| PONG | 0x02 | Ping response |
| DATA | 0x10 | Sensor data payload |
| STATUS | 0x11 | Device status |
| COMMAND | 0x20 | Control command |
| ACK | 0x30 | Acknowledgment |
| SYNC_REQUEST | 0x40 | Sync request |
| SYNC_RESPONSE | 0x41 | Sync response |

### Commands
| Command | Description |
|---------|-------------|
| `GET_STATUS` | Request device status |
| `GET_DATA_COUNT` | Get buffered data count |
| `SEND_BUFFER` | Transmit buffered data |
| `CLEAR_BUFFER` | Clear data buffer |
| `ENABLE_RELAY` | Enable relay mode |
| `DISABLE_RELAY` | Disable relay mode |
| `RETURN_HOME` | Return to home position |
| `STOP` | Emergency stop |

### Sensor Data Format
```
TEMP:27.4,HUM:78.7,GAS:12.5,SOIL:45.6,LAT:6.123456,LNG:80.123456,AX:-0.04,AY:-0.44,AZ:9.92,GX:-0.01,GY:0.03,GZ:-0.00
```

---

## ⚙️ Configuration

### WiFi/MQTT (main_config.h)
```cpp
#define WIFI_SSID          "your_wifi_ssid"
#define WIFI_PASSWORD      "your_wifi_password"
#define MQTT_SERVER        "mqtt.example.com"
#define MQTT_PORT          1883
#define MQTT_USER          "rover_user"
#define MQTT_PASSWORD      "rover_password"
```

### Controller MAC Address
Update the receiver MAC address in controller files:
```cpp
// main_robot_control/main.cpp
uint8_t receiverMac[] = { 0x88, 0x57, 0x21, 0x8E, 0xD1, 0xC4 };

// sub_robot_control/main.cpp
uint8_t receiverMac[] = { 0x68, 0xFE, 0x71, 0x81, 0x88, 0x18 };
```

### Timing Configuration (app_config.h)
| Parameter | Value | Description |
|-----------|-------|-------------|
| SAMPLE_INTERVAL | 30000 ms | Sensor sampling interval |
| TRANSMIT_INTERVAL | 60000 ms | Data transmission interval |
| SYNC_INTERVAL | 120000 ms | Sync check interval |
| BATTERY_CHECK_INTERVAL | 10000 ms | Battery monitoring interval |

---

## 🚀 Usage

### Initial Setup

1. **Flash all devices** with their respective firmware
2. **Get MAC addresses** from serial output of robots:
   ```
   Receiver MAC: XX:XX:XX:XX:XX:XX
   ```
3. **Update controller MAC addresses** and re-flash controllers
4. **Power on in order**: Controllers → Robots → Base Station

### Operating the Main Robot

1. Power on the main robot
2. Wait for initialization (status LED turns off when ready)
3. Power on the main robot controller
4. Use left joystick for movement control
5. Sensor data transmits automatically every 5 seconds

### Monitoring Data

1. Open serial monitor for base station (115200 baud)
2. Received data displays in format:
   ```
   ========== SENSOR DATA RECEIVED ==========
   Data: TEMP:27.4,HUM:78.7,GAS:12.5,...
   ==========================================
   ```

---

## 📚 Dependencies

| Library | Version | Purpose |
|---------|---------|---------|
| LoRa | ^0.8.0 | LoRa communication |
| DHT sensor library | ^1.4.4 | Temperature/humidity |
| TinyGPSPlus | ^1.0.3 | GPS parsing |
| Adafruit MPU6050 | ^2.2.6 | IMU sensor |
| Adafruit Unified Sensor | ^1.1.14 | Sensor abstraction |
| Adafruit BMP280 Library | ^2.6.6 | Pressure sensor |
| ArduinoJson | ^6.21.3 | JSON handling |
| PubSubClient | ^2.8 | MQTT client |
| SD (esp32) | - | SD card support |

---

## 🔮 Future Improvements

- [ ] Implement WiFi relay mode in sub robot
- [ ] Add water quality sensors (pH, turbidity, DO)
- [ ] Implement autonomous navigation with obstacle avoidance
- [ ] Add MQTT cloud data upload
- [ ] Implement battery monitoring and low-battery return home
- [ ] Add web dashboard for real-time monitoring

---

## 📄 License

This project is open-source. Feel free to use and modify for your own projects.

---

## 👥 Contributors

- Environmental Monitoring Rover Team
