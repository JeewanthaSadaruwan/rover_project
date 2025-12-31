/*
 * Application Configuration
 * Shared constants and pin definitions
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

// ========== LORA PINS (Same for both robots) ==========
#define LORA_SCK            18
#define LORA_MISO           19
#define LORA_MOSI           23
#define LORA_CS             5
#define LORA_RST            14
#define LORA_DIO0           26

// ========== LORA CONFIGURATION ==========
#ifndef LORA_FREQUENCY
#define LORA_FREQUENCY      433E6    // 433 MHz
#endif
#define LORA_TX_POWER       20       // dBm
#define LORA_SPREADING_FACTOR 7      // 7-12
#define LORA_BANDWIDTH      125E3
#define LORA_CODING_RATE    5
#define LORA_SYNC_WORD      0x12

// ========== TIMING ==========
#define SAMPLE_INTERVAL     30000    // 30 seconds
#define TRANSMIT_INTERVAL   60000    // 60 seconds
#define SYNC_INTERVAL       120000   // 2 minutes
#define BATTERY_CHECK_INTERVAL 10000 // 10 seconds

// ========== BUFFERS ==========
#define MAX_BUFFER_SIZE     100

#endif // APP_CONFIG_H
