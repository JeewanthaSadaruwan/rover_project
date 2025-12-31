/*
 * Firebase Configuration
 * Replace the placeholder values with your actual credentials
 */

#ifndef FIREBASE_CONFIG_H
#define FIREBASE_CONFIG_H

// ==================== WiFi CREDENTIALS ====================
// Replace with your WiFi network name and password
#define WIFI_SSID           "XYZ"
#define WIFI_PASSWORD       "12345678"

// ==================== FIREBASE CREDENTIALS ====================
// From google-services.json
#define FIREBASE_API_KEY    "AIzaSyDQkcIPm3OLDINcDXxD1UTr_4PX_Qszphg"

// Firebase Realtime Database URL (without https://)
#define FIREBASE_DATABASE_URL "env-rover-default-rtdb.asia-southeast1.firebasedatabase.app"

// ==================== FIREBASE AUTHENTICATION ====================
// Option 1: Email/Password Authentication
// Create a user in Firebase Console -> Authentication -> Users
#define FIREBASE_USER_EMAIL    "jeewansadaruwan987@gmail.com"
#define FIREBASE_USER_PASSWORD "123456"

// ==================== DATABASE PATHS ====================
// Path where sensor data will be stored in Firebase
#define FIREBASE_ROVER_ID      "rover_01"
#define FIREBASE_BASE_PATH     "/rovers/rover_01"
#define FIREBASE_LATEST_PATH   "/rovers/rover_01/latest"
#define FIREBASE_HISTORY_PATH  "/rovers/rover_01/history"
#define FIREBASE_STATUS_PATH   "/rovers/rover_01/status"

#endif // FIREBASE_CONFIG_H
