/*
 * Badminton Motion Tracking - XIAO nRF52840 Sense
 * Optimized data transmission rates for BLE and Serial
 * Added time synchronization feature
 * Replaced PDM mic with mic.h implementation
 */

#include <Arduino.h>
#include <Wire.h>
#include <LSM6DS3.h>
#include <ArduinoBLE.h>
#include <xiaobattery.h>
#include <TimeLib.h>
#include <mic.h>  // Added mic.h instead of PDM.h
#include "nrf.h"
#include "version.h"

// Global variable to store the unique device ID
uint16_t DEVICE_ID = 0;

// Create IMU instance
LSM6DS3 imu(I2C_MODE, 0x6A);  // I2C address = 0x6A

// Create Battery monitoring
Xiao battery;
#define BATTERY_SAMPLE_COUNT 10
float batteryVoltageHistory[BATTERY_SAMPLE_COUNT] = { 0 };
int batteryHistoryIndex = 0;

// Microphone buffer and settings - NEW IMPLEMENTATION
#define MIC_BUFFER_SIZE 256
#define MIC_SAMPLE_RATE 16000

// Mic configuration (from mic_serial_plotter)
mic_config_t mic_config{
  .channel_cnt = 1,
  .sampling_rate = 16000,
  .buf_size = 1600,
  .debug_pin = LED_BUILTIN  // Toggles each DAC ISR (if DEBUG is set to 1)
};

// Mic instance based on architecture
#if defined(ARDUINO_ARCH_NRF52840)
NRF52840_ADC_Class Mic(&mic_config);
#else
// Fallback or other architecture
#error "Unsupported microcontroller for mic.h library"
#endif

// Buffer for microphone recording
#define SAMPLES 800
int16_t recording_buf[SAMPLES];
volatile uint8_t recording = 0;
volatile static bool record_ready = false;

// Define UUIDs for the BLE service and characteristic
#define DESCRIPTOR_UUID "2901"  // Standard UUID for user description string

#define VERSION_INFO_CHAR_UUID "14A168D7-04D1-6C4F-7E53-F2E807B11900"

#define IMU_SERVICE_UUID "14A168D7-04D1-6C4F-7E53-F2E800B11900"
#define IMU_CHARACTERISTIC_UUID "14A168D7-04D1-6C4F-7E53-F2E801B11900"

#define TIME_SYNC_CHAR_UUID "14A168D7-04D1-6C4F-7E53-F2E802B11900"

#define MICROPHONE_SERVICE_UUID "14A168D7-04D1-6C4F-7E53-F2E804B11900"
#define MICROPHONE_CONTROL_CHAR_UUID "14A168D7-04D1-6C4F-7E53-F2E806B11900"

#define BATTERY_CHARGING_CHAR_UUID "14A168D7-04D1-6C4F-7E53-F2E803B11900"
#define BATTERY_CHAR_UUID "2A19"  // Standard BLE battery characteristic

// Define UUID for loop time characteristic
#define LOOP_TIME_CHAR_UUID "14A168D7-04D1-6C4F-7E53-F2E808B11900"
#define LOOP_TIME_RESET_CHAR_UUID "14A168D7-04D1-6C4F-7E53-F2E809B11900"


// Buffer for collecting multiple IMU readings before sending
#define BUFFER_SIZE 5     // Store 5 readings (at 20ms interval = 100ms of data)
#define BLE_DATA_SIZE 38  // Timestamp + Device + IMU + MIC data

typedef struct {
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  uint64_t timestamp;  // timestamp
  uint64_t deviceId;   // device ID
} imu_reading_t;

typedef struct {
  int32_t level;       // Current
  int32_t peak;        // Peak
  uint64_t timestamp;  // timestamp
  uint16_t deviceId;   // device ID
} mic_reading_t;

// ============== BLE : Version characteristics ==============
BLEDescriptor versionInfoDesc(DESCRIPTOR_UUID, "Version Info");
BLECharacteristic versionInfoChar(VERSION_INFO_CHAR_UUID, BLERead, 32);

/// ============== BLE : IMU  Service and characteristics ==============
BLEDescriptor imuServiceDesc(DESCRIPTOR_UUID, "IMU Service");
BLEDescriptor imuCharDesc(DESCRIPTOR_UUID, "IMU Data");
BLEDescriptor timeSyncCharDesc(DESCRIPTOR_UUID, "Time Sync");

BLEService imuService(IMU_SERVICE_UUID);
BLECharacteristic imuCharacteristic(IMU_CHARACTERISTIC_UUID,
                                    BLERead | BLENotify,
                                    BLE_DATA_SIZE);

// Time sync characteristic - allows client to write timestamp
// Using 8 bytes for timestamp to accommodate Unix epoch timestamps or other large values
BLECharacteristic timeSyncCharacteristic(TIME_SYNC_CHAR_UUID,
                                         BLERead | BLEWrite, 8);  // 8 bytes for 64-bit timestamp

// ============== BLE : Battery Service and characteristics ==============
BLEDescriptor batteryServiceDesc(DESCRIPTOR_UUID, "Battery Service");
BLEDescriptor batteryLevelDesc(DESCRIPTOR_UUID, "Battery Level");
BLEDescriptor batteryChargingDesc(DESCRIPTOR_UUID, "Battery Charging Status");

BLEService batteryService("180F");  // Standard battery service
BLEUnsignedCharCharacteristic batteryLevelChar(BATTERY_CHAR_UUID, BLERead | BLENotify);
BLEBoolCharacteristic batteryChargingChar(BATTERY_CHARGING_CHAR_UUID, BLERead | BLENotify);

// ============== BLE : Microphone Service and characteristics ==============
BLEDescriptor micServiceDesc(DESCRIPTOR_UUID, "Microphone Service");
BLEDescriptor micControlDesc(DESCRIPTOR_UUID, "Microphone Control");

BLEService microphoneService(MICROPHONE_SERVICE_UUID);                     // Service for microphone related functions
BLEByteCharacteristic microphoneControlChar(MICROPHONE_CONTROL_CHAR_UUID,  // Characteristic for control
                                            BLERead | BLEWrite);

// Add these BLE characteristics declarations after other BLE characteristics (around line 170)
BLEDescriptor loopTimeCharDesc(DESCRIPTOR_UUID, "Loop Time Data");
BLEDescriptor loopTimeResetDesc(DESCRIPTOR_UUID, "Reset Loop Time");
BLELongCharacteristic loopTimeChar(LOOP_TIME_CHAR_UUID, BLERead | BLENotify);
BLEBoolCharacteristic loopTimeResetChar(LOOP_TIME_RESET_CHAR_UUID, BLERead | BLEWrite);

// Timer variables
unsigned long lastImuMicRead = 0;
unsigned long lastBleTransmit = 0;
unsigned long lastSerialTransmit = 0;
unsigned long lastBatteryRead = 0;
unsigned long lastBatteryUpdate = 0;
unsigned long lastSyncTime = 0;
unsigned long blinkTimer = 0;

// Time sync variables
int64_t deviceTimeOffset = 0;  // Using 64-bit integer for larger time values


// Default timestamp: 2025-04-14 00:00:00 TST in milliseconds since epoch
uint64_t systemBaseTime = 1744876800000ULL;
// Store millis() at sync time
unsigned long syncReceivedMillis = 0;
bool timeIsSynced = false;

// Different intervals for different operations
const int readImuMicInterval = 20;        // 20ms = 50Hz for reading IMU and MIC (capture motion accurately)
const int bleTransmitInterval = 100;      // 100ms = 10Hz for BLE transmission (reduced from 50Hz)
const int serialTransmitInterval = 500;   // 500ms = 2Hz for Serial output (further reduced)
const int batteryReadInterval = 1000;     // 1000ms = 5Hz for Serial output (further reduced)
const int batteryUpdateInterval = 30000;  // 30s battery update rate
const int micUpdateInterval = 100;        // 100ms = 10Hz update rate for mic data
const int levelResetInterval = 3000;      // Reset peak level every 3 seconds

imu_reading_t imuBuffer[BUFFER_SIZE];
mic_reading_t micBuffer[BUFFER_SIZE];
int sharedBufferIndex = 0;
bool bufferFull = false;

// State variables to avoid blocking loops
bool imuInitialized = false;
bool bleInitialized = false;
int errorBlinkState = 0;
unsigned long errorBlinkTimer = 0;

// Timer for microphone data
unsigned long lastMicUpdate = 0;

// Flag to control microphone streaming
bool micStreamingEnabled = false;

// Add these variables for sound level calculation
int32_t microphoneLevel = 0;
int32_t microphonePeak = 0;
unsigned long lastLevelReset = 0;

// Add these global variables after other timer variables (around line 195)
unsigned long loopStartTime = 0;
unsigned long loopTime = 0;            // Current loop execution time
unsigned long maxLoopTime = 0;         // Maximum recorded loop time
unsigned long lastLoopTimeUpdate = 0;  // For periodic loop time updates


unsigned long lastToggle = 0;
int blinkCount = 0;
int ledState = LOW;

void setup() {
  // Initialize LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);  // Brief flash only
  digitalWrite(LED_BUILTIN, LOW);

  // Defiend Charge current with 100mA (LOW) [50mA: High , 100mA: Low]
  battery = Xiao();
  digitalWrite(BAT_HIGH_CHARGE, LOW);

  // Initialize Serial for debugging - shorter delay
  Serial.begin(1000000);  // Higher baud rate for faster serial
  delay(100);             // Reduced delay

  // Get and store unique device ID ONCE
  uint64_t deviceid = getUniqueDeviceID();
  DEVICE_ID = getShortDeviceID(deviceid);
  String formattedShortID = formatShortDeviceID(DEVICE_ID);

  Serial.println("Device ID (16 bits):");
  Serial.println(formattedShortID);

  Serial.println("\n\n====================================");
  Serial.println("Badminton Motion Tracking - XIAO nRF52840");
  printVersionInfo();
  Serial.println("Optimized data transmission rates");
  Serial.println("Added time synchronization feature");
  Serial.println("Updated with mic.h microphone implementation");
  Serial.println("====================================");

  // Print default timestamp
  Serial.println("Default timestamp set to: April 14, 2025 00:00:00 UTC");
  Serial.print("In milliseconds since epoch: ");
  Serial.print((uint32_t)(systemBaseTime >> 32), HEX);
  Serial.print("-");
  Serial.println((uint32_t)systemBaseTime, HEX);

  // Enable Battery Low Charging Mode


  // Start IMU initialization - don't block on failure
  Serial.println("Initializing IMU...");
  if (imu.begin() != 0) {
    Serial.println("Failed to initialize IMU!");
    // Don't block in an infinite loop - just note the failure
    imuInitialized = false;
  } else {
    Serial.println("IMU initialized successfully");
    imuInitialized = true;
  }

  // Initialize microphone using new mic.h implementation
  bool micInitialized = initializeMicrophone();
  if (micInitialized) {
    Serial.println("Mic initialization successful, starting recording...");
    micStreamingEnabled = true;
  } else {
    Serial.println("WARNING: Mic initialization failed, no audio data will be available");
  }
  // Start BLE initialization - don't block on failure
  Serial.println("Initializing BLE...");
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    // Don't block in an infinite loop - just note the failure
    bleInitialized = false;
  } else {
    bleInitialized = true;
    Serial.println("BLE initialized successfully");

    // Set BLE device name and advertised services
    // Add device ID to local name and advertising for more clarity
    char localName[25];
    snprintf(localName, sizeof(localName), "Badminton Tracker (%s)", formattedShortID.c_str());
    Serial.println(localName);
    BLE.setLocalName(localName);
    BLE.setAdvertisedService(imuService);

    // Add Loop Time characteristics to IMU service
    loopTimeChar.addDescriptor(loopTimeCharDesc);
    loopTimeResetChar.addDescriptor(loopTimeResetDesc);
    imuService.addCharacteristic(loopTimeChar);
    imuService.addCharacteristic(loopTimeResetChar);

    // Set initial values
    loopTimeChar.writeValue(0);
    loopTimeResetChar.writeValue(false);

    // Add IMU characteristic to the service
    imuService.addCharacteristic(imuCharacteristic);

    // Add Version characteristic to the service
    imuService.addCharacteristic(versionInfoChar);
    versionInfoChar.addDescriptor(versionInfoDesc);

    updateVersionInfo();

    // Add Time Sync characteristic to the IMU service
    imuService.addCharacteristic(timeSyncCharacteristic);

    // Set initial value for time sync characteristic
    uint8_t initValue[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    timeSyncCharacteristic.writeValue(initValue, 8);

    // Add the service
    imuCharacteristic.addDescriptor(imuCharDesc);
    timeSyncCharacteristic.addDescriptor(timeSyncCharDesc);
    BLE.setAdvertisedServiceUuid(IMU_SERVICE_UUID);
    BLE.addService(imuService);

    // Setup Battery Service
    // Add descriptors to Battery characteristics
    batteryLevelChar.addDescriptor(batteryLevelDesc);
    batteryChargingChar.addDescriptor(batteryChargingDesc);

    batteryService.addCharacteristic(batteryLevelChar);
    batteryService.addCharacteristic(batteryChargingChar);
    BLE.addService(batteryService);

    // Initial battery level reading
    uint8_t batteryLevel = readBatteryLevel();
    batteryLevelChar.writeValue(batteryLevel);

    // Initial battery charging status
    bool isCharging = battery.IsChargingBattery();
    batteryChargingChar.writeValue(isCharging);

    // Setup Microphone Service
    microphoneControlChar.addDescriptor(micControlDesc);
    BLE.setAdvertisedServiceUuid(MICROPHONE_SERVICE_UUID);
    BLE.addService(microphoneService);

    // Set initial values with explanation
    microphoneControlChar.writeValue(0);  // 0 = disabled by default

    // Start advertising
    BLE.advertise();

    Serial.println("BLE advertising started");
  }

  // Set Default Time
  setTime(systemBaseTime / 1000);

  Serial.println("Setup complete - entering main loop");
  Serial.println("IMU reading rate: 50Hz (20ms)");
  Serial.println("BLE transmission rate: 10Hz (100ms)");
  Serial.println("Serial output rate (IMU and Mic): 2Hz (500ms)");
  Serial.println("Microphone update rate: 10Hz (100ms)");
  Serial.println("====================================");
}

void loop() {
  // Start measuring loop time
  loopStartTime = micros();

  // Handle error states with non-blocking blink patterns
  if (!imuInitialized || !bleInitialized) {
    handleErrorState();
    // Calculate and update loop execution time
    loopTime = micros() - loopStartTime;
    if (loopTime > maxLoopTime) {
      maxLoopTime = loopTime;
    }
    return;  // Skip the rest of the loop if we're in an error state
  }

  // Always read IMU at the high frequency to capture motion accurately
  if (millis() - lastImuMicRead >= readImuMicInterval) {
    lastImuMicRead = millis();
    readImuData();
    readMicData();
  }

  if (millis() - lastBatteryRead >= batteryReadInterval) {
    lastBatteryRead = millis();
    readBatteryLevel();  // Just read and store for average
  }

  // BLE functionality - only if BLE is initialized and at a reduced rate
  if (bleInitialized) {
    BLEDevice central = BLE.central();

    // If a central device is connected
    if (central) {
      static bool previouslyConnected = false;

      if (!previouslyConnected) {
        Serial.print("Connected to central: ");
        Serial.println(central.address());
        digitalWrite(LED_BUILTIN, HIGH);  // Turn on LED when connected
        previouslyConnected = true;
      }

      // While the central is still connected
      if (central.connected()) {
        // Check for time sync updates from client
        if (timeSyncCharacteristic.written()) {


          // 獲取客戶端時間戳
          const uint8_t* timeBytes = timeSyncCharacteristic.value();
          uint64_t clientTimestamp = 0;

          for (int i = 0; i < 8; i++) {
            clientTimestamp |= ((uint64_t)timeBytes[i] << (i * 8));
          }

          systemBaseTime = clientTimestamp;

          // Record the exact millis() value at sync time
          syncReceivedMillis = millis();

          setTime(systemBaseTime / 1000);

          timeIsSynced = true;

          Serial.println("Time Synchronization Details:");
          Serial.print("Client Timestamp: ");
          Serial.println(clientTimestamp);
          Serial.print("System Base Time: ");
          Serial.println(systemBaseTime);
          Serial.print("Sync Received at millis(): ");
          Serial.println(syncReceivedMillis);

          // Provide visual feedback
          scheduleBlink(2);
        }

        // Check if microphone control characteristic was written
        if (microphoneControlChar.written()) {
          byte controlValue = microphoneControlChar.value();
          micStreamingEnabled = (controlValue == 1);

          // If enabling streaming, reset and start recording
          if (micStreamingEnabled) {
            recording = 1;
          }

          Serial.print("Microphone streaming ");
          Serial.println(micStreamingEnabled ? "enabled" : "disabled");

          // Visual feedback

          scheduleBlink(1);
        }

        // Check if loop time reset characteristic was written
        if (loopTimeResetChar.written()) {
          bool resetValue = loopTimeResetChar.value();
          if (resetValue) {
            // Reset the max loop time
            maxLoopTime = 0;
            // Reset the characteristic value
            loopTimeResetChar.writeValue(false);

            Serial.println("Maximum loop time reset via BLE");

            // Provide visual feedback

            scheduleBlink(1);
          }
        }

        // Transmit IMU and MIC data at reduced rate
        if (millis() - lastBleTransmit >= bleTransmitInterval) {
          lastBleTransmit = millis();
          transmitCombinedDataOverBLE();
          digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  // Toggle LED on data send
        }

        // Update battery data
        if (millis() - lastBatteryUpdate >= batteryUpdateInterval) {
          lastBatteryUpdate = millis();
          uint8_t batteryLevel = readBatteryLevel();
          batteryLevelChar.writeValue(batteryLevel);
          bool isCharging = battery.IsChargingBattery();
          batteryChargingChar.writeValue(isCharging);
        }

        // Update loop time via BLE periodically (every 1 second)
        if (bleInitialized && millis() - lastLoopTimeUpdate >= 1000) {
          lastLoopTimeUpdate = millis();

          // Create a single value that max loop time
          loopTimeChar.writeValue((uint32_t)maxLoopTime);

          // Also send current values via Serial if not connected to BLE
          if (!BLE.connected()) {
            Serial.print("Loop time: ");
            Serial.print(loopTime);
            Serial.print(" µs, Max: ");
            Serial.print(maxLoopTime);
            Serial.println(" µs");
          }
        }

      } else {
        // When the central disconnects
        digitalWrite(LED_BUILTIN, LOW);  // Turn off LED
        Serial.print("Disconnected from central: ");
        Serial.println(central.address());
        previouslyConnected = false;
        // When disconnected, disable microphone streaming to save power
        micStreamingEnabled = false;
      }
    } else {
      // If not connected to BLE, output to Serial at a reduced rate
      if (millis() - lastSerialTransmit >= serialTransmitInterval) {
        lastSerialTransmit = millis();
        transmitCombinedDataOverSerial();  // Update IMU and MIC Data
      }

      // If not connected, blink LED slowly
      if (millis() - blinkTimer >= 2000) {
        blinkTimer = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      }
      // When not connected, disable microphone streaming to save power
      micStreamingEnabled = false;
    }
  } else {

    // If BLE isn't available, always output to Serial at the reduced rate
    if (millis() - lastSerialTransmit >= serialTransmitInterval) {
      lastSerialTransmit = millis();
      transmitCombinedDataOverSerial();  // Update IMU and MIC Data
    }
  }

  handleBlink();
  processSerialCommands();
  // Calculate and update loop execution time
  loopTime = micros() - loopStartTime;
  if (loopTime > maxLoopTime) {
    maxLoopTime = loopTime;
  }
}

// Function to get unique device ID from hardware
uint64_t getUniqueDeviceID() {
  // Combine the two 32-bit unique device identifiers
  uint64_t uniqueID = 0;

  // Read unique device identifier registers
  // For nRF52840, the unique ID is stored in FICR (Factory Information Configuration Registers)
  uint32_t deviceIDLow = NRF_FICR->DEVICEID[0];
  uint32_t deviceIDHigh = NRF_FICR->DEVICEID[1];

  // Combine the two 32-bit values into a 64-bit unique ID
  uniqueID = ((uint64_t)deviceIDHigh << 32) | deviceIDLow;

  return uniqueID;
}

// Function to convert unique ID to a more readable string
String formatDeviceID(uint64_t uniqueID) {
  char buffer[20];

  // Format as hexadecimal
  snprintf(buffer, sizeof(buffer), "%08X%08X",
           (uint32_t)(uniqueID >> 32),        // High 32 bits
           (uint32_t)(uniqueID & 0xFFFFFFFF)  // Low 32 bits
  );

  return String(buffer);
}

uint16_t getShortDeviceID(uint64_t fullID) {
  return (uint16_t)(fullID & 0xFFFF);
}

String formatShortDeviceID(uint16_t shortID) {
  char buffer[5];
  snprintf(buffer, sizeof(buffer), "%04X", shortID);
  return String(buffer);
}

// Function to convert timestamp to formatted string
String formatTimestamp(uint64_t timestamp) {
  // Convert milliseconds since epoch to time_t (seconds since epoch)
  time_t rawTime = timestamp / 1000;

  // Use TimeLib to break down the timestamp
  tmElements_t tm;
  breakTime(rawTime, tm);

  // Custom formatting to match: YYYY-MM-DD HH:MM:SS.mmm
  char buffer[22];
  snprintf(buffer, sizeof(buffer),
           //"%04d-%02d-%02d %02d:%02d:%02d.%03d",
           //tm.Year + 1970,  // Years since 1970
           //tm.Month,        // Month (1-12)
           //tm.Day,          // Day of month (1-31)
           "%02d:%02d:%02d.%03d",
           tm.Hour,                 // Hours (0-23)
           tm.Minute,               // Minutes (0-59)
           tm.Second,               // Seconds (0-59)
           (int)(timestamp % 1000)  // Milliseconds
  );

  return String(buffer);
}

// Get synced timestamp
uint64_t getSyncedTime() {
  // Only add the milliseconds that have passed SINCE the sync event
  return systemBaseTime + ((uint64_t)millis() - syncReceivedMillis);
}

void printVersionInfo() {
  version_info_t version = getVersionInfo();

  Serial.println("\n====== VERSION INFO ======");
  Serial.print("Version: ");
  Serial.print(version.major);
  Serial.print(".");
  Serial.print(version.minor);
  Serial.print(".");
  Serial.println(version.patch);

  if (version.gitAvailable) {
    Serial.print("Git Tag: ");
    Serial.println(version.gitTag);
    Serial.print("Git Commit: ");
    Serial.println(version.gitCommit);
  } else {
    Serial.println("(Static version - Git not available)");
  }

  Serial.print("Build: ");
  Serial.println(version.buildTimestamp);
  Serial.println("==========================\n");
}

void updateVersionInfo() {
  if (!bleInitialized) return;

  version_info_t versionInfo = getVersionInfo();

  char versionStr[32];
  snprintf(versionStr, sizeof(versionStr), "%d.%d.%d (%s)",
           versionInfo.major,
           versionInfo.minor,
           versionInfo.patch,
           versionInfo.gitCommit);

  versionInfoChar.writeValue(versionStr);

  Serial.print("BLE版本信息已更新: ");
  Serial.println(versionStr);
}

// Non-blocking error state handler
void handleErrorState() {
  // Different blink patterns for different error states
  int blinkInterval;

  if (!imuInitialized && !bleInitialized) {
    blinkInterval = 200;  // Fast blinking for both errors
  } else if (!imuInitialized) {
    blinkInterval = 500;  // Medium blinking for IMU error
  } else {
    blinkInterval = 1000;  // Slow blinking for BLE error
  }

  if (millis() - errorBlinkTimer >= blinkInterval) {
    errorBlinkTimer = millis();
    errorBlinkState = !errorBlinkState;
    digitalWrite(LED_BUILTIN, errorBlinkState);
  }
}

void readImuData() {
  // Skip if IMU isn't initialized
  if (!imuInitialized) return;

  // Read accelerometer data
  imuBuffer[sharedBufferIndex].accelX = imu.readFloatAccelX();
  imuBuffer[sharedBufferIndex].accelY = imu.readFloatAccelY();
  imuBuffer[sharedBufferIndex].accelZ = imu.readFloatAccelZ();

  // Read gyroscope data
  imuBuffer[sharedBufferIndex].gyroX = imu.readFloatGyroX();
  imuBuffer[sharedBufferIndex].gyroY = imu.readFloatGyroY();
  imuBuffer[sharedBufferIndex].gyroZ = imu.readFloatGyroZ();

  // Store synced timestamp and device ID
  imuBuffer[sharedBufferIndex].timestamp = getSyncedTime();
  imuBuffer[sharedBufferIndex].deviceId = DEVICE_ID;


  // Update buffer index
  sharedBufferIndex = (sharedBufferIndex + 1) % BUFFER_SIZE;
  if (sharedBufferIndex == 0) {
    bufferFull = true;  // We've wrapped around, buffer is full
  }
}

uint8_t readBatteryLevel() {
  // Read battery voltage from VBAT_PIN
  float voltage = battery.GetBatteryVoltage();
  bool isCharging = battery.IsChargingBattery();

  // Add to history array
  batteryVoltageHistory[batteryHistoryIndex] = voltage;
  batteryHistoryIndex = (batteryHistoryIndex + 1) % BATTERY_SAMPLE_COUNT;

  // Calculate average voltage
  float avgVoltage = 0;
  int validSamples = 0;
  for (int i = 0; i < BATTERY_SAMPLE_COUNT; i++) {
    if (batteryVoltageHistory[i] > 0) {
      avgVoltage += batteryVoltageHistory[i];
      validSamples++;
    }
  }

  if (validSamples > 0) {
    avgVoltage /= validSamples;
  } else {
    avgVoltage = voltage;  // Fallback if no valid history
  }

  // Adjust these values based on your battery specifications
  // For LiPo 3.7V: ~3.0V (empty) to ~4.2V (full)
  int percent = map((int)(avgVoltage * 100), 300, 420, 0, 100);
  percent = constrain(percent, 0, 100);
  if (batteryHistoryIndex == 9) {
    Serial.print("Battery: Raw Voltage = ");
    Serial.print(voltage, 3);
    Serial.print("V, Avg Voltage = ");
    Serial.print(avgVoltage, 3);
    Serial.print("V, Charging = ");
    Serial.print(isCharging ? "Yes" : "No");
    Serial.print(", Percent = ");
    Serial.print(percent);
    Serial.println("%");
  }

  return percent;
}

// 透過 Serial 命令控制麥克風功能
void processSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    // Change all string to lower case
    command.toLowerCase();

    if (command == "mic on" || command == "micon") {
      // Set Mic On
      micStreamingEnabled = true;
      Serial.println("Microphone recording enabled");
    } else if (command == "mic off" || command == "micoff") {
      // Set Mic Off
      micStreamingEnabled = false;
      Serial.println("Microphone recording disabled");
    } else if (command == "mic status" || command == "micstatus") {
      // Report Mic Status
      Serial.print("Microphone recording: ");
      Serial.println(micStreamingEnabled ? "enabled" : "disabled");
    } else if (command == "loop time" || command == "looptime") {
      processLoopTimeCommand(command);
    } else if (command == "reset loop time" || command == "resetlooptime") {
      processLoopTimeCommand(command);
    } else if (command == "help") {
      // Show Available Serial Commands
      Serial.println("Available commands:");
      Serial.println("  mic on - Enable microphone recording");
      Serial.println("  mic off - Disable microphone recording");
      Serial.println("  mic status - Display microphone status");
      Serial.println("  loop time - Display current and maximum loop time");
      Serial.println("  reset loop time - Reset maximum loop time");
      Serial.println("  help - Display this help message");
    }
  }
}

// Initialize microphone (using new mic.h implementation)
bool initializeMicrophone() {
  // Set the callback function
  Mic.set_callback(audio_rec_callback);
  // Initialize the microphone
  if (!Mic.begin()) {
    Serial.println("Failed to initialize microphone!");
    return false;
  }

  Serial.println("Microphone initialized successfully");
  return true;
}

// Microphone callback function for new mic.h implementation
static void audio_rec_callback(uint16_t* buf, uint32_t buf_len) {
  static uint32_t idx = 0;
  for (uint32_t i = 0; i < buf_len; i++) {
    recording_buf[idx++] = buf[i];

    if (idx >= SAMPLES) {
      idx = 0;
      recording = 0;
      record_ready = true;
      break;
    }
  }
}

void readMicData() {
  // Skip if no data available
  if (!record_ready) return;

  // Calculate sound level (simple absolute average of samples)
  int32_t sumLevel = 0;
  int16_t maxSample = 0;

  for (int i = 0; i < SAMPLES; i++) {
    int16_t sample = abs(recording_buf[i]);
    sumLevel += sample;

    // Track maximum level
    if (sample > maxSample) {
      maxSample = sample;
    }
  }

  // Calculate average level
  microphoneLevel = sumLevel / SAMPLES;

  // Update peak if needed
  if (maxSample > microphonePeak) {
    microphonePeak = maxSample;
  }

  // Reset peak periodically
  if (millis() - lastLevelReset >= levelResetInterval) {
    lastLevelReset = millis();
    microphonePeak = 0;
  }
}

void transmitCombinedDataOverBLE() {
  uint8_t dummydata[4] = { 0x00 };

  // Skip if no data or BLE not available
  if (!imuInitialized || !bleInitialized || !BLE.connected()) return;

  // For BLE, just send the most recent reading to save bandwidth
  int latestIndex = (sharedBufferIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE;

  // Convert to byte array for BLE transmission
  uint8_t combinedData[BLE_DATA_SIZE];  // 8(timestamp) + 2(device ID) + 24(IMU) + 4(MIC) = 38 bytes total

  // Use pre-stored global DEVICE_ID
  uint16_t shortID = DEVICE_ID;

  // Copy timestamp (64-bit) to byte array
  uint64_t timestamp = imuBuffer[latestIndex].timestamp;
  for (int i = 0; i < 8; i++) {
    combinedData[i] = (timestamp >> (i * 8)) & 0xFF;
  }

  // Copy device ID (64-bit) to byte array
  for (int i = 0; i < 2; i++) {
    combinedData[8 + i] = (shortID >> (i * 8)) & 0xFF;
  }

  // Copy IMU float values to byte array
  memcpy(&combinedData[10], &imuBuffer[latestIndex].accelX, sizeof(float));
  memcpy(&combinedData[14], &imuBuffer[latestIndex].accelY, sizeof(float));
  memcpy(&combinedData[18], &imuBuffer[latestIndex].accelZ, sizeof(float));
  memcpy(&combinedData[22], &imuBuffer[latestIndex].gyroX, sizeof(float));
  memcpy(&combinedData[26], &imuBuffer[latestIndex].gyroY, sizeof(float));
  memcpy(&combinedData[30], &imuBuffer[latestIndex].gyroZ, sizeof(float));

  // Copy level data
  if (record_ready) {
    // Read the data if not already done
    readMicData();
    memcpy(&combinedData[34], &microphoneLevel, sizeof(int16_t));
    memcpy(&combinedData[36], &microphonePeak, sizeof(int16_t));
    record_ready = false;
    recording = 1;
  } else {
    memcpy(&combinedData[34], dummydata, sizeof(dummydata));
  }

  // Update BLE characteristic value
  imuCharacteristic.writeValue(combinedData, sizeof(combinedData));
}

void transmitCombinedDataOverSerial() {
  // 如果藍牙已連接或沒有可用資料，則跳過序列輸出
  if (BLE.connected()) return;

  // 確保 IMU 已初始化
  if (!imuInitialized) return;

  // 獲取 IMU 資料
  int latestIndex = (sharedBufferIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE;

  // Get format timestamp
  uint64_t currentTimestamp = imuBuffer[latestIndex].timestamp;
  String formattedTimestamp = formatTimestamp(currentTimestamp);

  // CSV format output: DeviceID,Timestamp,IMU,AccelX,Y,Z,GyroX,Y,Z
  Serial.print("IMU,");
  Serial.print(formattedTimestamp);
  Serial.print(",");
  Serial.print(imuBuffer[latestIndex].accelX, 4);
  Serial.print(",");
  Serial.print(imuBuffer[latestIndex].accelY, 4);
  Serial.print(",");
  Serial.print(imuBuffer[latestIndex].accelZ, 4);
  Serial.print(",");
  Serial.print(imuBuffer[latestIndex].gyroX, 4);
  Serial.print(",");
  Serial.print(imuBuffer[latestIndex].gyroY, 4);
  Serial.print(",");
  Serial.print(imuBuffer[latestIndex].gyroZ, 4);

  // 只有在麥克風數據準備好時才添加麥克風部分
  if (record_ready) {
    // 讀取麥克風資料
    readMicData();

    Serial.print(",MIC,");
    Serial.print(microphoneLevel);
    Serial.print(",");
    Serial.print(microphonePeak);

    // 標記麥克風資料已處理並重新開始錄音
    record_ready = false;
    recording = 1;
  } else {
    // 如果麥克風數據未準備好，輸出空值
    Serial.print(",MIC,0,0");
  }

  Serial.println();
}

void processLoopTimeCommand(String command) {
  if (command == "loop time" || command == "looptime") {
    Serial.print("Current loop time: ");
    Serial.print(loopTime);
    Serial.print(" µs, Maximum loop time: ");
    Serial.print(maxLoopTime);
    Serial.println(" µs");
  } else if (command == "reset loop time" || command == "resetlooptime") {
    maxLoopTime = 0;
    Serial.println("Maximum loop time reset to 0");
  }
}

void scheduleBlink(int blinks) {
  digitalWrite(LED_BUILTIN, LOW);
  blinkCount = blinks * 2;  // 2 次算 1 次閃（HIGH+LOW）
  lastToggle = millis();
  ledState = LOW;
}

void handleBlink() {
  if (blinkCount > 0 && millis() - lastToggle >= 50) {
    lastToggle = millis();
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    blinkCount--;
  }
  if (blinkCount == 0) digitalWrite(LED_BUILTIN, HIGH);
}
