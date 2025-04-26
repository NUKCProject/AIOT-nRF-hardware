/*
 * Badminton Motion Tracking - XIAO nRF52840 Sense
 * Optimized data transmission rates for BLE and Serial
 * Added time synchronization feature
 */

#include <Arduino.h>
#include <Wire.h>
#include <LSM6DS3.h>
#include <ArduinoBLE.h>
#include <xiaobattery.h>

// Create IMU instance
LSM6DS3 imu(I2C_MODE, 0x6A);  // I2C address = 0x6A

// Create Battery monitoring
Xiao battery;

// Define UUIDs for the BLE service and characteristic
#define IMU_SERVICE_UUID        "14A168D7-04D1-6C4F-7E53-F2E800B11900"
#define IMU_CHARACTERISTIC_UUID "14A168D7-04D1-6C4F-7E53-F2E801B11900"
#define BATTERY_CHAR_UUID       "2A19"  // Standard BLE battery characteristic

// Time sync characteristic UUID
#define TIME_SYNC_CHAR_UUID     "14A168D7-04D1-6C4F-7E53-F2E802B11900"

// BLE objects
BLEService imuService(IMU_SERVICE_UUID);
BLECharacteristic imuCharacteristic(IMU_CHARACTERISTIC_UUID, 
                                  BLERead | BLENotify, 
                                  32); // Timestamp 8 bytes + 6 float values (4 bytes each) = 32 bytes

// Time sync characteristic - allows client to write timestamp
// Using 8 bytes for timestamp to accommodate Unix epoch timestamps or other large values
BLECharacteristic timeSyncCharacteristic(TIME_SYNC_CHAR_UUID,
                                  BLERead | BLEWrite, 8); // 8 bytes for 64-bit timestamp

// Battery service
BLEService batteryService("180F"); // Standard battery service
BLEUnsignedCharCharacteristic batteryLevelChar(BATTERY_CHAR_UUID, BLERead | BLENotify);

// Timer variables
unsigned long lastImuRead = 0;
unsigned long lastBleTransmit = 0;
unsigned long lastSerialTransmit = 0;
unsigned long lastBatteryUpdate = 0;
unsigned long blinkTimer = 0;

// Time sync variables
int64_t deviceTimeOffset = 0;  // Using 64-bit integer for larger time values
unsigned long lastSyncTime = 0;
bool timeIsSynced = false;

// Default timestamp: April 14, 2025, 00:00:00 UTC in milliseconds since epoch
// (1744848000000 milliseconds since January 1, 1970)
const uint64_t DEFAULT_TIMESTAMP = 1744848000000ULL;

// Different intervals for different operations
const int imuReadInterval = 20;       // 20ms = 50Hz for reading IMU (capture motion accurately)
const int bleTransmitInterval = 100;  // 100ms = 10Hz for BLE transmission (reduced from 50Hz)
const int serialTransmitInterval = 200; // 200ms = 5Hz for Serial output (further reduced)
const int batteryUpdateInterval = 10000; // 10s battery update rate

// Buffer for collecting multiple IMU readings before sending
#define BUFFER_SIZE 5  // Store 5 readings (at 20ms interval = 100ms of data)
typedef struct {
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    uint64_t timestamp;  // Using 64-bit timestamp
} imu_reading_t;

imu_reading_t imuBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFull = false;

// State variables to avoid blocking loops
bool imuInitialized = false;
bool bleInitialized = false;
int errorBlinkState = 0;
unsigned long errorBlinkTimer = 0;

// Get synced timestamp
uint64_t getSyncedTime() {
  if (timeIsSynced) {
    return (uint64_t)millis() + deviceTimeOffset;
  } else {
    // If time hasn't been synced yet, return default time + milliseconds since boot
    return DEFAULT_TIMESTAMP + millis();
  }
}

void setup() {
  // Initialize LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);  // Brief flash only
  digitalWrite(LED_BUILTIN, LOW);
  
  // Initialize Serial for debugging - shorter delay
  Serial.begin(115200);  // Higher baud rate for faster serial
  delay(100);  // Reduced delay
  
  Serial.println("\n\n====================================");
  Serial.println("Badminton Motion Tracking - XIAO nRF52840");
  Serial.println("Optimized data transmission rates");
  Serial.println("Added time synchronization feature");
  Serial.println("====================================");

  // Print default timestamp
  Serial.println("Default timestamp set to: April 14, 2025 00:00:00 UTC");
  Serial.print("In milliseconds since epoch: ");
  // Print high and low 32 bits separately
  Serial.print((uint32_t)(DEFAULT_TIMESTAMP >> 32), HEX);
  Serial.print("-");
  Serial.println((uint32_t)DEFAULT_TIMESTAMP, HEX);

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
    BLE.setLocalName("Badminton Tracker");
    BLE.setAdvertisedService(imuService);
    
    // Add IMU characteristic to the service
    imuService.addCharacteristic(imuCharacteristic);
    
    // Add Time Sync characteristic to the IMU service
    imuService.addCharacteristic(timeSyncCharacteristic);
    
    // Set initial value for time sync characteristic
    uint8_t initValue[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    timeSyncCharacteristic.writeValue(initValue, 8);
    
    // Add the service
    BLE.addService(imuService);
    
    // Setup Battery Service
    batteryService.addCharacteristic(batteryLevelChar);
    BLE.addService(batteryService);
    
    // Initial battery level reading
    uint8_t batteryLevel = readBatteryLevel();
    batteryLevelChar.writeValue(batteryLevel);
    
    // Start advertising
    BLE.advertise();
    
    Serial.println("BLE advertising started");
  }
  
  Serial.println("Setup complete - entering main loop");
  Serial.println("IMU reading rate: 50Hz (20ms)");
  Serial.println("BLE transmission rate: 10Hz (100ms)");
  Serial.println("Serial output rate: 5Hz (200ms)");
  Serial.println("====================================");
}

void loop() {
  // Handle error states with non-blocking blink patterns
  if (!imuInitialized || !bleInitialized) {
    handleErrorState();
    return; // Skip the rest of the loop if we're in an error state
  }
  
  // Always read IMU at the high frequency to capture motion accurately
  if (millis() - lastImuRead >= imuReadInterval) {
    lastImuRead = millis();
    readImuData();  // Just read and store, don't transmit
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
        digitalWrite(LED_BUILTIN, HIGH); // Turn on LED when connected
        previouslyConnected = true;
      }
      
      // While the central is still connected
      if (central.connected()) {
        // Check for time sync updates from client
        if (timeSyncCharacteristic.written()) {
          // Get the byte array from the characteristic
          const uint8_t* timeBytes = timeSyncCharacteristic.value();
          unsigned long clientTimestamp = 0;
          
          // Convert byte array to unsigned long (assuming little endian format)
          for (int i = 0; i < 4; i++) {
            clientTimestamp |= (unsigned long)timeBytes[i] << (i * 8);
          }
          
          unsigned long localTime = millis();
          
          // Calculate offset between device time and client time
          deviceTimeOffset = clientTimestamp - localTime;
          timeIsSynced = true;
          lastSyncTime = localTime;
          
          Serial.print("Time synchronized. Offset: ");
          Serial.print(deviceTimeOffset);
          Serial.println(" ms");
          
          // Flash LED rapidly twice to indicate successful sync
          for (int i = 0; i < 2; i++) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(50);
            digitalWrite(LED_BUILTIN, LOW);
            delay(50);
          }
          digitalWrite(LED_BUILTIN, HIGH); // Restore LED state
        }
        
        // Transmit IMU data at reduced rate
        if (millis() - lastBleTransmit >= bleTransmitInterval) {
          lastBleTransmit = millis();
          transmitImuDataOverBLE();
          digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle LED on data send
        }
        
        // Update battery data
        if (millis() - lastBatteryUpdate >= batteryUpdateInterval) {
          lastBatteryUpdate = millis();
          uint8_t batteryLevel = readBatteryLevel();
          batteryLevelChar.writeValue(batteryLevel);
          Serial.print("Battery level: ");
          Serial.print(batteryLevel);
          Serial.println("%");
          
          // Also send time sync status
          if (timeIsSynced) {
            Serial.print("Time sync active. Current device time: ");
            Serial.println(getSyncedTime());
            Serial.print("Time since last sync: ");
            Serial.print((millis() - lastSyncTime) / 1000);
            Serial.println(" seconds");
          }
        }
      } else {
        // When the central disconnects
        digitalWrite(LED_BUILTIN, LOW); // Turn off LED
        Serial.print("Disconnected from central: ");
        Serial.println(central.address());
        previouslyConnected = false;
      }
    } else {
      // If not connected to BLE, output to Serial at a reduced rate
      if (millis() - lastSerialTransmit >= serialTransmitInterval) {
        lastSerialTransmit = millis();
        transmitImuDataOverSerial();
      }
      
      // If not connected, blink LED slowly
      if (millis() - blinkTimer >= 2000) {
        blinkTimer = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      }
    }
  } else {
    // If BLE isn't available, always output to Serial at the reduced rate
    if (millis() - lastSerialTransmit >= serialTransmitInterval) {
      lastSerialTransmit = millis();
      transmitImuDataOverSerial();
    }
  }
}

// Non-blocking error state handler
void handleErrorState() {
  // Different blink patterns for different error states
  int blinkInterval;
  
  if (!imuInitialized && !bleInitialized) {
    blinkInterval = 200; // Fast blinking for both errors
  } else if (!imuInitialized) {
    blinkInterval = 500; // Medium blinking for IMU error
  } else {
    blinkInterval = 1000; // Slow blinking for BLE error
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
  imuBuffer[bufferIndex].accelX = imu.readFloatAccelX();
  imuBuffer[bufferIndex].accelY = imu.readFloatAccelY();
  imuBuffer[bufferIndex].accelZ = imu.readFloatAccelZ();
  
  // Read gyroscope data
  imuBuffer[bufferIndex].gyroX = imu.readFloatGyroX();
  imuBuffer[bufferIndex].gyroY = imu.readFloatGyroY();
  imuBuffer[bufferIndex].gyroZ = imu.readFloatGyroZ();
  
  // Store synced timestamp instead of local time
  imuBuffer[bufferIndex].timestamp = getSyncedTime();
  
  // Update buffer index
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  if (bufferIndex == 0) {
    bufferFull = true; // We've wrapped around, buffer is full
  }
}

void transmitImuDataOverBLE() {
  // Skip if no data or BLE not available
  if (!imuInitialized || !bleInitialized || !BLE.connected()) return;
  
  // For BLE, just send the most recent reading to save bandwidth
  int latestIndex = (bufferIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE;
  
  // Convert to byte array for BLE transmission
  uint8_t imuData[32]; // timestamp 8 bytes + 6 floats × 4 bytes = 32 bytes
  
  // Copy timestamp (64-bit) to byte array
  uint64_t timestamp = imuBuffer[latestIndex].timestamp;
  for (int i = 0; i < 8; i++) {
    imuData[i] = (timestamp >> (i * 8)) & 0xFF;
  }
  
  // Copy float values to byte array
  memcpy(&imuData[8], &imuBuffer[latestIndex].accelX, sizeof(float));
  memcpy(&imuData[12], &imuBuffer[latestIndex].accelY, sizeof(float));
  memcpy(&imuData[16], &imuBuffer[latestIndex].accelZ, sizeof(float));
  memcpy(&imuData[20], &imuBuffer[latestIndex].gyroX, sizeof(float));
  memcpy(&imuData[24], &imuBuffer[latestIndex].gyroY, sizeof(float));
  memcpy(&imuData[28], &imuBuffer[latestIndex].gyroZ, sizeof(float));
  
  // Update BLE characteristic value
  imuCharacteristic.writeValue(imuData, sizeof(imuData));
}

void transmitImuDataOverSerial() {
  // Skip if no data
  if (!imuInitialized) return;
  
  // Option 1: Send only the most recent reading
  int latestIndex = (bufferIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE;
  
  Serial.print(imuBuffer[latestIndex].timestamp);
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
  Serial.println(imuBuffer[latestIndex].gyroZ, 4);
  
  /* Option 2: Send multiple readings at once (uncomment if desired)
  // Only proceed if we have a full buffer or at least some readings
  if (!bufferFull && bufferIndex == 0) return;
  
  // Calculate how many entries to send
  int numEntries = bufferFull ? BUFFER_SIZE : bufferIndex;
  
  // Send the buffer size first
  Serial.print("BATCH:");
  Serial.println(numEntries);
  
  // Send each entry
  for (int i = 0; i < numEntries; i++) {
    // Calculate the actual index, considering circular buffer
    int idx = bufferFull ? (bufferIndex + i) % BUFFER_SIZE : i;
    
    Serial.print(imuBuffer[idx].timestamp);
    Serial.print(",");
    Serial.print(imuBuffer[idx].accelX, 4);
    Serial.print(",");
    Serial.print(imuBuffer[idx].accelY, 4); 
    Serial.print(",");
    Serial.print(imuBuffer[idx].accelZ, 4);
    Serial.print(",");
    Serial.print(imuBuffer[idx].gyroX, 4);
    Serial.print(",");
    Serial.print(imuBuffer[idx].gyroY, 4);
    Serial.print(",");
    Serial.println(imuBuffer[idx].gyroZ, 4);
  }
  
  // Reset buffer state after sending
  bufferFull = false;
  bufferIndex = 0;
  */
}

uint8_t readBatteryLevel() {
  // Read battery voltage from VBAT_PIN
  float voltage = battery.GetBatteryVoltage();
  bool isCharging = battery.IsChargingBattery();

  Serial.print("Battery: Voltage = ");
  Serial.print(voltage, 3);
  Serial.print("V, Charging = ");
  Serial.print(isCharging ? "Yes" : "No");
  Serial.print(", ");

  int percent = map((int)(voltage * 100), 300, 420, 0, 100);
  percent = constrain(percent, 0, 100);
  
  Serial.print("Percent = ");
  Serial.println(percent);
  
  return percent;
}

// Write register function
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(0x6A); // LSM6DS3 address
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}