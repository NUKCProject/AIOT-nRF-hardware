# aiot-hardware-imu

# Firmware Basic information
- Serila port Speed : 500kbps
- UUID : 14A168D7-04D1-6C4F-7E53-F2E800B11900
- Sytem Read IMU : 20ms
- System Read Battery ADC : 1000ms
- BLE IMU Transmit : 100ms
- BLE Battery Transmit : 30s
- Serial IMU Transmit : 200ms
- Serial Battery Transmit : 1s

# Copy libary to your Arduino libaraies
```
user-path\Arduino\libraries
```

Navigate to **File > Preferences**, and fill **"Additional Boards Manager URLs"** with the url below:

[*https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json*](https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json)

Arduino Configuration

- Board Configuration [ **Tools > Board > Boards Manager..]**
    - **Selected  Seeed nRF52 mebd-enabled Boards**
        - **Search seeed nrf52**
        
- Arduino Libraries  [Tools > Manage Libraries…]
    - Install ArduinoBLE

    - Install Seeed Arduino LSM6DS3
	
	- Install Timelib
	
	- Install Seed Aruino Mic (From GitHub)
		- Download zip file Library  https://github.com/Seeed-Studio/Seeed_Arduino_Mic
		- Add .ZIP Library **Seeed_Arduino_Mic-master.zip**
        
    - Install Xiao NRF52840 Battery
        
        - The head file have to modified as below
            1. Remove `bluefruit.h`
            2. Add Pin define 
                1. #define VBAT_ENABLE P0_14
                2. #define PIN_VBAT P0_31
            
            Example as below
            
        
        ```arduino
        #include <Arduino.h>
        // Removed 
        #define VBAT_ENABLE P0_14    // 
        #define PIN_VBAT P0_31
        
        #define BAT_HIGH_CHARGE 22  // HIGH for 50mA, LOW for 100mA
        #define BAT_CHARGE_STATE 23 // LOW for charging, HIGH not charging
        
        class Xiao {
        public:
          Xiao();
          float GetBatteryVoltage();
          bool IsChargingBattery();
        };
        
        Xiao::Xiao() {
          pinMode(VBAT_ENABLE, OUTPUT);
          pinMode(BAT_CHARGE_STATE, INPUT);
        
          digitalWrite(BAT_HIGH_CHARGE, HIGH); // charge with 50mA
        }
        
        #define VBAT_MV_PER_LBS (0.003395996F)
        
        float Xiao::GetBatteryVoltage() {
          digitalWrite(VBAT_ENABLE, LOW);
        
          uint32_t adcCount = analogRead(PIN_VBAT);
          float adcVoltage = adcCount * VBAT_MV_PER_LBS;
          float vBat = adcVoltage * (1510.0 / 510.0);
        
          digitalWrite(VBAT_ENABLE, HIGH);
        
          return vBat;
        }
        
        bool Xiao::IsChargingBattery() { return digitalRead(BAT_CHARGE_STATE) == LOW; }
        ```
        
 
