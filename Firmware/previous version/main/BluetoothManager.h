#ifndef BLUETOOTH_MANAGER_H
#define BLUETOOTH_MANAGER_H

#include <ArduinoBLE.h>
#include "SensorManager.h"

// Custom service and characteristic UUIDs
#define HEALTH_SERVICE_UUID           "180D" // Heart Rate Service
#define SENSOR_DATA_CHAR_UUID         "2A37" // Heart Rate Measurement
#define TEMPERATURE_CHAR_UUID         "2A1C" // Temperature Measurement  
#define IMU_DATA_CHAR_UUID            "2A19" // Battery Level (repurposed for IMU)
#define DEVICE_INFO_CHAR_UUID         "2A29" // Device Info

class BluetoothManager {
private:
  BLEService healthService;
  BLECharacteristic sensorDataChar;
  BLECharacteristic temperatureChar;
  BLECharacteristic imuDataChar;
  BLECharacteristic deviceInfoChar;
  
  bool bleInitialized;
  bool deviceConnected;
  unsigned long lastConnectionCheck;
  
  // Data formatting helpers
  void formatSensorData(const SensorData& data, uint8_t* buffer, int& length);
  void formatTemperatureData(float temp_f, uint8_t* buffer, int& length);
  void formatIMUData(float ax, float ay, float az, float gx, float gy, float gz, uint8_t* buffer, int& length);
  
public:
  BluetoothManager();
  bool init();
  void handleConnections();
  void sendSensorData(const SensorData& data);
  bool isConnected() { return deviceConnected; }
};

// Implementation
BluetoothManager::BluetoothManager() : 
  healthService(HEALTH_SERVICE_UUID),
  sensorDataChar(SENSOR_DATA_CHAR_UUID, BLERead | BLENotify, 20),
  temperatureChar(TEMPERATURE_CHAR_UUID, BLERead | BLENotify, 8),
  imuDataChar(IMU_DATA_CHAR_UUID, BLERead | BLENotify, 24),
  deviceInfoChar(DEVICE_INFO_CHAR_UUID, BLERead, 20)
{
  bleInitialized = false;
  deviceConnected = false;
  lastConnectionCheck = 0;
}

bool BluetoothManager::init() {
  Serial.println("Initializing Bluetooth...");
  
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    return false;
  }
  
  // Set device name
  BLE.setLocalName("HealthMonitor");
  BLE.setDeviceName("Multi-Sensor Health Monitor");
  
  // Add characteristics to service
  healthService.addCharacteristic(sensorDataChar);
  healthService.addCharacteristic(temperatureChar);
  healthService.addCharacteristic(imuDataChar);
  healthService.addCharacteristic(deviceInfoChar);
  
  // Add service
  BLE.addService(healthService);
  
  // Set device info
  String deviceInfo = "nRF52840-HealthMon-v1.0";
  deviceInfoChar.writeValue(deviceInfo.c_str());
  
  // Set advertising data
  BLE.setAdvertisedService(healthService);
  
  // Start advertising
  BLE.advertise();
  
  Serial.println("✓ Bluetooth initialized and advertising");
  Serial.println("Device name: Multi-Sensor Health Monitor");
  
  bleInitialized = true;
  return true;
}

void BluetoothManager::handleConnections() {
  if (!bleInitialized) return;
  
  // Check connection status every 500ms
  if (millis() - lastConnectionCheck > 500) {
    BLEDevice central = BLE.central();
    bool wasConnected = deviceConnected;
    deviceConnected = central && central.connected();
    
    if (deviceConnected && !wasConnected) {
      Serial.println("✓ Bluetooth device connected");
      Serial.print("Connected to: ");
      Serial.println(central.address());
    } else if (!deviceConnected && wasConnected) {
      Serial.println("✗ Bluetooth device disconnected");
      // Restart advertising
      BLE.advertise();
    }
    
    lastConnectionCheck = millis();
  }
  
  // Poll BLE events
  BLE.poll();
}

void BluetoothManager::sendSensorData(const SensorData& data) {
  if (!bleInitialized || !deviceConnected) return;
  
  // Send heart rate and SpO2 data
  uint8_t heartRateBuffer[20];
  int heartRateLength;
  formatSensorData(data, heartRateBuffer, heartRateLength);
  sensorDataChar.writeValue(heartRateBuffer, heartRateLength);
  
  // Send temperature data
  uint8_t tempBuffer[8];
  int tempLength;
  formatTemperatureData(data.temperature_f, tempBuffer, tempLength);
  temperatureChar.writeValue(tempBuffer, tempLength);
  
  // Send IMU data
  uint8_t imuBuffer[24];
  int imuLength;
  formatIMUData(data.accel_x, data.accel_y, data.accel_z, 
                data.gyro_x, data.gyro_y, data.gyro_z, imuBuffer, imuLength);
  imuDataChar.writeValue(imuBuffer, imuLength);
}

void BluetoothManager::formatSensorData(const SensorData& data, uint8_t* buffer, int& length) {
  // Format: [flags][heart_rate_16bit][spo2_8bit][ir_value_32bit][status]
  buffer[0] = 0x01; // Flags: 16-bit heart rate value present
  
  // Heart rate (16-bit)
  uint16_t hr = data.heartRateValid ? data.heartRateAvg : 0;
  buffer[1] = hr & 0xFF;
  buffer[2] = (hr >> 8) & 0xFF;
  
  // SpO2 (8-bit, as percentage)
  buffer[3] = data.heartRateValid ? (uint8_t)data.spo2 : 0;
  
  // IR Value (32-bit) - for finger detection
  uint32_t ir = data.irValue;
  buffer[4] = ir & 0xFF;
  buffer[5] = (ir >> 8) & 0xFF;
  buffer[6] = (ir >> 16) & 0xFF;
  buffer[7] = (ir >> 24) & 0xFF;
  
  // Status flags
  buffer[8] = 0;
  if (data.heartRateValid) buffer[8] |= 0x01;
  if (data.temperatureValid) buffer[8] |= 0x02;
  if (data.imuValid) buffer[8] |= 0x04;
  
  length = 9;
}

void BluetoothManager::formatTemperatureData(float temp_f, uint8_t* buffer, int& length) {
  // Temperature in Fahrenheit as 32-bit float
  union {
    float f;
    uint8_t bytes[4];
  } temp_union;
  
  temp_union.f = temp_f;
  
  buffer[0] = temp_union.bytes[0];
  buffer[1] = temp_union.bytes[1];
  buffer[2] = temp_union.bytes[2];
  buffer[3] = temp_union.bytes[3];
  
  length = 4;
}

void BluetoothManager::formatIMUData(float ax, float ay, float az, float gx, float gy, float gz, uint8_t* buffer, int& length) {
  // Pack 6 floats (24 bytes total)
  union {
    float f;
    uint8_t bytes[4];
  } float_union;
  
  int idx = 0;
  
  // Accelerometer X, Y, Z
  float_union.f = ax;
  for (int i = 0; i < 4; i++) buffer[idx++] = float_union.bytes[i];
  
  float_union.f = ay;
  for (int i = 0; i < 4; i++) buffer[idx++] = float_union.bytes[i];
  
  float_union.f = az;
  for (int i = 0; i < 4; i++) buffer[idx++] = float_union.bytes[i];
  
  // Gyroscope X, Y, Z
  float_union.f = gx;
  for (int i = 0; i < 4; i++) buffer[idx++] = float_union.bytes[i];
  
  float_union.f = gy;
  for (int i = 0; i < 4; i++) buffer[idx++] = float_union.bytes[i];
  
  float_union.f = gz;
  for (int i = 0; i < 4; i++) buffer[idx++] = float_union.bytes[i];
  
  length = 24;
}

#endif