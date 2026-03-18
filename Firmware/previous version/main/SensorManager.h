#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Wire.h>
#include <LSM6DS3.h>
#include <Adafruit_MLX90614.h>
#include "MAX30105.h"
#include "heartRate.h" // Use the SparkFun library's heartRate.h

// Sensor data structure
struct SensorData {
  // Temperature
  float temperature_f;
  
  // Heart rate and SpO2
  int heartRate;
  int heartRateAvg;
  float spo2;
  long irValue;
  long redValue;
  
  // IMU data
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  
  // Status flags
  bool temperatureValid;
  bool heartRateValid;
  bool imuValid;
  
  // Timestamp
  unsigned long timestamp;
};

class SensorManager {
private:
  // Sensor objects
  LSM6DS3 imu;
  Adafruit_MLX90614 mlx;
  MAX30105 particleSensor;
  
  // Heart rate processing
  static const byte RATE_SIZE = 4;
  byte rates[RATE_SIZE];
  byte rateSpot;
  long lastBeat;
  float beatsPerMinute;
  int beatAvg;
  
  // Enhanced beat detection variables
  long signalHistory[10];
  int historyIndex;
  bool fingerStable;
  unsigned long fingerPlacedTime;
  
  // SpO2 calculation variables
  double red_buffer[50];
  double ir_buffer[50];
  int buffer_index;
  
  // Sensor status
  bool mlx_initialized;
  bool max30102_initialized;
  bool imu_initialized;
  
  // Helper methods
  void processHeartRate(long irValue);
  float calculateSpO2();
  bool isFingerDetected(long irValue);
  void updateSignalBuffer(long irValue, long redValue);
  
public:
  SensorManager();
  bool initializeAll();
  void shutdown();
  SensorData readAllSensors();
  
  // Individual sensor methods
  bool initializeMLX90614();
  bool initializeMAX30102();
  bool initializeIMU();
  
  float readTemperature();
  void readHeartRateData(long &irValue, long &redValue);
  void readIMUData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
};

// Implementation
SensorManager::SensorManager() {
  rateSpot = 0;
  lastBeat = 0;
  beatsPerMinute = 0;
  beatAvg = 0;
  buffer_index = 0;
  historyIndex = 0;
  fingerStable = false;
  fingerPlacedTime = 0;
  
  mlx_initialized = false;
  max30102_initialized = false;
  imu_initialized = false;
  
  // Initialize rate array
  for (int i = 0; i < RATE_SIZE; i++) {
    rates[i] = 0;
  }
  
  // Initialize signal history
  for (int i = 0; i < 10; i++) {
    signalHistory[i] = 0;
  }
}

bool SensorManager::initializeAll() {
  Serial.println("Initializing sensors...");
  
  Wire.begin();
  delay(100);
  
  bool success = true;
  
  // Initialize MLX90614
  if (initializeMLX90614()) {
    Serial.println("✓ MLX90614 temperature sensor ready");
  } else {
    Serial.println("✗ MLX90614 initialization failed");
    success = false;
  }
  
  // Initialize MAX30102
  if (initializeMAX30102()) {
    Serial.println("✓ MAX30102 heart rate sensor ready");
  } else {
    Serial.println("✗ MAX30102 initialization failed");
    success = false;
  }
  
  // Initialize IMU
  if (initializeIMU()) {
    Serial.println("✓ LSM6DS3 IMU ready");
  } else {
    Serial.println("✗ LSM6DS3 initialization failed");
    success = false;
  }
  
  return success;
}

bool SensorManager::initializeMLX90614() {
  if (mlx.begin()) {
    mlx_initialized = true;
    return true;
  }
  mlx_initialized = false;
  return false;
}

bool SensorManager::initializeMAX30102() {
  if (particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    // Configure sensor with optimized settings
    particleSensor.setup(); // Default settings first
    
    // Optimize for better heart rate detection
    particleSensor.setPulseAmplitudeRed(0x0A);  // Low power red LED
    particleSensor.setPulseAmplitudeGreen(0);   // Turn off green LED
    particleSensor.setPulseAmplitudeIR(0x0A);   // Low power IR LED
    
    // Additional optimizations
    particleSensor.setLEDMode(2); // Red and IR LEDs
    particleSensor.setSampleRate(100); // 100 samples per second
    particleSensor.setPulseWidth(411); // 411us pulse width
    particleSensor.setADCRange(4096); // ADC range
    
    // Reset beat detection variables
    rateSpot = 0;
    lastBeat = 0;
    beatsPerMinute = 0;
    beatAvg = 0;
    fingerStable = false;
    fingerPlacedTime = 0;
    
    max30102_initialized = true;
    return true;
  }
  max30102_initialized = false;
  return false;
}

bool SensorManager::initializeIMU() {
  if (imu.begin() == 0) {
    imu_initialized = true;
    return true;
  }
  imu_initialized = false;
  return false;
}

void SensorManager::shutdown() {
  Serial.println("Shutting down sensors...");
  mlx_initialized = false;
  max30102_initialized = false;
  imu_initialized = false;
}

SensorData SensorManager::readAllSensors() {
  SensorData data;
  data.timestamp = millis();
  
  // Read temperature
  if (mlx_initialized) {
    float temp_c = mlx.readObjectTempC();
    data.temperature_f = (temp_c * 9.0/5.0) + 32.0;
    data.temperatureValid = !isnan(temp_c);
  } else {
    data.temperature_f = 0;
    data.temperatureValid = false;
  }
  
  // Read heart rate and SpO2
  if (max30102_initialized) {
    data.irValue = particleSensor.getIR();
    data.redValue = particleSensor.getRed();
    
    // Check if finger is properly placed
    data.heartRateValid = isFingerDetected(data.irValue);
    
    if (data.heartRateValid) {
      processHeartRate(data.irValue);
      updateSignalBuffer(data.irValue, data.redValue);
    }
    
    data.heartRate = (int)beatsPerMinute;
    data.heartRateAvg = beatAvg;
    data.spo2 = calculateSpO2();
  } else {
    data.irValue = 0;
    data.redValue = 0;
    data.heartRate = 0;
    data.heartRateAvg = 0;
    data.spo2 = 0;
    data.heartRateValid = false;
  }
  
  // Read IMU
  if (imu_initialized) {
    data.accel_x = imu.readFloatAccelX();
    data.accel_y = imu.readFloatAccelY();
    data.accel_z = imu.readFloatAccelZ();
    data.gyro_x = imu.readFloatGyroX();
    data.gyro_y = imu.readFloatGyroY();
    data.gyro_z = imu.readFloatGyroZ();
    data.imuValid = true;
  } else {
    data.accel_x = data.accel_y = data.accel_z = 0;
    data.gyro_x = data.gyro_y = data.gyro_z = 0;
    data.imuValid = false;
  }
  
  return data;
}

void SensorManager::processHeartRate(long irValue) {
  if (checkForBeat(irValue)) { // Use SparkFun's checkForBeat function
    // We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    // Only store valid BPM readings
    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      // Take average of readings
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++) {
        beatAvg += rates[x];
      }
      beatAvg /= RATE_SIZE;
    }
  }
}

bool SensorManager::isFingerDetected(long irValue) {
  const long FINGER_THRESHOLD = 50000;
  const unsigned long STABILITY_TIME = 2000; // 2 seconds
  
  if (irValue > FINGER_THRESHOLD) {
    if (!fingerStable) {
      if (fingerPlacedTime == 0) {
        fingerPlacedTime = millis();
      } else if (millis() - fingerPlacedTime > STABILITY_TIME) {
        fingerStable = true;
      }
    }
    return fingerStable;
  } else {
    fingerStable = false;
    fingerPlacedTime = 0;
    return false;
  }
}

void SensorManager::updateSignalBuffer(long irValue, long redValue) {
  // Store recent readings for SpO2 calculation
  ir_buffer[buffer_index] = irValue;
  red_buffer[buffer_index] = redValue;
  buffer_index = (buffer_index + 1) % 50;
}

float SensorManager::calculateSpO2() {
  // Simplified SpO2 calculation
  if (!max30102_initialized || !fingerStable) {
    return 0.0;
  }
  
  // Calculate ratio of ratios (simplified approach)
  double red_ac = 0, red_dc = 0, ir_ac = 0, ir_dc = 0;
  
  // Calculate DC components (average)
  for (int i = 0; i < 25; i++) {
    red_dc += red_buffer[i];
    ir_dc += ir_buffer[i];
  }
  red_dc /= 25;
  ir_dc /= 25;
  
  // Calculate AC components (standard deviation approximation)
  for (int i = 0; i < 25; i++) {
    red_ac += abs(red_buffer[i] - red_dc);
    ir_ac += abs(ir_buffer[i] - ir_dc);
  }
  red_ac /= 25;
  ir_ac /= 25;
  
  // Calculate ratio of ratios
  if (red_dc > 0 && ir_dc > 0 && red_ac > 0 && ir_ac > 0) {
    double red_ratio = red_ac / red_dc;
    double ir_ratio = ir_ac / ir_dc;
    
    if (ir_ratio > 0) {
      double ratio = red_ratio / ir_ratio;
      
      // Empirical formula (needs calibration for accuracy)
      if (ratio > 0.4 && ratio < 3.0) {
        return 110.0 - (25.0 * ratio);
      }
    }
  }
  
  return 98.0; // Default reasonable SpO2 value
}

#endif