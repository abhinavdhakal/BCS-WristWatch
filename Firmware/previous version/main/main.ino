#include "SensorManager.h"
#include "BluetoothManager.h"
#include "PowerManager.h"

// Pin definitions
#define POWER_SENSE_PIN D10
#define SYSTEM_ON_THRESHOLD 1000  // millis to debounce power state

// Global managers
SensorManager sensors;
BluetoothManager bluetooth;
PowerManager power;

// System state
bool systemOn = false;
unsigned long lastPowerCheck = 0;
unsigned long lastDataSend = 0;
const unsigned long DATA_SEND_INTERVAL = 1000; // Send data every 1 second

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("=== Multi-Sensor Health Monitor ===");
  
  // Initialize power management
  power.init(POWER_SENSE_PIN);
  
  // Initialize Bluetooth
  bluetooth.init();
  
  Serial.println("System ready. Waiting for power state...");
}

void loop() {
  // Check power state every 100ms
  if (millis() - lastPowerCheck > 100) {
    bool newPowerState = power.checkPowerState();
    
    if (newPowerState != systemOn) {
      systemOn = newPowerState;
      handlePowerStateChange();
    }
    lastPowerCheck = millis();
  }
  
  // If system is on, read sensors and send data
  if (systemOn && (millis() - lastDataSend > DATA_SEND_INTERVAL)) {
    SensorData data = sensors.readAllSensors();
    bluetooth.sendSensorData(data);
    printSensorData(data);
    lastDataSend = millis();
  }
  
  // Handle Bluetooth
  bluetooth.handleConnections();
  
  // Small delay to prevent overwhelming the system
  delay(10);
}

void handlePowerStateChange() {
  if (systemOn) {
    Serial.println("=== SYSTEM POWERING ON ===");
    Serial.println("Sensors powered, initializing...");
    
    // Initialize all sensors
    if (sensors.initializeAll()) {
      Serial.println("All sensors initialized successfully");
    } else {
      Serial.println("Warning: Some sensors failed to initialize");
    }
    
    // TODO: Wake from deep sleep implementation
    // power.wakeFromDeepSleep();
    
  } else {
    Serial.println("=== SYSTEM POWERING OFF ===");
    Serial.println("Sensors powered down, preparing for sleep...");
    
    // Cleanup sensors
    sensors.shutdown();
    
    // TODO: Deep sleep implementation
    // power.enterDeepSleep();
  }
}

void printSensorData(const SensorData& data) {
  Serial.println("=== SENSOR DATA ===");
  
  // Temperature
  Serial.print("Temperature: ");
  Serial.print(data.temperature_f, 1);
  Serial.println("°F");
  
  // Heart Rate
  Serial.print("Heart Rate: ");
  Serial.print(data.heartRate);
  Serial.print(" BPM (Avg: ");
  Serial.print(data.heartRateAvg);
  Serial.print("), SpO2: ");
  Serial.print(data.spo2);
  Serial.println("%");
  
  // IMU Data
  Serial.print("Accel (g): X=");
  Serial.print(data.accel_x, 2);
  Serial.print(", Y=");
  Serial.print(data.accel_y, 2);
  Serial.print(", Z=");
  Serial.println(data.accel_z, 2);
  
  Serial.print("Gyro (°/s): X=");
  Serial.print(data.gyro_x, 2);
  Serial.print(", Y=");
  Serial.print(data.gyro_y, 2);
  Serial.print(", Z=");
  Serial.println(data.gyro_z, 2);
  
  Serial.print("IR Value: ");
  Serial.print(data.irValue);
  if (data.irValue < 50000) {
    Serial.print(" (No finger detected)");
  }
  Serial.println();
  Serial.println("==================");
}