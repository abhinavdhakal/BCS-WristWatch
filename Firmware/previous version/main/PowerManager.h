#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <Arduino.h>

class PowerManager {
private:
  int powerSensePin;
  bool lastPowerState;
  unsigned long lastStateChange;
  const unsigned long DEBOUNCE_TIME = 100; // 100ms debounce
  
public:
  PowerManager();
  void init(int pin);
  bool checkPowerState();
  
  // Deep sleep functions (templates for future implementation)
  void enterDeepSleep();
  void wakeFromDeepSleep();
  void configureWakeupSources();
};

// Implementation
PowerManager::PowerManager() {
  powerSensePin = -1;
  lastPowerState = false;
  lastStateChange = 0;
}

void PowerManager::init(int pin) {
  powerSensePin = pin;
  pinMode(pin, INPUT_PULLDOWN);  // Use pull-down like your Python code
  
  // Read initial state
  lastPowerState = checkPowerState();
  lastStateChange = millis();
  
  Serial.print("Power manager initialized on pin ");
  Serial.println(pin);
  Serial.print("Initial power state: ");
  Serial.println(lastPowerState ? "SENSORS ON" : "SENSORS OFF");
}

bool PowerManager::checkPowerState() {
  if (powerSensePin == -1) return false;
  
  // Read the pin state
  // Based on your working Python code:
  // When pin is HIGH: Sensors OFF (3V goes to D10)
  // When pin is LOW: Sensors ON (3V goes to sensors)
  bool pinHigh = digitalRead(powerSensePin) == HIGH;
  
  // System is ON when pin is LOW (sensors powered)
  bool currentState = !pinHigh;
  
  // Debounce the reading
  if (currentState != lastPowerState) {
    if (millis() - lastStateChange > DEBOUNCE_TIME) {
      lastPowerState = currentState;
      lastStateChange = millis();
      
      if (currentState) {
        Serial.println("POWER STATE: Sensors ON (D10 LOW)");
      } else {
        Serial.println("POWER STATE: Sensors OFF (D10 HIGH) - Deep sleep mode");
      }
    }
  }
  
  return lastPowerState;
}

void PowerManager::enterDeepSleep() {
  // TODO: Implement deep sleep functionality
  Serial.println("📱 DEEP SLEEP TEMPLATE: Entering deep sleep mode...");
  Serial.println("   - Shutting down peripherals");
  Serial.println("   - Configuring wake sources");
  Serial.println("   - Reducing CPU frequency");
  Serial.println("   - Entering sleep mode");
  
  /*
  // Template for nRF52840 deep sleep implementation:
  
  // 1. Shutdown unnecessary peripherals
  // BLE.end();
  // Wire.end();
  // SPI.end();
  
  // 2. Configure GPIO for minimal power consumption
  // Set unused pins to input with pullup/pulldown as appropriate
  
  // 3. Configure wake-up sources
  // attachInterrupt(digitalPinToInterrupt(powerSensePin), wakeUpISR, CHANGE);
  
  // 4. Enter system sleep mode
  // sd_power_system_off(); // For Nordic nRF52 series
  // or use lower power sleep modes like:
  // __WFI(); // Wait For Interrupt
  
  */
}

void PowerManager::wakeFromDeepSleep() {
  // TODO: Implement wake-up functionality
  Serial.println("⚡ DEEP SLEEP TEMPLATE: Waking from deep sleep...");
  Serial.println("   - Reinitializing peripherals");
  Serial.println("   - Restoring CPU frequency");
  Serial.println("   - Restarting services");
  
  /*
  // Template for wake-up sequence:
  
  // 1. Reinitialize peripherals
  // Wire.begin();
  // SPI.begin();
  
  // 2. Reinitialize sensors (will be handled by main loop)
  
  // 3. Restart Bluetooth if needed
  // bluetooth.init();
  
  // 4. Resume normal operation
  
  */
}

void PowerManager::configureWakeupSources() {
  // TODO: Configure what can wake the system from deep sleep
  Serial.println("🔧 DEEP SLEEP TEMPLATE: Configuring wake sources...");
  Serial.println("   - Power state change (D11 pin)");
  Serial.println("   - Timer wake (optional)");
  Serial.println("   - External interrupt (optional)");
  
  /*
  // Template for wake source configuration:
  
  // 1. Power state pin interrupt
  // attachInterrupt(digitalPinToInterrupt(powerSensePin), wakeUpISR, CHANGE);
  
  // 2. Timer-based wake (for periodic sensor readings even in sleep)
  // app_timer_create(&wake_timer, APP_TIMER_MODE_REPEATED, wake_timer_handler);
  // app_timer_start(wake_timer, APP_TIMER_TICKS(30000), NULL); // Wake every 30 seconds
  
  // 3. Motion detection wake (using accelerometer interrupt)
  // Configure IMU to generate interrupt on motion threshold
  
  */
}

#endif