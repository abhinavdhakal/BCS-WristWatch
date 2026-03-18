#include <Wire.h>
#include <ArduinoBLE.h>
#include "MAX30105.h"
#include "Adafruit_MLX90614.h"
#include "LSM6DS3.h"
#include "nrf.h"
#include "nrf_power.h"
#include "nrf_gpio.h"

#define SWITCH_PIN D10
#define LED_PIN LED_BUILTIN
#include "nrf_saadc.h"

// Pin definitions
#define VBAT_ENABLE_PIN 14             // P0.14, must be LOW to enable divider
const float VREF = 3.3;                // Reference voltage
const float DIVIDER_RATIO = 2.96;      // Built-in resistor divider
const float BATTERY_CAL_FACTOR = 1.00; // Start at 1.00, tune gently with multimeter
const int BATTERY_SAMPLES = 16;
const float BATTERY_MIN_VALID_V = 2.5;
const float BATTERY_MAX_VALID_V = 4.5;
const char *BLE_DEVICE_NAME = "BCS";
const uint32_t DEVICE_SLEEP_MARKER = 0xFFFFFFFF;

float readBatteryVoltage()
{
    static float lastGoodVoltage = 3.7;

    uint32_t sum = 0;
    for (int i = 0; i < BATTERY_SAMPLES; i++)
    {
        sum += analogRead(PIN_VBAT);
        delayMicroseconds(250);
    }

    float raw = sum / (float)BATTERY_SAMPLES; // 12-bit ADC (0-4095)
    float v_pin = (raw / 4095.0f) * VREF;
    float voltage = v_pin * DIVIDER_RATIO * BATTERY_CAL_FACTOR;

    if (voltage < BATTERY_MIN_VALID_V || voltage > BATTERY_MAX_VALID_V)
        return lastGoodVoltage;

    lastGoodVoltage = voltage;
    return voltage;
}

MAX30105 max30102;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
LSM6DS3 imu(I2C_MODE, 0x6A);

// BLE setup
BLEService sensorService("12345678-1234-1234-1234-123456789abc");
BLECharacteristic irChar("12345678-1234-1234-1234-123456789ab1", BLERead | BLENotify, 4);
BLECharacteristic redChar("12345678-1234-1234-1234-123456789ab2", BLERead | BLENotify, 4);
BLECharacteristic mlxChar("12345678-1234-1234-1234-123456789ab3", BLERead | BLENotify, 4);
BLECharacteristic imuChar("12345678-1234-1234-1234-123456789ab4", BLERead | BLENotify, 24);
BLECharacteristic battChar("12345678-1234-1234-1234-123456789ab5", BLERead | BLENotify, 4);

// Descriptors
BLEDescriptor irDesc("2901", "IR Light Sensor");
BLEDescriptor redDesc("2901", "Red Light Sensor");
BLEDescriptor mlxDesc("2901", "Temperature Sensor");
BLEDescriptor imuDesc("2901", "IMU Data (ax,ay,az,gx,gy,gz)");
BLEDescriptor battDesc("2901", "Battery Voltage");

bool sensorsInitialized = false;
bool bleInitialized = false;

// Retry sensor initialization while switch remains ON
unsigned long lastInitAttempt = 0;
const unsigned long INIT_RETRY_INTERVAL_MS = 2000;

// Timers for dual-rate updates
unsigned long lastSendIR = 0;
unsigned long lastSendOther = 0;
const int intervalIR = 20;     // 50Hz
const int intervalOther = 200; // 5Hz

// Deep sleep mode flag
volatile bool inDeepSleep = false;

bool isSwitchOn()
{
    // ON means D10 is LOW (active-low logic)
    int lowCount = 0;
    for (int i = 0; i < 5; i++)
    {
        if (digitalRead(SWITCH_PIN) == LOW)
            lowCount++;
        delay(2);
    }
    return lowCount >= 3;
}

bool initSensors()
{
    Serial.println("Initializing sensors...");
    bool maxOk = false;
    bool mlxOk = false;
    bool imuOk = false;

    if (!max30102.begin(Wire, I2C_SPEED_STANDARD))
        Serial.println("MAX30102 not found!");
    else
    {
        max30102.setup();
        maxOk = true;
    }

    if (!mlx.begin())
        Serial.println("MLX90614 not found!");
    else
        mlxOk = true;

    if (imu.begin() != 0)
        Serial.println("IMU not found!");
    else
        imuOk = true;

    sensorsInitialized = maxOk && mlxOk && imuOk;

    if (sensorsInitialized)
    {
        Serial.println("Sensors initialized");
        digitalWrite(LED_PIN, HIGH);
    }
    else
    {
        Serial.println("Sensor init incomplete, will retry...");
        digitalWrite(LED_PIN, LOW);
    }

    return sensorsInitialized;
}

void deinitSensors()
{
    Serial.println("Deinitializing sensors...");
    sensorsInitialized = false;
    digitalWrite(LED_PIN, LOW);
}

void notifyDeepSleepToCentral()
{
    if (!bleInitialized)
        return;

    uint32_t marker = DEVICE_SLEEP_MARKER;
    battChar.setValue((uint8_t *)&marker, 4);
    delay(80);
}

void configureWakeSenseLow()
{
    uint32_t gpioPin = (uint32_t)digitalPinToPinName(SWITCH_PIN);
    nrf_gpio_cfg_sense_input(gpioPin, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_LOW);
}

// Enter deep sleep mode
void enterDeepSleep()
{
    Serial.println("Entering deep sleep mode...");
    Serial.flush(); // Ensure all serial data is sent
    delay(100);

    inDeepSleep = true;

    // Notify connected GUI right before BLE shutdown.
    notifyDeepSleepToCentral();

    // Stop BLE to save power
    BLE.end();
    bleInitialized = false;

    // Disable UART
    Serial.end();

    // Turn off LED
    digitalWrite(LED_PIN, LOW);
    pinMode(LED_PIN, INPUT);

    // Disable battery voltage divider
    digitalWrite(VBAT_ENABLE_PIN, HIGH);
    pinMode(VBAT_ENABLE_PIN, INPUT);

    // Set all unused pins to INPUT to prevent current draw
    for (int i = 0; i < 32; i++)
    {
        // Skip SWITCH_PIN as we need it for wake-up
        if (i != SWITCH_PIN)
        {
            pinMode(i, INPUT);
        }
    }

    // Small delay for peripherals to shut down
    delay(10);

    // Configure wake source for true SYSTEMOFF.
    configureWakeSenseLow();

    // Enter deepest sleep. Wake causes reset and starts from setup().
    NRF_POWER->SYSTEMOFF = 1;

    // Fallback (should never be reached)
    while (1)
        ;
}

// Exit deep sleep and resume normal operation
void exitDeepSleep()
{
    // Re-initialize Serial
    Serial.begin(115200);
    delay(100);

    Serial.println("Resuming from deep sleep...");
    inDeepSleep = false;

    // Re-enable battery voltage divider
    pinMode(VBAT_ENABLE_PIN, OUTPUT);
    digitalWrite(VBAT_ENABLE_PIN, LOW);

    // Re-enable LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Small delay for system stabilization
    delay(100);
}

bool initBLEStack()
{
    if (bleInitialized)
        return true;

    if (!BLE.begin())
    {
        Serial.println("BLE init failed!");
        return false;
    }

    BLE.setDeviceName(BLE_DEVICE_NAME);
    BLE.setLocalName(BLE_DEVICE_NAME);

    BLE.setAdvertisedService(sensorService);
    sensorService.addCharacteristic(irChar);
    sensorService.addCharacteristic(redChar);
    sensorService.addCharacteristic(mlxChar);
    sensorService.addCharacteristic(imuChar);
    sensorService.addCharacteristic(battChar);

    irChar.addDescriptor(irDesc);
    redChar.addDescriptor(redDesc);
    mlxChar.addDescriptor(mlxDesc);
    imuChar.addDescriptor(imuDesc);
    battChar.addDescriptor(battDesc);

    BLE.addService(sensorService);

    uint32_t initialValue = 0;
    irChar.setValue((uint8_t *)&initialValue, 4);
    redChar.setValue((uint8_t *)&initialValue, 4);
    mlxChar.setValue((uint8_t *)&initialValue, 4);
    battChar.setValue((uint8_t *)&initialValue, 4);

    uint8_t initialImuData[24] = {0};
    imuChar.setValue(initialImuData, 24);

    bleInitialized = true;
    return true;
}

void startAdvertisingIfReady()
{
    if (bleInitialized && sensorsInitialized)
    {
        BLE.advertise();
        Serial.println("BLE advertising...");
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("FW tag: current_version.ino / BLE name BCS");

    analogReadResolution(12);

    pinMode(VBAT_ENABLE_PIN, OUTPUT);
    digitalWrite(VBAT_ENABLE_PIN, LOW);

    pinMode(SWITCH_PIN, INPUT_PULLDOWN);
    pinMode(LED_PIN, OUTPUT);

    // Set battery charging current to 100mA
    pinMode(22, OUTPUT);
    digitalWrite(22, LOW);

    Serial.println("=== XIAO BLE Sensor System ===");

    bool sensorsPowered = isSwitchOn();
    if (!sensorsPowered)
    {
        Serial.println("Switch is OFF at boot -> deep sleep");
        enterDeepSleep();
        return;
    }

    if (!initBLEStack())
    {
        while (1)
            ;
    }

    initSensors();
    startAdvertisingIfReady();
}

void loop()
{
    unsigned long now = millis();

    // ===== Check power state first =====
    bool sensorsPowered = isSwitchOn();

    if (!sensorsPowered)
    {
        if (sensorsInitialized)
            deinitSensors();
        enterDeepSleep();
        return;
    }

    // Retry sensor init while switch is ON until all sensors are ready
    if (!sensorsInitialized && (now - lastInitAttempt >= INIT_RETRY_INTERVAL_MS))
    {
        lastInitAttempt = now;

        if (!bleInitialized && !initBLEStack())
        {
            Serial.println("BLE init retry failed");
            delay(300);
            return;
        }

        initSensors();
        if (sensorsInitialized)
            startAdvertisingIfReady();
    }

    if (!sensorsInitialized)
    {
        delay(100);
        return;
    }

    // ===== Handle BLE connection and data transmission =====
    BLEDevice central = BLE.central();

    if (central)
    {
        Serial.print("Connected: ");
        Serial.println(central.address());

        while (central.connected())
        {
            now = millis();

            // Re-check power state during connection
            sensorsPowered = isSwitchOn();

            if (!sensorsPowered)
            {
                deinitSensors();
                enterDeepSleep();
                return;
            }

            // ===== Send sensor data only when sensors are initialized =====
            if (sensorsInitialized)
            {
                // ----- IR & Red at 50Hz -----
                if (now - lastSendIR >= intervalIR)
                {
                    lastSendIR = now;
                    long ir = max30102.getIR();
                    long red = max30102.getRed();

                    uint32_t irValue = (uint32_t)ir;
                    uint32_t redValue = (uint32_t)red;
                    irChar.setValue((uint8_t *)&irValue, 4);
                    redChar.setValue((uint8_t *)&redValue, 4);

                    // Optional: print occasionally
                    static unsigned long lastPrintIR = 0;
                    if (now - lastPrintIR >= 200)
                    {
                        lastPrintIR = now;
                        Serial.print("IR: ");
                        Serial.print(ir);
                        Serial.print("  RED: ");
                        Serial.println(red);
                    }
                }

                // ----- Other sensors at 5Hz -----
                if (now - lastSendOther >= intervalOther)
                {
                    lastSendOther = now;

                    // Temperature
                    float objectTemp = mlx.readObjectTempC();
                    uint32_t tempValue = (uint32_t)(objectTemp * 100);
                    mlxChar.setValue((uint8_t *)&tempValue, 4);

                    // IMU
                    float ax = imu.readFloatAccelX();
                    float ay = imu.readFloatAccelY();
                    float az = imu.readFloatAccelZ();
                    float gx = imu.readFloatGyroX();
                    float gy = imu.readFloatGyroY();
                    float gz = imu.readFloatGyroZ();
                    uint8_t imuData[24];
                    memcpy(imuData, &ax, 4);
                    memcpy(imuData + 4, &ay, 4);
                    memcpy(imuData + 8, &az, 4);
                    memcpy(imuData + 12, &gx, 4);
                    memcpy(imuData + 16, &gy, 4);
                    memcpy(imuData + 20, &gz, 4);
                    imuChar.setValue(imuData, 24);

                    // Battery
                    float vbat = readBatteryVoltage();
                    uint32_t battValue = (uint32_t)(vbat * 1000);
                    battChar.setValue((uint8_t *)&battValue, 4);

                    // Optional: print occasionally
                    Serial.print("Temp: ");
                    Serial.print(objectTemp, 1);
                    Serial.print("  AX: ");
                    Serial.print(ax, 2);
                    Serial.print(" AY: ");
                    Serial.print(ay, 2);
                    Serial.print(" AZ: ");
                    Serial.print(az, 2);
                    Serial.print("  GX: ");
                    Serial.print(gx, 2);
                    Serial.print(" GY: ");
                    Serial.print(gy, 2);
                    Serial.print(" GZ: ");
                    Serial.print(gz, 2);
                    Serial.print("  Batt: ");
                    Serial.println(vbat, 2);
                }
            }
        }

        Serial.println("Central disconnected, restarting advertising...");
        startAdvertisingIfReady();
    }

    // Small delay to prevent excessive loop iterations when not connected
    delay(100);
}