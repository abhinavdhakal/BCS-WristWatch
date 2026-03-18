#include <Wire.h>
#include <ArduinoBLE.h>
#include "MAX30105.h"
#include "Adafruit_MLX90614.h"
#include "LSM6DS3.h"

#define SWITCH_PIN D10
#define LED_PIN LED_BUILTIN
#include "nrf_saadc.h"

// Pin definitions
#define VBAT_ENABLE_PIN 14        // P0.14, must be LOW to enable divider
const float VREF = 3.3;           // Reference voltage
const float DIVIDER_RATIO = 2.96; // Built-in resistor divider

float readBatteryVoltage()
{
    int raw = analogRead(PIN_VBAT); // 12-bit ADC (0-4095)
    float v_pin = (raw / 4095.0) * VREF;
    float voltage = v_pin * DIVIDER_RATIO;
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

// Timers for dual-rate updates
unsigned long lastSendIR = 0;
unsigned long lastSendOther = 0;
const int intervalIR = 20;     // 50Hz
const int intervalOther = 200; // 5Hz

void initSensors()
{
    Serial.println("Initializing sensors...");
    if (!max30102.begin(Wire, I2C_SPEED_STANDARD))
        Serial.println("MAX30102 not found!");
    else
        max30102.setup();

    if (!mlx.begin())
        Serial.println("MLX90614 not found!");

    if (imu.begin() != 0)
        Serial.println("IMU not found!");

    Serial.println("Sensors initialized");
    digitalWrite(LED_PIN, HIGH);
    sensorsInitialized = true;
}

void deinitSensors()
{
    Serial.println("Deinitializing sensors...");
    sensorsInitialized = false;
    digitalWrite(LED_PIN, LOW);
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    pinMode(VBAT_ENABLE_PIN, OUTPUT);
    digitalWrite(VBAT_ENABLE_PIN, LOW);

    pinMode(SWITCH_PIN, INPUT_PULLDOWN);
    pinMode(LED_PIN, OUTPUT);

    Serial.println("=== XIAO BLE Sensor System ===");

    BLE.setLocalName("BCS-Watch");

    if (!BLE.begin())
    {
        Serial.println("BLE init failed!");
        while (1)
            ;
    }

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

    // Initialize BLE characteristic values
    uint32_t initialValue = 0;
    irChar.setValue((uint8_t *)&initialValue, 4);
    redChar.setValue((uint8_t *)&initialValue, 4);
    mlxChar.setValue((uint8_t *)&initialValue, 4);
    battChar.setValue((uint8_t *)&initialValue, 4);

    uint8_t initialImuData[24] = {0};
    imuChar.setValue(initialImuData, 24);

    BLE.advertise();
    Serial.println("BLE advertising...");
}

void loop()
{
    BLEDevice central = BLE.central();

    if (central)
    {
        Serial.print("Connected: ");
        Serial.println(central.address());

        while (central.connected())
        {
            unsigned long now = millis();
            bool sensorsPowered = digitalRead(SWITCH_PIN) == LOW;

            if (sensorsPowered && !sensorsInitialized)
                initSensors();
            else if (!sensorsPowered && sensorsInitialized)
                deinitSensors();

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
        BLE.advertise();
    }
}
