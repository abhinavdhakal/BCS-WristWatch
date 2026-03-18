#include <Arduino.h>
#include "max30102.h"
#include "algorithm_by_RF.h"

// Quick-test: lower thresholds for weak signals
// In algorithm_by_RF.h:
// const float min_autocorrelation_ratio = 0.5;
// const float min_pearson_correlation = 0.5;

uint32_t red_buffer[BUFFER_SIZE];
uint32_t ir_buffer[BUFFER_SIZE];

void setup() {
    Serial.begin(115200);
    delay(500);

    if (!maxim_max30102_init()) {
        Serial.println("MAX30102 init failed!");
        while (1);
    }

    Serial.println("Quick test started. Place finger on sensor.");
}

void loop() {
    float spo2, ratio, correl;
    int8_t spo2_valid, hr_valid;
    int32_t heart_rate;

    // Read BUFFER_SIZE samples
    Serial.println("Reading buffer...");
    for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
        maxim_max30102_read_fifo(&red_buffer[i], &ir_buffer[i]);
        delay(2);
#ifdef DEBUG
        Serial.print("Sample "); Serial.print(i);
        Serial.print(" RED="); Serial.print(red_buffer[i]);
        Serial.print(" IR="); Serial.println(ir_buffer[i]);
#endif
    }

    // Compute HR & SpO2
    rf_heart_rate_and_oxygen_saturation(
        ir_buffer, BUFFER_SIZE, red_buffer,
        &spo2, &spo2_valid, &heart_rate, &hr_valid,
        &ratio, &correl
    );

    // Read sensor temperature
    int8_t temp_int;
    uint8_t temp_frac;
    maxim_max30102_read_temperature(&temp_int, &temp_frac);
    float temp_c = temp_int + temp_frac / 16.0;

    // Print results
    if (hr_valid && spo2_valid) {
        Serial.print("HR: "); Serial.print(heart_rate);
        Serial.print(" bpm, SpO2: "); Serial.print(spo2);
        Serial.print(" %, Temp: "); Serial.print(temp_c);
        Serial.println(" C");
    } else {
        Serial.println("Reading invalid, showing raw values:");
        for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
            Serial.print("RED="); Serial.print(red_buffer[i]);
            Serial.print(" IR="); Serial.println(ir_buffer[i]);
        }
    }

    delay(2000); // 2 seconds between readings
}
