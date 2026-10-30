#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AS5600.h>
#include "Encoder.h"

Adafruit_AS5600 as5600;

// data pin is A6 on ESP32
// clock pin is A7 on ESP32
const int SDA_PIN = A6;
const int SCL_PIN = A7;

// Calibration Values
// allow for 10 degrees offset from original positioning
const float CALIBRATION_TOLERANCE = 10.0;
const float MAX_ANGLE = 140.0;
const float SENSOR_ZERO = 0.0;

// store calibration data
float c_starting_angle = 0.0;
float c_max_angle = 0.0;

bool calibrated = false;
bool isEncoderCalibrated() {
    return calibrated;
}

float getSensorAngle() {
    uint16_t raw_angle = as5600.getRawAngle();
    float angle_deg = raw_angle * 360.0 / 4096.0;
    return angle_deg;
}

// setup() function
bool initializeEncoder() {

    // initalize I2C for clock and data
    Wire1.begin(SDA_PIN, SCL_PIN);

    // if the sensor is not detected notify the user and wait until it is detected
    if (!as5600.begin(0x36, &Wire1)) {
        Serial.println("Error: AS5600 was not detected");

        while (!as5600.begin(0x36, &Wire1)) {
            delay(2000);
            Serial.println("Retrying AS5600 connection...");
        }
    }

    Serial.println("AS5600 is connected");

    // check that magnet is detected
    if (!as5600.isMagnetDetected()) {
        Serial.println("Error: Magnet is not detected");

        while (!as5600.isMagnetDetected()) {
            delay(2000);
            Serial.println("Retrying magnet detection...");
        }
    }

    Serial.println("Magnet detected");

    int retries = 1;

    while (retries < 4 && !calibrated) {

        float initial_angle = getSensorAngle();

        Serial.print("Initial sensor angle: ");
        Serial.println(initial_angle);

        // check if the starting position is within 10 degrees of 0
        float angle_difference = initial_angle - SENSOR_ZERO;

        if (abs(angle_difference) <= CALIBRATION_TOLERANCE) {

            c_starting_angle = initial_angle;
            calibrated = true;

            Serial.println("AS5600 Calibrated Successfully");

            Serial.print("New zero position: ");
            Serial.println(c_starting_angle);

            // Change the maximum allowed angle based on the calibrated offset angle
            float limit_from_original_zero = SENSOR_ZERO + MAX_ANGLE;
            float limit_from_starting_position = c_starting_angle + MAX_ANGLE;

            c_max_angle = min(
                limit_from_original_zero,
                limit_from_starting_position
            );

            Serial.print("Maximum permitted sensor angle: ");
            Serial.println(c_max_angle);
        }
        else {

            float current_uncalibrated_angle = getSensorAngle();

            Serial.println("Calibration failed (arm must be within 10 degrees of perfectly straight)");

            Serial.print("Arm angle: ");
            Serial.println(current_uncalibrated_angle);

            Serial.print("Trying again, Retry: ");
            Serial.println(retries);

            delay(100);
        }

        retries += 1;
        delay(2000);
    }

    if (!calibrated) {
        Serial.println("Calibration failed");
        Serial.println("Arm must be within 10 degrees of perfectly straight");

        return false;  // CHANGED: return failure instead of infinite loop
    }

    return true;  // CHANGED: return success
}


// loop() function
void updateEncoder() {

    if (!calibrated) {
        return;
    }

    if (!as5600.isMagnetDetected()) {
        Serial.println("Magnet is too weak");
        delay(1000);
    }

    float current_angle = getSensorAngle();

    float angle_from_original_zero = current_angle - SENSOR_ZERO;

    float angle_from_starting_position = current_angle - c_starting_angle;

    Serial.print("Current sensor angle: ");
    Serial.print(current_angle);

    Serial.print(" | Movement from starting position: ");
    Serial.print(angle_from_starting_position);

    Serial.print(" | Max angle permitted: ");
    Serial.println(c_max_angle);

    if (angle_from_original_zero > c_max_angle) {
        Serial.println("At the maximum rotation limit!");

        // NO MOTOR MOVEMENT COMMANDS FROM HERE
    }

    delay(500);
}