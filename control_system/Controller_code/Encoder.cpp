#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AS5600.h>
#include <float.h>
#include "Encoder.h"

Adafruit_AS5600 as5600;

// data pin is A6 on ESP32
// clock pin is A7 on ESP32
const int SDA_PIN = A6;
const int SCL_PIN = A7;

// Calibration Values
// allow for 10 degrees offset from original positioning
const float CALIBRATION_TOLERANCE = 50.0; // change back to 10 later (for now makes testing easier)
const float MAX_ANGLE = 140.0;
const float SENSOR_ZERO = 0.0;
const int CALIBRATION_SAMPLES = 100;
const float MAX_CALIBRATION_VARIATION = 10.0;
const int MAX_RETRIES = 10; // change later (for now makes testing easier)
const int DIRECTION = 1; // set to -1 if the magnet is flipped when placed in enclosure

// store calibration data
float c_starting_angle = 0.0;
float c_max_angle = 0.0;
float c_min_angle = 0.0;
bool calibrated = false;

bool isEncoderCalibrated() {
    return calibrated;
}

float getSensorAngle() {
  uint16_t raw_angle = as5600.getRawAngle();
  float angle_deg = raw_angle * 360.0 / 4096.0;
  
  // switch angle range from 0 to 360 to -180 to 180
  while(angle_deg>180.0){
    angle_deg = angle_deg - 360.0;
  }
    while(angle_deg<=-180.0){
    angle_deg = angle_deg + 360.0;
  }
  return DIRECTION * angle_deg;
}

bool getStableAngle(float &avg_angle){
  float angle_sum = 0.0;
  float min_angle = FLT_MAX;
  float max_angle = -FLT_MAX;

  for (int i = 0; i<CALIBRATION_SAMPLES; i++){
    float angle=getSensorAngle();
    
    angle_sum = angle + angle_sum;

    if (angle<min_angle) {
      min_angle= angle;
    }
    if (angle>max_angle){
      max_angle= angle;
    }
    delay(10);
  }
  // chack the variation betweeon the highest and lowest readings to ensure they are within normal range
  float variation = max_angle-min_angle;

  Serial.print("Calibration variation: ");
  Serial.println(variation);

  if (variation > MAX_CALIBRATION_VARIATION){
    return false;
  }

  avg_angle = angle_sum / CALIBRATION_SAMPLES;
  return true;
}

// setup() function 
bool initializeEncoder() {
// void setup(){ //Keep this for testing - I can't test this on the arduino without a void setup and loop function 
    Serial.begin(115200);
    // initalize I2C for clock and data
    Wire1.begin(SDA_PIN, SCL_PIN);

    // if the sensor is not detected notify the user and wait until it is detected
    while (!as5600.begin(0x36, &Wire1)) {
        Serial.println("Error: AS5600 was not detected, retrying ...");
        delay(1000);
    }
    Serial.println("AS5600 connected");
    
    // check that magnet is detected
    while (!as5600.isMagnetDetected()) {
        Serial.println("Error: Magnet is not detected, retrying ...");
        delay(1000);
    }
    Serial.println("Magnet detected");
    
    for (int retries=1; retries<=MAX_RETRIES && !calibrated; retries++) {
        float initial_angle = 0.0;
    
        if (!getStableAngle(initial_angle)){
          Serial.println("Calibration failed: reading unstable - keep arm still");
          delay(1000);
          continue;
        }

        Serial.print("Initial sensor angle: ");
        Serial.println(initial_angle);

        // check if the starting position is within 10 degrees of 0
        float angle_difference = initial_angle - SENSOR_ZERO;

        if (fabs(angle_difference) <= CALIBRATION_TOLERANCE) {

            c_starting_angle = initial_angle;
            calibrated = true;

            c_min_angle = c_starting_angle;
            c_max_angle = min(MAX_ANGLE, c_starting_angle + MAX_ANGLE);

            Serial.println("AS5600 Calibrated Successfully");
            Serial.print("New zero position: ");
            Serial.println(c_starting_angle);
            Serial.print("Maximum permitted sensor angle: ");
            Serial.println(c_max_angle);
            Serial.print("Minimum permitted sensor angle: ");
            Serial.println(c_min_angle);
        }
        else {
            Serial.print("Calibration failed - arm must be within ");  
            Serial.print(CALIBRATION_TOLERANCE);
            Serial.println(" degrees of perfectly straight");
            Serial.print("Arm angle: ");
            Serial.println(initial_angle);
            Serial.print("Trying again, Retry: ");
            Serial.println(retries);
            delay(2000);
        }
    }

    if (!calibrated) {
        Serial.println("Calibration failed");
        Serial.println("Straighten arm and reset the board");

       return false;  // CHANGED: return failure instead of infinite loop
    }

   return true;  // CHANGED: return success
}


// loop() function
void updateEncoder() {
// void loop(){ //keep for testing

    if (!calibrated) {
        return;
    }

    while (!as5600.isMagnetDetected()) {
        Serial.println("Magnet is too weak");
        delay(1000);
    }

    float raw_current_angle = getSensorAngle(); //from zero

    float elbow_angle = raw_current_angle - c_starting_angle; // from calibrated start

    Serial.print("Raw sensor angle: ");
    Serial.print(raw_current_angle);

    Serial.print(" | Elbow Movement from starting position: ");
    Serial.print(elbow_angle);

    Serial.print(" | Max angle permitted: ");
    Serial.println(c_max_angle);
    Serial.print(" | Min angle permitted: ");
    Serial.println(c_min_angle);

    if (raw_current_angle > c_max_angle) {
        Serial.println("At the maximum rotation limit!");
        // NO MOTOR MOVEMENT COMMANDS FROM HERE
    }

    if (raw_current_angle < c_min_angle) {
        Serial.println("At the minimum rotation limit!");
        // NO MOTOR MOVEMENT COMMANDS FROM HERE
    }

    delay(500);
}
