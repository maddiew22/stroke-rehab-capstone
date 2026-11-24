#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AS5600.h>
#include <float.h>
//#include "Encoder.h"
#include <Preferences.h>

Adafruit_AS5600 as5600;
Preferences pref;

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

const float MIN_ALLOWED_ANGLE = -10.0;
const float MAX_ALLOWED_ANGLE = 150.0;

// store calibration data
float c_starting_angle = 0.0;
float c_max_angle = 0.0;
float c_min_angle = 0.0;

bool calibrated = false;

bool patient_limits_saved = false; 
bool min_limit_set = false;
bool max_limit_set = false;

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

bool load_patient_limits() {
  pref.begin("patient", false);
  bool limit_exists = pref.getBool("limits_set", false);

  if (limit_exists) {
    c_min_angle = pref.getFloat("min_angle", 0.0);
    c_max_angle = pref.getFloat("max_angle", 0.0);
  }
  pref.end();
  if (limit_exists && c_min_angle < c_max_angle && c_min_angle >= MIN_ALLOWED_ANGLE && c_max_angle <= MAX_ALLOWED_ANGLE){
    patient_limits_saved = true;

    min_limit_set = true;
    max_limit_set = true;
    Serial.println("Patient Calibrations loaded successfully");

    Serial.print("Minimum loaded elbow angle: ");
    Serial.println(c_min_angle);
    Serial.print("Maximum loaded elbow angle: ");
    Serial.println(c_max_angle);

    return true;
  }

  Serial.println("No valid patient calibration found");
  Serial.println("Physiotherapist must calibrate device before use");
  patient_limits_saved = false;
  min_limit_set = false;
  max_limit_set = false;
  return false;
}


bool save_patient_limits(){
  pref.begin("patient", false);
  pref.putFloat("min_angle", c_min_angle);
  pref.putFloat("max_angle", c_max_angle);
  
  pref.putBool("limits_set", true);
  pref.end();
  patient_limits_saved=true;

  Serial.println("Patient calibration saved successfully");
  return true;
}


void set_patient_limits() {
  Serial.println("PATIENT CALIBRATION LIMITS");
  Serial.println("Press L when ready to set LOWER limit");
  Serial.println("Press U when ready to set UPPER limit");
  delay(200);

  while (!min_limit_set || !max_limit_set){
      Serial.print("Current elbow angle: ");
      Serial.println(getSensorAngle());

      if (Serial.available()){
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toUpperCase();

        if (command == "L"){
          Serial.println("Keep arm straight at LOWER limit");
          c_min_angle = getSensorAngle(); 
          min_limit_set = true;
          Serial.print("Lower elbow angle: ");
          Serial.println(c_min_angle);
        }
        else if (command == "U"){
          Serial.println("Keep arm bent at UPPER limit");
          c_max_angle = getSensorAngle(); 
          max_limit_set = true;
          Serial.print("Upper elbow angle: ");
          Serial.println(c_max_angle);
        }
        else if (command == "R"){
          c_min_angle = 0.0;
          c_max_angle = 0.0;
          min_limit_set = false;
          max_limit_set = false;
          Serial.println("Both limits RESET");
          Serial.println("Recalibrate both limits");
          Serial.println("Press L when ready to set LOWER limit");
          Serial.println("Press U when ready to set UPPER limit");
        }
        else {
          Serial.println("Invalid command. Enter L, U or R");
        }
      }
      delay(200);
  }
}

bool calibrate_patient_limits(){
  while (true) {
    min_limit_set = false;
    max_limit_set = false;
    c_min_angle = 0.0;
    c_max_angle = 0.0;

    set_patient_limits();

    // validate the limits 
    if (c_min_angle >= c_max_angle || c_min_angle < MIN_ALLOWED_ANGLE || c_max_angle > MAX_ALLOWED_ANGLE) {
      Serial.println("ERROR: Invalid Patient limits - RECALIBRATE");
      continue;
    }

    Serial.println("Patient calibration complete");
//    Serial.print("Starting position: ");
//    Serial.println(c_starting_angle);
    Serial.print("Lower limit: ");
    Serial.println(c_min_angle);
    Serial.print("Upper limit: ");
    Serial.println(c_max_angle);

    save_patient_limits();
    break;
  }
  return true;
}


// setup() function 
void initializeEncoder() {
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

    bool saved = load_patient_limits();

    if(saved) {
      Serial.println("Previous patient calibration found");
    }
    else {
      calibrate_patient_limits();
    }
    
    calibrated = true;
    
//    return false;  // CHANGED: return failure instead of infinite loop
//
//   return true;  // CHANGED: return success
}


// loop() function
void updateEncoder() {
// void loop(){ //keep for testing

  if (Serial.available()){
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();

    if (command == "C") {
      Serial.println("Recalibration Requested");

      calibrated = false;
      min_limit_set = false;
      max_limit_set = false;

      calibrate_patient_limits();

      calibrated = true;
      Serial.println("Recalibration complete");
    }
  }

    if (!calibrated) {
        return;
    }

    while (!as5600.isMagnetDetected()) {
        Serial.println("Magnet is too weak");
        delay(1000);
    }

    float raw_current_angle = getSensorAngle(); //from zero

//    float elbow_angle = raw_current_angle - c_starting_angle; // from calibrated start
    
    Serial.print("Raw sensor angle: ");
    Serial.print(raw_current_angle);

//    Serial.print(" | Elbow Movement from starting position: ");
//    Serial.print(elbow_angle);

    Serial.print(" | Max angle permitted: ");
    Serial.print(c_max_angle);
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