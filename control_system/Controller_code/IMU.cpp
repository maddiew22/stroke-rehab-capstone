
// import libraries
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Preferences.h>

// Initialization
Adafruit_MPU6050 mpu_right;
Adafruit_MPU6050 mpu_left;
Preferences pref;

// CONSTANTS
const int SDA_PIN = A6;
const int SCL_PIN = A7;
const int MPU_ADDRESS_RIGHT = 0x68;
const int MPU_ADDRESS_LEFT = 0x69;
const int VIBRATIONAL_MOTOR_PIN = 4; // update when motor is in place
// calibration constants
const float FORWARD_LEAN_THRESHOLD = 10.0;
const float SIDE_LEAN_THRESHOLD = 10.0;
const int CALIBRATION_SAMPLES = 500;
const float ACCEL_X = 0.0;
const float ACCEL_Y = 0.0;
const float ACCEL_Z = 9.81;

struct IMU_calibration {
  float gyro_offset_x = 0.0;
  float gyro_offset_y = 0.0;
  float gyro_offset_z = 0.0;

  float accel_offset_x = 0.0;
  float accel_offset_y = 0.0;
  float accel_offset_z = 0.0;

  float accel_scale_x = 1.0;
  float accel_scale_y = 1.0;
  float accel_scale_z = 1.0;

  float pitch_offset = 0.0;
  float roll_offset = 0.0;

  float pitch_filtered = 0.0;
  float roll_filtered = 0.0;
};

IMU_calibration calibration_right;
IMU_calibration calibration_left;

// Calibrate Gyroscopes Together
void calibrateBothGyroscopes() {
  // initialize temp variables for averaging final calibration
  float right_sum_x = 0.0;
  float right_sum_y = 0.0;
  float right_sum_z = 0.0;

  float left_sum_x = 0.0;
  float left_sum_y = 0.0;
  float left_sum_z = 0.0;
  
  sensors_event_t accel_right;
  sensors_event_t gyro_right; 
  sensors_event_t temp_right;
   
  sensors_event_t accel_left;
  sensors_event_t gyro_left; 
  sensors_event_t temp_left;

  Serial.println("CALIBRATING GYROSCOPE");
  Serial.println("Sit upright and keep completely still...");
  delay(1000);

  for (int i=0; i<CALIBRATION_SAMPLES; i++){
    mpu_right.getEvent(&accel_right, &gyro_right, &temp_right);
    mpu_left.getEvent(&accel_left, &gyro_left, &temp_left);

    right_sum_x += gyro_right.gyro.x;
    right_sum_y += gyro_right.gyro.y;
    right_sum_z += gyro_right.gyro.z;

    left_sum_x += gyro_left.gyro.x;
    left_sum_y += gyro_left.gyro.y;
    left_sum_z += gyro_left.gyro.z;

    delay(10);
  }

  calibration_right.gyro_offset_x = right_sum_x/CALIBRATION_SAMPLES;
  calibration_right.gyro_offset_y = right_sum_y/CALIBRATION_SAMPLES;
  calibration_right.gyro_offset_z = right_sum_z/CALIBRATION_SAMPLES;

  calibration_left.gyro_offset_x = left_sum_x/CALIBRATION_SAMPLES;
  calibration_left.gyro_offset_y = left_sum_y/CALIBRATION_SAMPLES;
  calibration_left.gyro_offset_z = left_sum_z/CALIBRATION_SAMPLES;

  Serial.println("Both Gyroscopes are calibrated");
  delay(500);
  Serial.print("RIGHT X OFFSET: ");
  Serial.println(calibration_right.gyro_offset_x);
  Serial.print("RIGHT Y OFFSET: ");
  Serial.println(calibration_right.gyro_offset_y);
  Serial.print("RIGHT Z OFFSET: ");
  Serial.println(calibration_right.gyro_offset_z);

  Serial.print("LEFT X OFFSET: ");
  Serial.println(calibration_left.gyro_offset_x);
  Serial.print("LEFT Y OFFSET: ");
  Serial.println(calibration_left.gyro_offset_y);
  Serial.print("LEFT Z OFFSET: ");
  Serial.println(calibration_left.gyro_offset_z);
}

void calibrateBothAccelerometers(){
  // initialize temp variables for averaging final calibration
  float right_sum_x = 0.0;
  float right_sum_y = 0.0;
  float right_sum_z = 0.0;

  float left_sum_x = 0.0;
  float left_sum_y = 0.0;
  float left_sum_z = 0.0;
  
  sensors_event_t accel_right;
  sensors_event_t gyro_right; 
  sensors_event_t temp_right;
   
  sensors_event_t accel_left;
  sensors_event_t gyro_left; 
  sensors_event_t temp_left;

  Serial.println("ACCELEROMETER CALIBRATION");
  delay(1000);

  for (int i=0; i<CALIBRATION_SAMPLES; i++){
    mpu_right.getEvent(&accel_right, &gyro_right, &temp_right);
    mpu_left.getEvent(&accel_left, &gyro_left, &temp_left);

    right_sum_x += accel_right.acceleration.x;
    right_sum_y += accel_right.acceleration.y;
    right_sum_z += accel_right.acceleration.z;
      
    left_sum_x += accel_left.acceleration.x;
    left_sum_y += accel_left.acceleration.y;
    left_sum_z += accel_left.acceleration.z;
    
    delay(10);
  }
  float right_avg_x = right_sum_x/CALIBRATION_SAMPLES;
  float right_avg_y = right_sum_y/CALIBRATION_SAMPLES;
  float right_avg_z = right_sum_z/CALIBRATION_SAMPLES;

  float left_avg_x = left_sum_x/CALIBRATION_SAMPLES;
  float left_avg_y = left_sum_y/CALIBRATION_SAMPLES;
  float left_avg_z = left_sum_z/CALIBRATION_SAMPLES;

  // Adjust for direction and gravity

  calibration_right.accel_offset_x = right_avg_x - ACCEL_X;
  calibration_right.accel_offset_y = right_avg_y - ACCEL_Y;
  calibration_right.accel_offset_z = right_avg_z - ACCEL_Z;

  calibration_left.accel_offset_x = left_avg_x - ACCEL_X;
  calibration_left.accel_offset_y = left_avg_y - ACCEL_Y;
  calibration_left.accel_offset_z = left_avg_z - (-1)*ACCEL_Z;

  // UPDATE SOON: need to perform one time calibration on each sensor to get these values
  calibration_right.accel_scale_x = 1.0; 
  calibration_right.accel_scale_y = 1.0; 
  calibration_right.accel_scale_z = 1.0; 

  calibration_left.accel_scale_x = 1.0; 
  calibration_left.accel_scale_y = 1.0; 
  calibration_left.accel_scale_z = 1.0;
  
  Serial.println("Both Accelerometers are Calibrated");
  delay(500);
  Serial.print("RIGHT X avg: ");
  Serial.println(right_avg_x);
  Serial.print("RIGHT Y avg: ");
  Serial.println(right_avg_y);
  Serial.print("RIGHT Z avg: ");
  Serial.println(right_avg_z);

  Serial.print("RIGHT X OFFSET: ");
  Serial.println(calibration_right.accel_offset_x);
  Serial.print("RIGHT Y OFFSET: ");
  Serial.println(calibration_right.accel_offset_y);
  Serial.print("RIGHT Z OFFSET: ");
  Serial.println(calibration_right.accel_offset_z);

  delay(500);
  Serial.print("LEFT X avg: ");
  Serial.println(left_avg_x);
  Serial.print("LEFT Y avg: ");
  Serial.println(left_avg_y);
  Serial.print("LEFT Z avg: ");
  Serial.println(left_avg_z);
  
  Serial.print("LEFT X OFFSET: ");
  Serial.println(calibration_left.accel_offset_x);
  Serial.print("LEFT Y OFFSET: ");
  Serial.println(calibration_left.accel_offset_y);
  Serial.print("LEFT Z OFFSET: ");
  Serial.println(calibration_left.accel_offset_z);
}

void saveCalibration() {

  pref.begin("mpu_calibration", false);

  // Right Gyroscope
  pref.putFloat("rgx", calibration_right.gyro_offset_x);
  pref.putFloat("rgy", calibration_right.gyro_offset_y);
  pref.putFloat("rgz", calibration_right.gyro_offset_z);

  // Right Accelerometer
  pref.putFloat("rax", calibration_right.accel_offset_x);
  pref.putFloat("ray", calibration_right.accel_offset_y);
  pref.putFloat("raz", calibration_right.accel_offset_z);

  // Left Gyroscope
  pref.putFloat("lgx", calibration_left.gyro_offset_x);
  pref.putFloat("lgy", calibration_left.gyro_offset_y);
  pref.putFloat("lgz", calibration_left.gyro_offset_z);

  // Right Accelerometer
  pref.putFloat("lax", calibration_left.accel_offset_x);
  pref.putFloat("lay", calibration_left.accel_offset_y);
  pref.putFloat("laz", calibration_left.accel_offset_z);

  pref.end();

  Serial.println("Sensor calibrations saved!");
}

bool loadCalibration(){
  // not calibrated - no preferences saved
  if (!calibrationExists()){
    pref.end();
    return false;
  }

  pref.begin("mpu_calibration", true);

  // RIGHT 
  calibration_right.gyro_offset_x = pref.getFloat("rgx",0.0);
  calibration_right.gyro_offset_y = pref.getFloat("rgy",0.0);
  calibration_right.gyro_offset_z = pref.getFloat("rgz",0.0);

  calibration_right.accel_offset_x = pref.getFloat("rax",0.0);
  calibration_right.accel_offset_y = pref.getFloat("ray",0.0);
  calibration_right.accel_offset_z = pref.getFloat("raz",0.0);

  // LEFT
  calibration_left.gyro_offset_x = pref.getFloat("lgx",0.0);
  calibration_left.gyro_offset_y = pref.getFloat("lgy",0.0);
  calibration_left.gyro_offset_z = pref.getFloat("lgz",0.0);

  calibration_left.accel_offset_x = pref.getFloat("lax",0.0);
  calibration_left.accel_offset_y = pref.getFloat("lay",0.0);
  calibration_left.accel_offset_z = pref.getFloat("laz",0.0);

  pref.end();
  return true;
}

bool calibrationExists(){
  pref.begin("mpu_calibration", true);

  bool exists = pref.isKey("rgx") &&
                pref.isKey("rgy") &&
                pref.isKey("rgz") &&
                pref.isKey("rax") &&
                pref.isKey("ray") &&
                pref.isKey("raz") &&
                pref.isKey("lgx") &&
                pref.isKey("lgy") &&
                pref.isKey("lgz") &&
                pref.isKey("lax") &&
                pref.isKey("lay") &&
                pref.isKey("laz");
                
  pref.end();
  return exists;
}

void checkRecalibrationCommands(){
  if (Serial.available()>0){
    char command = Serial.read();
    if (command == 'C' || command == 'c'){
      Serial.println("Recalibrating both IMUs Accelerometer and Gyroscope");

      calibrateBothGyroscopes();
      calibrateBothAccelerometers();
      saveCalibration();
    }
    else if (command == 'P' || command == 'p'){
      Serial.println("Recalibrating Posture");

      calibrateBothPostures();
    }
    else if (command == 'G' || command == 'g'){
      Serial.println("Recalibrating Gyroscope");

      calibrateBothGyroscopes();
      saveCalibration();
    }
    else if (command == 'A' || command == 'a'){
      Serial.println("Recalibrating Accelerometer");
      calibrateBothAccelerometers();
      saveCalibration();
    }
  }
}

void getCalibratedAcceleration(Adafruit_MPU6050 &mpu, IMU_calibration &calibration, float &x_accel, float &y_accel, float &z_accel){
  sensors_event_t accel;
  sensors_event_t gyro; 
  sensors_event_t temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // original accelerometer readings
  x_accel = (accel.acceleration.x - calibration.accel_offset_x) * calibration.accel_scale_x;
  y_accel = (accel.acceleration.y - calibration.accel_offset_y) * calibration.accel_scale_y;
  z_accel = (accel.acceleration.z - calibration.accel_offset_z) * calibration.accel_scale_z;

}

void getOrientation(Adafruit_MPU6050 &mpu, IMU_calibration &calibration, float &pitch, float &roll) {
  float x_accel, y_accel, z_accel;

  getCalibratedAcceleration(mpu, calibration, x_accel, y_accel, z_accel);
  
  pitch = atan2(-x_accel, sqrt(y_accel*y_accel + z_accel*z_accel)) * 180.0/PI;
  roll = atan2(z_accel,y_accel) * 180.0 /PI;
}

void calibrateBothPostures() {
  float right_sum_pitch = 0.0;
  float right_sum_roll = 0.0;

  float left_sum_pitch = 0.0;
  float left_sum_roll = 0.0;

  Serial.println("POSTURE CALIBRATION");
  Serial.println("Sit upright with both shoulders at the same height - engage your core and look ahead");
  delay(3000);

  for (int i = 0; i<CALIBRATION_SAMPLES; i++){
    float pitch_right = 0.0; 
    float roll_right = 0.0;
    float pitch_left = 0.0; 
    float roll_left = 0.0;

    getOrientation(mpu_right, calibration_right, pitch_right, roll_right);
    getOrientation(mpu_left, calibration_left, pitch_left, roll_left);

    right_sum_pitch += pitch_right;
    right_sum_roll += roll_right;

    left_sum_pitch += pitch_left;
    left_sum_roll += roll_left;
    delay(10);
  }

  calibration_right.pitch_offset = right_sum_pitch / CALIBRATION_SAMPLES;
  calibration_right.roll_offset = right_sum_roll / CALIBRATION_SAMPLES;

  calibration_left.pitch_offset = left_sum_pitch / CALIBRATION_SAMPLES;
  calibration_left.roll_offset = left_sum_roll / CALIBRATION_SAMPLES;

  Serial.println("Both sensor postural calibrations are complete");
  delay(500);
  Serial.print("RIGHT PITCH avg: ");
  Serial.println(calibration_right.pitch_offset);
  Serial.print("RIGHT ROLL avg: ");
  Serial.println(calibration_right.roll_offset);

  Serial.print("LEFT PITCH avg: ");
  Serial.println(calibration_left.pitch_offset);
  Serial.print("LEFT ROLL avg: ");
  Serial.println(calibration_left.roll_offset);
  
}

void checkPosture() {
  float right_pitch = 0.0; 
  float right_roll = 0.0;
  float left_pitch = 0.0; 
  float left_roll = 0.0;

  getOrientation(mpu_right, calibration_right, right_pitch, right_roll);
  getOrientation(mpu_left, calibration_left, left_pitch, left_roll);

  // calcualte chages from sensors internal reference
  float right_pitch_change = right_pitch - calibration_right.pitch_offset;
  float right_roll_change = right_roll - calibration_right.roll_offset;
  float left_pitch_change = left_pitch - calibration_left.pitch_offset;
  float left_roll_change = left_roll - calibration_left.roll_offset;

  // forward/backward lean
  float average_pitch_change = (right_pitch_change + left_pitch_change) / 2.0;
  float average_roll_change = (right_roll_change + left_roll_change) / 2.0;

  float pitch_difference = right_pitch_change - left_pitch_change;
  float roll_difference = right_roll_change - left_roll_change;

  if (fabs(average_pitch_change) > FORWARD_LEAN_THRESHOLD){
    Serial.println("FORWARD LEAN DETECTED");
  }

  if (fabs(average_roll_change) > SIDE_LEAN_THRESHOLD){
    Serial.println("SIDE LEAN DETECTED");
    digitalWrite(VIBRATIONAL_MOTOR_PIN, HIGH);
    delay(1000);
    digitalWrite(VIBRATIONAL_MOTOR_PIN, LOW);
  }

  Serial.print("Right Pitch change: ");
  Serial.println(right_pitch_change);
  Serial.print("Right Roll change: ");
  Serial.println(right_roll_change);

  Serial.print("Left Pitch change: ");
  Serial.println(left_pitch_change);
  Serial.print("Left Roll change: ");
  Serial.println(left_roll_change);

  Serial.print("Average Pitch change: ");
  Serial.println(average_pitch_change);
  Serial.print("Pitch difference: ");
  Serial.println(pitch_difference);
  Serial.print("Roll difference: ");
  Serial.println(roll_difference);
}

void setup() {
  Serial.begin(115200);
  delay(10000);
  pinMode(VIBRATIONAL_MOTOR_PIN, OUTPUT);
  digitalWrite(VIBRATIONAL_MOTOR_PIN, LOW);
  Serial.println("Starting up IMU");
  //Configure I2C pins
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize IMU
  if(!mpu_right.begin(MPU_ADDRESS_RIGHT)){
   Serial.println("Error: MPU6050 on RIGHT SIDE not found!");
    while (true){
      delay(10);
    }
  }
  if(!mpu_left.begin(MPU_ADDRESS_LEFT)){
   Serial.println("Error: MPU6050 on LEFT SIDE not found!");
    while (true){
      delay(10);
    }
  }
  Serial.println("MPU6050 connected!");

  mpu_right.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu_right.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu_right.setFilterBandwidth(MPU6050_BAND_21_HZ); // might want to increase this paper has it at 100Hz

  mpu_left.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu_left.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu_left.setFilterBandwidth(MPU6050_BAND_21_HZ); // might want to increase this paper has it at 100Hz

  if (!calibrationExists()){
    Serial.println("NO STORED CALIBRATION FOUND");
    Serial.println("Starting sensor calibrations");
    calibrateBothGyroscopes();
    calibrateBothAccelerometers();
    saveCalibration();
  }
  else{
    Serial.println("Loading stored sensor offsets");
    if (loadCalibration()){
      Serial.println("Stored sensor calibrations loaded!");
    }
  }
  
  Serial.println("Starting Posture Calibration");
  calibrateBothPostures();
  delay(2000);

  Serial.println("BOTH SENSORS CALIBRATED - READY FOR USE!");
  delay(2000);
}

void loop() {
  checkPosture();
  checkRecalibrationCommands();
  delay(100);
}

//// KEEP - USED TO EVALUATE CALIBRATION OFFSETS
//void compareCalibrationReadings(){
//    sensors_event_t accel;
//  sensors_event_t gyro; 
//  sensors_event_t temp;
//  mpu_right.getEvent(&accel, &gyro, &temp);
// 
//
//  // calibrated gyroscope readings
//  float x_gyro = gyro.gyro.x - calibration_right.gyro_offset_x;
//  float y_gyro = gyro.gyro.y - calibration_right.gyro_offset_y;
//  float z_gyro = gyro.gyro.z - calibration_right.gyro_offset_z;
//
//  // original accelerometer readings
//  float x_accel = (accel.acceleration.x - calibration_right.accel_offset_x) * calibration_right.accel_scale_x;
//  float y_accel = (accel.acceleration.y - calibration_right.accel_offset_y) * calibration_right.accel_scale_y;
//  float z_accel = (accel.acceleration.z - calibration_right.accel_offset_z) * calibration_right.accel_scale_z;
//
//  // original gyroscope readings
//  Serial.print("Gyro X Original: ");
//  Serial.print(gyro.gyro.x);
//  Serial.print(" Gyro Y Original: ");
//  Serial.print(gyro.gyro.y);
//  Serial.print(" Gyro Z Original: ");
//  Serial.println(gyro.gyro.z);
//
//  // calibrated gyroscope readings
//  Serial.print("Gyro X Calibrated: ");
//  Serial.print(x_gyro);
//  Serial.print(" Gyro Y Calibrated: ");
//  Serial.print(y_gyro);
//  Serial.print(" Gyro Z Calibrated: ");
//  Serial.println(z_gyro);
//
//  // original accelerometer readings
//  Serial.print("Accel X Original: ");
//  Serial.print(accel.acceleration.x);
//  Serial.print(" Accel Y Original: ");
//  Serial.print(accel.acceleration.y);
//  Serial.print(" Accel Z Original: ");
//  Serial.println(accel.acceleration.z);
//
//  // calibrated accelerometer readings
//  Serial.print("Accel X Calibrated: ");
//  Serial.print(x_accel);
//  Serial.print(" Accel Y Calibrated: ");
//  Serial.print(y_accel);
//  Serial.print(" Accel Z Calibrated: ");
//  Serial.println(z_accel);
//  delay(50);