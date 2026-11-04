// import libraries
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Initialization
Adafruit_MPU6050 mpu_right;
//Adafruit_MPU6050 mpu_left;

// CONSTANTS
const int SDA_PIN = A6;
const int SCL_PIN = A7;
const int MPU_ADDRESS = 0x68;
//const int VIBRATIONAL_MOTOR_PIN = 4; // update when motor is in place
// calibration constants
const float FORWARD_LEAN_THRESHOLD = 10.0;
const float SIDE_LEAN_THRESHOLD = 10.0;
const int CALIBRATION_SAMPLES = 500;
const float ACCEL_X = 9.81;
const float ACCEL_Y = 0.0;
const float ACCEL_Z = 0.0;

struct IMU_calibration {
  float gyro_offset_x = 0.0;
  float gyro_offset_y = 0.0;
  float gyro_offset_z = 0.0;

  float accel_offset_x = 0.0;
  float accel_offset_y = 0.0;
  float accel_offset_z = 0.0;

  float accel_scale_x = 0.0;
  float accel_scale_y = 0.0;
  float accel_scale_z = 0.0;

  float pitch_offset = 0.0;
  float roll_offset = 0.0;
};

IMU_calibration calibration_right;


void getGyroscopeCalibration(IMU_calibration &calibration){
  // initialize temp variables for averaging final calibration
  float sum_x = 0.0;
  float sum_y = 0.0;
  float sum_z = 0.0;
  
  sensors_event_t accel;
  sensors_event_t gyro; 
  sensors_event_t temp;

  Serial.println("Calibrating Gyroscope in 3 secs keep completely still");
  delay(3000);

  for (int i=0; i<CALIBRATION_SAMPLES; i++){
    mpu_right.getEvent(&accel, &gyro, &temp);

    sum_x += gyro.gyro.x;
    sum_y += gyro.gyro.y;
    sum_z += gyro.gyro.z;

    delay(10);
  }

  calibration.gyro_offset_x = sum_x/CALIBRATION_SAMPLES;
  calibration.gyro_offset_y = sum_y/CALIBRATION_SAMPLES;
  calibration.gyro_offset_z = sum_z/CALIBRATION_SAMPLES;

  Serial.println("Gyroscope Calibration Complete");
  delay(500);
  Serial.print("X OFFSET: ");
  Serial.println(calibration.gyro_offset_x);
  Serial.print("Y OFFSET: ");
  Serial.println(calibration.gyro_offset_y);
  Serial.print("Z OFFSET: ");
  Serial.println(calibration.gyro_offset_z);
}

void getAccelerometerCalibration(IMU_calibration &calibration){
  // initialize temp variables for averaging final calibration
  float sum_x = 0.0;
  float sum_y = 0.0;
  float sum_z = 0.0;
  
  sensors_event_t accel;
  sensors_event_t gyro; 
  sensors_event_t temp;

  Serial.println("Calibrating Accelerometer in 3 secs keep completely still");
  delay(3000);

  for (int i=0; i<CALIBRATION_SAMPLES; i++){
    mpu_right.getEvent(&accel, &gyro, &temp);
  
    sum_x += accel.acceleration.x;
    sum_y += accel.acceleration.y;
    sum_z += accel.acceleration.z;
    
    delay(10);
  }
  float avg_x = sum_x/CALIBRATION_SAMPLES;
  float avg_y = sum_y/CALIBRATION_SAMPLES;
  float avg_z = sum_z/CALIBRATION_SAMPLES;

  // Adjust for direction and gravity

  calibration.accel_offset_x = avg_x - ACCEL_X;
  calibration.accel_offset_y = avg_y - ACCEL_Y;
  calibration.accel_offset_z = avg_z - ACCEL_Z;

  // UPDATE SOON: need to perform one time calibration on each sensor to get these values
  calibration.accel_scale_x = 1.0; 
  calibration.accel_scale_y = 1.0; 
  calibration.accel_scale_z = 1.0; 

  Serial.println("Accelerometer calibrations complete");
  delay(500);
  Serial.print("X avg: ");
  Serial.println(avg_x);
  Serial.print("Y avg: ");
  Serial.println(avg_y);
  Serial.print("Z avg: ");
  Serial.println(avg_z);

  delay(500);
  Serial.print("X OFFSET: ");
  Serial.println(calibration.accel_offset_x);
  Serial.print("Y OFFSET: ");
  Serial.println(calibration.accel_offset_y);
  Serial.print("Z OFFSET: ");
  Serial.println(calibration.accel_offset_z);
}



//
//void getOrientation(float &pitch, float&roll) {
//  
//}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting up IMU");

  //Configure I2C pins
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize IMU
  if(!mpu_right.begin(MPU_ADDRESS)){
   Serial.println("Error: MPU6050 not found!");
    while (true){
      delay(10);
    }
  }
  Serial.println("MPU6050 connected!");

  mpu_right.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu_right.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu_right.setFilterBandwidth(MPU6050_BAND_21_HZ); // might want to increase this paper has it at 100Hz

  Serial.println("Sit up straight, calibration starting soon...");
  delay(3000);
  Serial.println("Starting Calibration...");
  Serial.println("Starting Gyroscope Calibration");
  getGyroscopeCalibration(calibration_right);

  delay(3000);
  Serial.println("Starting Accelerometer Calibration");
  getAccelerometerCalibration(calibration_right);

  delay(2000);
}



void loop() {
  sensors_event_t accel;
  sensors_event_t gyro; 
  sensors_event_t temp;
  mpu_right.getEvent(&accel, &gyro, &temp);
 

  // calibrated gyroscope readings
  float x_gyro = gyro.gyro.x - calibration_right.gyro_offset_x;
  float y_gyro = gyro.gyro.y - calibration_right.gyro_offset_y;
  float z_gyro = gyro.gyro.z - calibration_right.gyro_offset_z;

  // original accelerometer readings
  float x_accel = (accel.acceleration.x - calibration_right.accel_offset_x) * calibration_right.accel_scale_x;
  float y_accel = (accel.acceleration.y - calibration_right.accel_offset_y) * calibration_right.accel_scale_y;
  float z_accel = (accel.acceleration.z - calibration_right.accel_offset_z) * calibration_right.accel_scale_z;

  // original gyroscope readings
  Serial.print("Gyro X Original: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" Gyro Y Original: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" Gyro Z Original: ");
  Serial.println(gyro.gyro.z);

  // calibrated gyroscope readings
  Serial.print("Gyro X Calibrated: ");
  Serial.print(x_gyro);
  Serial.print(" Gyro Y Calibrated: ");
  Serial.print(y_gyro);
  Serial.print(" Gyro Z Calibrated: ");
  Serial.println(z_gyro);

  // original accelerometer readings
  Serial.print("Accel X Original: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" Accel Y Original: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" Accel Z Original: ");
  Serial.println(accel.acceleration.z);

  // calibrated accelerometer readings
  Serial.print("Accel X Calibrated: ");
  Serial.print(x_accel);
  Serial.print(" Accel Y Calibrated: ");
  Serial.print(y_accel);
  Serial.print(" Accel Z Calibrated: ");
  Serial.println(z_accel);
  delay(50);
}