
#include <Wire.h>
#include <Adafruit_AS5600.h>
//#include <ESP32TimerInterrupt.h>
#include <math.h>
//#include <mcp_can.h>
//#include <SPI.h>
//#include "Encoder.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// USER SETTINGS

// Operating modes
const int STEP_MODE = 0;
const int TRAJECTORY_MODE = 1;

// Select operating mode here
int Mode = TRAJECTORY_MODE;

// Step input reference (radians)
float Reference_Input = 0.261799;  // 15 degrees

// PID settings
const float ERROR_DEADBAND = 0.0872665;
float Kp = 354.1268;
float Ki = 0.0;
float Kd = 10.6569;
float N = 100.0;

// Motor voltage
float V_in = 24.0;

// Controller timing
unsigned int Period = 1700;
float Time = 5.0;

// Elbow joint limits (radians)
const float MAX_ELBOW_ANGLE = 2.44346;  // 140 degrees
const float MIN_ELBOW_ANGLE = 0.0;

//// CAN SETTINGS
//
//const int SPI_CS_PIN = 10;
//MCP_CAN CAN(SPI_CS_PIN);
//
//const unsigned long MOTOR_CAN_ID = 0x01;

float Motor_Duty = 0.0;

// SAFETY INPUTS

const int ESTOP_PIN = 2;
const int START_PIN = 3;

// GLOBAL VARIABLES

const float pi = 3.141593;

// Controller state
volatile bool Go = false;
volatile bool Stop = false;
volatile bool StartRequested = false;

// Timer state
volatile uint32_t Timer_Ticks = 0;

hw_timer_t *ControlTimer = NULL;

// Controller status
bool ControllerRunning = false;

// Reference and encoder angles (radians)
float Reference_Angle = 0.0;
float Encoder_elbow_angle = 0.0;

#define SERVICE_UUID "12345678-1234-1234-1234-1234567890AB"
#define CHARACTERISTIC_UUID "12345678-1234-1234-1234-1234567890AC"

BLEServer *BLE_Server = NULL;
BLECharacteristic *BLE_Reference_Characteristic = NULL;

bool BLE_Connected = false;

portMUX_TYPE ReferenceMux = portMUX_INITIALIZER_UNLOCKED;

// Reference stream status
volatile bool Has_Reference = false;

unsigned long Last_Reference_Time = 0;
const unsigned long REFERENCE_TIMEOUT_MS = 250;

// PID variables
float CO[3] = {0.0, 0.0, 0.0};
float E[3] = {0.0, 0.0, 0.0};

float C1 = 0.0;
float C2 = 0.0;

float E0 = 0.0;
float E1 = 0.0;
float E2 = 0.0;

// Timing variables
float Ts = 0.0;
float Freq = 0.0;

float t = 0.0;
int Cnt = 0;

// CAN FUNCTIONS
//
//void Pack32(long value, byte data[4])
//{
//    data[0] = (byte)(value >> 24);
//    data[1] = (byte)(value >> 16);
//    data[2] = (byte)(value >> 8);
//    data[3] = (byte)value;
//}
//
//long Unpack32(byte data[4])
//{
//    return ((long)data[0] << 24) |
//           ((long)data[1] << 16) |
//           ((long)data[2] << 8) |
//           (long)data[3];
//}
////  ENCODER FUNCTIONS
//
//signed long int Read_Encoder()
//{
//  return = getSensorAngle();
//}
//
////  MOTOR FUNCTIONS
//
//void Ready_Motor()
//{
//    delay(100);
//}
//
//void Move_Motor(float duty)
//{
//    duty = constrain(duty, -0.95, 0.95);
//
//    long command = (long)(duty * 100000.0);
//
//    byte data[4];
//    Pack32(command, data);
//
//    CAN.sendMsgBuf(MOTOR_CAN_ID, 1, 4, data);
//}
//
//void Stop_Motor()
//{
//    Move_Motor(0.0);
//}

// REFERENCE INPUT FUNCTION

class ReferenceCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
      std::string received = pCharacteristic->getValue();
      String receivedValue = String(received.c_str());

      receivedValue.trim();

      if (receivedValue.length() == 0)
      {
        return;
      }

      const char *input = receivedValue.c_str();
      char *endPtr;

      float newReference = strtof(input, &endPtr);

      if (endPtr != input &&
          *endPtr == '\0' &&
          isfinite(newReference) &&
          newReference >= MIN_ELBOW_ANGLE &&
          newReference <= MAX_ELBOW_ANGLE)
      {
        portENTER_CRITICAL(&ReferenceMux);

        Reference_Angle = newReference;
        Last_Reference_Time = millis();
        Has_Reference = true;

        portEXIT_CRITICAL(&ReferenceMux);

        Serial.println(newReference);
      }
    }
};

class ServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
      BLE_Connected = true;
      Serial.println("Python connected via BLE!");
    }

    void onDisconnect(BLEServer *pServer)
    {
      BLE_Connected = false;
      Serial.println("Python disconnected!");

      BLEDevice::startAdvertising();
    }
};

void Initialize_BLE()
{
  BLEDevice::init("Exoskeleton_ESP32");

  BLE_Server = BLEDevice::createServer();
  BLE_Server->setCallbacks(new ServerCallbacks());

  BLEService *BLE_Service =
    BLE_Server->createService(SERVICE_UUID);

  BLE_Reference_Characteristic =
    BLE_Service->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_WRITE_NR
    );

  BLE_Reference_Characteristic->setCallbacks(
    new ReferenceCallbacks()
  );

  BLE_Service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();

  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);

  BLEDevice::startAdvertising();

  Serial.println("BLE started.");
  Serial.println("Device name: Exoskeleton_ESP32");
}

bool Get_Reference_Status(float &reference, unsigned long &lastTime)
{
  bool hasReference;

  portENTER_CRITICAL(&ReferenceMux);

  reference = Reference_Angle;
  lastTime = Last_Reference_Time;
  hasReference = Has_Reference;

  portEXIT_CRITICAL(&ReferenceMux);

  return hasReference;
}

//  PID FUNCTIONS

void CLStep()
{
  float error = Reference_Angle - Encoder_elbow_angle;
  if (fabs(error) <= ERROR_DEADBAND) {
    Motor_Duty = 0.0;

    // Reset PID history to prevent accumulated
    // controller output when assistance resumes
    CO[0] = 0.0;
    CO[1] = 0.0;
    CO[2] = 0.0;

    E[0] = 0.0;
    E[1] = 0.0;
    E[2] = 0.0;

    return;
  }

  // Error is outside deadband then activate controller
  E[0] = error;

  CO[0] = -C1 * CO[1] - C2 * CO[2]
          + E0 * E[0] + E1 * E[1] + E2 * E[2];

  Motor_Duty = CO[0] / V_in;

  CO[2] = CO[1];
  CO[1] = CO[0];

  E[2] = E[1];
  E[1] = E[0];
  Serial.println(Motor_Duty);
}

// CONTROLLER MANAGEMENT

void Reset_Controller()
{
  // Reset PID history
  CO[0] = 0.0;
  CO[1] = 0.0;
  CO[2] = 0.0;

  E[0] = 0.0;
  E[1] = 0.0;
  E[2] = 0.0;

  Motor_Duty = 0.0;

  // Reset timing
  t = 0.0;
  Cnt = 0;

  // Clear pending timer events
  noInterrupts();
  Timer_Ticks = 0;
  interrupts();
}

void Start_Controller()
{
  if (Stop || digitalRead(ESTOP_PIN) == HIGH)
  {
    return;
  }

  if (Mode == STEP_MODE)
  {
    if (!isfinite(Reference_Input) ||
        Reference_Input < MIN_ELBOW_ANGLE ||
        Reference_Input > MAX_ELBOW_ANGLE)
    {
      return;
    }

    Reference_Angle = Reference_Input;
  }
  else if (Mode == TRAJECTORY_MODE)
  {
    float reference;
    unsigned long lastTime;

    if (!Get_Reference_Status(reference, lastTime) ||
        millis() - lastTime > REFERENCE_TIMEOUT_MS)
    {
      return;
    }
  }
  else
  {
    return;
  }

  // Reset PID history
  Reset_Controller();

  // Start timer
  timerWrite(ControlTimer, 0);

  Go = true;
  ControllerRunning = true;

  timerStart(ControlTimer);

  Serial.println("Controller started.");
}

void Stop_Controller()
{
  Go = false;

  timerStop(ControlTimer);

  ControllerRunning = false;

  noInterrupts();
  Timer_Ticks = 0;
  interrupts();

  //    Stop_Motor();
}

//  INTERRUPTS
void IRAM_ATTR Timer_ISR()
{
  Timer_Ticks++;
}

void IRAM_ATTR STOP()
{
  Stop = true;
}

//  SETUP
void setup()
{
  Serial.begin(115200);

  Initialize_BLE();

  //    pinMode(ESTOP_PIN, INPUT);
  //    pinMode(START_PIN, INPUT);

  //    attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), STOP, RISING);

  //    // Initialize SPI and CAN
  //    SPI.begin();
  //
  //    while (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK)
  //    {
  //        Serial.println("CAN initialization failed!");
  //        delay(100);
  //    }
  //
  //    CAN.setMode(MCP_NORMAL);

  //    // Initialize encoder
  //    if (!initializeEncoder())
  //    {
  //        Serial.println("Encoder initialization failed!");
  //        Stop_Motor();
  //
  //        while (true)
  //        {
  //            delay(100);
  //        }
  //    }

  //    Ready_Motor();
  //    Stop_Motor();

  Ts = Period / 1000000.0;
  Freq = 1.0 / Ts;

  C1 = 0.0;
  C2 = 0.0;

  E0 = Kp + Kd * N;
  E1 = -Kp + Ki * Ts - 2.0 * Kd * N;
  E2 = Kd * N;

  // Initialize selected operating mode
  if (Mode == STEP_MODE)
  {
    Reference_Angle = Reference_Input;

    Serial.println("Operating mode: STEP INPUT");
  }
  else
  {
    Has_Reference = false;

    Serial.println("Operating mode: REFERENCE TRAJECTORY");
    Serial.println("Waiting for reference angles...");
  }

  // Initialize control timer
  ControlTimer = timerBegin(0, 80, true);

  timerAttachInterrupt(
    ControlTimer,
    &Timer_ISR,
    true
  );

  timerAlarmWrite(
    ControlTimer,
    Period,
    true
  );

  timerAlarmEnable(ControlTimer);

  timerStop(ControlTimer);

  Serial.println("Exoskeleton controller initialized.");
  Serial.print("Sampling period: ");
  Serial.print(Ts);
  Serial.println(" seconds");

  Serial.print("Sampling frequency: ");
  Serial.print(Freq);
  Serial.println(" Hz");
}

//  MAIN LOOP
void loop()
{
  float reference;
  unsigned long lastReferenceTime;

  bool hasReference =
    Get_Reference_Status(reference, lastReferenceTime);

  //    // Emergency stop handling (placeholder for now)
  //    if (Stop || digitalRead(ESTOP_PIN) == HIGH)
  //    {
  //        Stop = true;
  //
  //        Stop_Controller();
  //
  //        return;
  //    }

  // Automatically start step mode
  if (Mode == STEP_MODE && !ControllerRunning)
  {
    Start_Controller();
  }

  // Automatically start trajectory mode once a valid reference angle has been received
  if (Mode == TRAJECTORY_MODE && !ControllerRunning)
  {
    if (hasReference &&
        millis() - lastReferenceTime <= REFERENCE_TIMEOUT_MS)
    {
      Start_Controller();
    }
  }

  // Check for lost reference stream
  if (Mode == TRAJECTORY_MODE && ControllerRunning)
  {
    if (!hasReference ||
        millis() - lastReferenceTime > REFERENCE_TIMEOUT_MS)
    {
      Serial.println("Reference stream lost!");

      Stop_Controller();

      portENTER_CRITICAL(&ReferenceMux);
      Has_Reference = false;
      portEXIT_CRITICAL(&ReferenceMux);

      return;
    }
  }

  // Wait until controller is enabled
  if (!Go || !ControllerRunning)
  {
    return;
  }

  // Check pending timer events
  uint32_t pendingTicks;

  noInterrupts();

  pendingTicks = Timer_Ticks;

  Timer_Ticks = 0;

  interrupts();

  if (pendingTicks == 0)
  {
    return;
  }

  // More than one pending tick indicates an overrun.
  // Stop rather than executing delayed control updates.

  if (pendingTicks > 1)
  {
    Serial.println("Control loop overrun!");

    Stop_Controller();

    return;
  }


  //    // Read actual elbow angle from encoder
  //    Encoder_elbow_angle = Read_Encoder();
  //
  //    if (!isfinite(Encoder_elbow_angle))
  //    {
  //        Serial.println("Invalid encoder reading!");
  //
  //        Stop_Controller();
  //
  //        return;
  //    }
  //
  //    // Check elbow joint limits
  //    if (Encoder_elbow_angle < MIN_ELBOW_ANGLE ||
  //        Encoder_elbow_angle > MAX_ELBOW_ANGLE)
  //    {
  //        Serial.println("Elbow angle limit exceeded!");
  //
  //        Stop_Controller();
  //
  //        return;
  //    }

  // Select reference input
  if (Mode == STEP_MODE)
  {
    // Fixed reference
    Reference_Angle = Reference_Input;
  }
  else
  {
    Reference_Angle = reference;
  }

  // In trajectory mode, Reference_Angle is updated
  // continuously by BLE.

  // Calculate PID output
  //    CLStep();

  // Final safety check
  //    if (Stop || digitalRead(ESTOP_PIN) == HIGH)
  //    {
  //        Stop = true;
  //
  //        Stop_Controller();
  //
  //        return;
  //    }


  //    // Send motor command
  //    Move_Motor(Motor_Duty);

  // Update elapsed time
  Cnt++;

  t = Cnt * Ts;

  // Stop after specified duration in step mode
  if (Mode == STEP_MODE && t >= Time)
  {
    Serial.println("Step response completed.");

    Stop_Controller();
  }
}
