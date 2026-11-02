
#include <Wire.h>
#include <Adafruit_AS5600.h>
#include <math.h>
#include "Encoder.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// USER SETTINGS

// Operating modes
const int STEP_MODE = 0;
const int TRAJECTORY_MODE = 1;

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
const float MAX_ELBOW_ANGLE = 2.44346;
const float MIN_ELBOW_ANGLE = 0.0;

// SAFETY INPUTS
const int ESTOP_PIN = 2;
const int START_PIN = 3;

// GLOBAL VARIABLES

const float pi = 3.141593;

float Motor_Duty = 0.0;

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


// BLE SETTINGS

#define SERVICE_UUID \
"12345678-1234-1234-1234-1234567890AB"

#define REFERENCE_UUID \
"12345678-1234-1234-1234-1234567890AC"

#define FEEDBACK_UUID \
"12345678-1234-1234-1234-1234567890AD"

BLEServer *BLE_Server = NULL;

BLECharacteristic *BLE_Reference_Characteristic = NULL;
BLECharacteristic *BLE_Feedback_Characteristic = NULL;

volatile bool BLE_Connected = false;

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


// ENCODER FUNCTIONS

float Read_Encoder()
{
    return (float)getSensorAngle();
}


// BLE REFERENCE CALLBACKS

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
        }
    }
};


// BLE SERVER CALLBACKS

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


// INITIALIZE BLE

void Initialize_BLE()
{
    BLEDevice::init("Exoskeleton_ESP32");

    BLE_Server = BLEDevice::createServer();

    BLE_Server->setCallbacks(new ServerCallbacks());

    BLEService *BLE_Service =
        BLE_Server->createService(SERVICE_UUID);


    // Python -> ESP32: Reference angle

    BLE_Reference_Characteristic =
        BLE_Service->createCharacteristic(
            REFERENCE_UUID,
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_WRITE_NR
        );

    BLE_Reference_Characteristic->setCallbacks(
        new ReferenceCallbacks()
    );


    // ESP32 -> Python: Actual encoder angle

    BLE_Feedback_Characteristic =
        BLE_Service->createCharacteristic(
            FEEDBACK_UUID,
            BLECharacteristic::PROPERTY_NOTIFY
        );

    BLE_Feedback_Characteristic->addDescriptor(
        new BLE2902()
    );


    BLE_Service->start();

    BLEAdvertising *advertising =
        BLEDevice::getAdvertising();

    advertising->addServiceUUID(SERVICE_UUID);

    advertising->setScanResponse(true);

    BLEDevice::startAdvertising();

    Serial.println("BLE started.");
    Serial.println("Device name: Exoskeleton_ESP32");
}


// GET REFERENCE STATUS

bool Get_Reference_Status(
    float &reference,
    unsigned long &lastTime
)
{
    bool hasReference;

    portENTER_CRITICAL(&ReferenceMux);

    reference = Reference_Angle;
    lastTime = Last_Reference_Time;
    hasReference = Has_Reference;

    portEXIT_CRITICAL(&ReferenceMux);

    return hasReference;
}


// SEND ENCODER FEEDBACK TO PYTHON

void Send_Encoder_Feedback(float feedbackAngle)
{
    if (!BLE_Connected || BLE_Feedback_Characteristic == NULL)
    {
        return;
    }

    // Send the measured elbow angle as a binary 32-bit float (radians).
    BLE_Feedback_Characteristic->setValue(
        (uint8_t *)&feedbackAngle,
        sizeof(feedbackAngle)
    );

    BLE_Feedback_Characteristic->notify();
}


// PID FUNCTIONS

void CLStep()
{
    float error = Reference_Angle - Encoder_elbow_angle;
    Serial.print("Reference: ");
    Serial.println(Reference_Angle);
    Serial.print("Encoder Angle: ");
    Serial.println(Encoder_elbow_angle);

    if (fabs(error) <= ERROR_DEADBAND)
    {
        Motor_Duty = 0.0;

        CO[0] = 0.0;
        CO[1] = 0.0;
        CO[2] = 0.0;

        E[0] = 0.0;
        E[1] = 0.0;
        E[2] = 0.0;

        return;
    }

    E[0] = error;

    CO[0] = -C1 * CO[1] - C2 * CO[2]
            + E0 * E[0] + E1 * E[1] + E2 * E[2];

    Motor_Duty = CO[0] / V_in;

    CO[2] = CO[1];
    CO[1] = CO[0];

    E[2] = E[1];
    E[1] = E[0];
}


// CONTROLLER MANAGEMENT

void Reset_Controller()
{
    CO[0] = 0.0;
    CO[1] = 0.0;
    CO[2] = 0.0;

    E[0] = 0.0;
    E[1] = 0.0;
    E[2] = 0.0;

    Motor_Duty = 0.0;

    t = 0.0;
    Cnt = 0;

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

    Reset_Controller();

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

    Motor_Duty = 0.0;

    // Stop_Motor();
}


// INTERRUPTS

void IRAM_ATTR Timer_ISR()
{
    Timer_Ticks++;
}


void IRAM_ATTR STOP()
{
    Stop = true;
}


// SETUP

void setup()
{
    Serial.begin(115200);

    Initialize_BLE();

    // Initialize encoder
    if (!initializeEncoder())
    {
        Serial.println("Encoder initialization failed!");

        while (true)
        {
            delay(100);
        }
    }

    Encoder_elbow_angle = Read_Encoder();
    Serial.print("Initial encoder reading (assumed radians): ");
    Serial.println(Encoder_elbow_angle, 6);

    Ts = Period / 1000000.0;

    Freq = 1.0 / Ts;

    C1 = 0.0;
    C2 = 0.0;

    E0 = Kp + Kd * N;

    E1 = -Kp + Ki * Ts - 2.0 * Kd * N;

    E2 = Kd * N;

    // Initialize operating mode

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

    Serial.println("BLE feedback: 50 Hz, independent of controller state");
}


// MAIN LOOP

void loop()
{
    // SEND ENCODER FEEDBACK AT 50 Hz
    static unsigned long Last_Feedback_Time = 0;

    if (BLE_Connected && millis() - Last_Feedback_Time >= 20)
    {
        float measuredAngle = Read_Encoder();

        if (isfinite(measuredAngle))
        {
            Encoder_elbow_angle = measuredAngle;
            Send_Encoder_Feedback(measuredAngle);
        }
        else
        {
            Serial.println("Invalid encoder reading during BLE feedback!");
        }

        Last_Feedback_Time = millis();
    }

    float reference;

    unsigned long lastReferenceTime;

    bool hasReference =
        Get_Reference_Status(reference, lastReferenceTime);


    // AUTOMATICALLY START STEP MODE

    if (Mode == STEP_MODE && !ControllerRunning)
    {
        Start_Controller();
    }


    // AUTOMATICALLY START TRAJECTORY MODE

    if (Mode == TRAJECTORY_MODE && !ControllerRunning)
    {
        if (hasReference &&
            millis() - lastReferenceTime <= REFERENCE_TIMEOUT_MS)
        {
            Start_Controller();
        }
    }


    // CHECK FOR LOST REFERENCE STREAM

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


    // WAIT UNTIL CONTROLLER IS ENABLED

    if (!Go || !ControllerRunning)
    {
        return;
    }


    // CHECK PENDING TIMER EVENTS

    uint32_t pendingTicks;

    noInterrupts();

    pendingTicks = Timer_Ticks;

    Timer_Ticks = 0;

    interrupts();


    if (pendingTicks == 0)
    {
        return;
    }


    // CHECK FOR CONTROL LOOP OVERRUN

    if (pendingTicks > 1)
    {
        Serial.println("Control loop overrun!");

        Stop_Controller();

        return;
    }


    // READ ACTUAL ELBOW ANGLE

    Encoder_elbow_angle = Read_Encoder();

    if (!isfinite(Encoder_elbow_angle))
    {
        Serial.println("Invalid encoder reading!");

        Stop_Controller();

        return;
    }


    // CHECK ELBOW JOINT LIMITS

    if (Encoder_elbow_angle < MIN_ELBOW_ANGLE ||
        Encoder_elbow_angle > MAX_ELBOW_ANGLE)
    {
        Serial.println("Elbow angle limit exceeded!");

        Stop_Controller();

        return;
    }


    // SELECT REFERENCE INPUT

    if (Mode == STEP_MODE)
    {
        Reference_Angle = Reference_Input;
    }

    else
    {
        Reference_Angle = reference;
    }


    // RUN PID CONTROLLER

    CLStep();


    // SEND MOTOR COMMAND

    // Move_Motor(Motor_Duty);


    // UPDATE ELAPSED TIME

    Cnt++;

    t = Cnt * Ts;


    // STOP AFTER SPECIFIED DURATION IN STEP MODE

    if (Mode == STEP_MODE && t >= Time)
    {
        Serial.println("Step response completed.");

        Stop_Controller();
    }
}
