
#include <Wire.h>
#include <Adafruit_AS5600.h>
#include <math.h>
#include "Encoder.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "driver/twai.h"

// USER SETTINGS

// Communication protocol toggle
enum CommMode {
    COMM_SERIAL = 0,
    COMM_CAN = 1
};
CommMode CommProtocol = COMM_SERIAL;
// Motor CAN & Serial Settings
const uint8_t MOTOR_CAN_ID = 1;       // Default CubeMars motor CAN ID 
const gpio_num_t CAN_TX_PIN = GPIO_NUM_4; // Connect to CAN Transceiver TXD (D2 on Nano ESP32 = GPIO 5, D4 = GPIO 7, etc)
const gpio_num_t CAN_RX_PIN = GPIO_NUM_5; // Connect to CAN Transceiver RXD
#define MotorSerial Serial1           // Nano ESP32 Hardware Serial1 (TX1 / RX1 pins)

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


// CUBEMARS SERIAL CRC16 TABLE
const unsigned short crc16_tab[] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};
unsigned short calculate_crc16(unsigned char *buf, unsigned int len)
{
    unsigned int i;
    unsigned short cksum = 0;
    for (i = 0; i < len; i++)
    {
        cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
    }
    return cksum;
}

// MOTOR COMMUNICATION FUNCTIONS (SERIAL & CAN)
void Initialize_Motor_Comm()
{
    if (CommProtocol == COMM_SERIAL)
    {
        MotorSerial.begin(115200);
        Serial.println("Motor communication initialized: SERIAL (UART @ 115200 baud)");
    }
    else if (CommProtocol == COMM_CAN)
    {
        twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
        twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); // CubeMars default 1 Mbps
        twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK && twai_start() == ESP_OK)
        {
            Serial.println("Motor communication initialized: CAN (TWAI @ 1 Mbps)");
        }
        else
        {
            Serial.println("Failed to initialize CAN (TWAI) driver!");
            while (true)
            {
                delay(100);
            }
        }
    }
}

void Send_Duty_Serial(float duty)
{
    int32_t duty_scaled = (int32_t)(duty * 100000.0f);

    uint8_t payload[5];
    payload[0] = 5; // COMM_SET_DUTY Command ID
    payload[1] = (duty_scaled >> 24) & 0xFF;
    payload[2] = (duty_scaled >> 16) & 0xFF;
    payload[3] = (duty_scaled >> 8) & 0xFF;
    payload[4] = duty_scaled & 0xFF;

    unsigned short crc = calculate_crc16(payload, 5);

    MotorSerial.write(0x02); // Header
    MotorSerial.write(0x05); // Payload Length
    for (int i = 0; i < 5; i++)
    {
        MotorSerial.write(payload[i]);
    }
    MotorSerial.write((crc >> 8) & 0xFF);
    MotorSerial.write(crc & 0xFF);
    MotorSerial.write(0x03); // Tail
}

void Send_Duty_CAN(float duty)
{
    // CubeMars Servo Duty Cycle Mode uses Extended 29-bit CAN ID:
    // Packet ID = (Control_Mode_ID << 8) | Motor_ID, where Duty Cycle Mode ID = 0
    int32_t duty_scaled = (int32_t)(duty * 100000.0f);

    twai_message_t tx_msg;
    tx_msg.identifier = (0UL << 8) | MOTOR_CAN_ID;
    tx_msg.extd = 1; // Extended 29-bit frame required by CubeMars Servo protocol
    tx_msg.rtr = 0;
    tx_msg.data_length_code = 4;

    tx_msg.data[0] = (duty_scaled >> 24) & 0xFF;
    tx_msg.data[1] = (duty_scaled >> 16) & 0xFF;
    tx_msg.data[2] = (duty_scaled >> 8) & 0xFF;
    tx_msg.data[3] = duty_scaled & 0xFF;

    twai_transmit(&tx_msg, pdMS_TO_TICKS(1));
}

void Move_Motor(float duty)
{
    // Clamp duty cycle between -0.95 (-95%) and 0.95 (+95%) for safety
    float safe_duty = constrain(duty, -0.95f, 0.95f);

    if (CommProtocol == COMM_SERIAL)
    {
        Send_Duty_Serial(safe_duty);
    }
    else if (CommProtocol == COMM_CAN)
    {
        Send_Duty_CAN(safe_duty);
    }
}

void Stop_Motor()
{
    Motor_Duty = 0.0f;
    Move_Motor(0.0f);
}


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
    // Serial.print("Reference: ");
    // Serial.println(Reference_Angle);
    // Serial.print("Encoder Angle: ");
    // Serial.println(Encoder_elbow_angle);

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

    Initialize_Motor_Comm();
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
