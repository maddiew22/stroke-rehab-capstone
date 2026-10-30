
#include <Wire.h>
#include <Adafruit_AS5600.h>
#include <ESP32TimerInterrupt.h>
#include <math.h>
#include <mcp_can.h>
#include <SPI.h>
#include "Encoder.h"

//  USER SETTINGS 

int Mode = 1;
int Step_Input = 1;

float Kp = 2.2881;
float Ki = 0.0;
float Kd = 0.0582836;
float N = 100.0;

float Reference_Input = 0.0872665;

float H_Gain = 651.9;
float I_Gain = 651.9;

float V_in = 24.0;

unsigned int Period = 5000;
float Time = 5.0;

const float MAX_ELBOW_ANGLE = 140.0;

//  CAN SETTINGS 

const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);

const unsigned long MOTOR_CAN_ID = 0x01;

float Motor_Duty = 0.0;

//  SAFETY INPUTS 

const int ESTOP_PIN = 2;
const int START_PIN = 3;

//  GLOBAL VARIABLES 

const float pi = 3.141593;

volatile bool Go = false;
volatile bool Timer_Go = false;
volatile bool Stop = false;
volatile bool Stop_bit = false;

signed long int Reference_Input_Encoder = 0;
signed long int Encoder_Count = 0;

float CO[3] = {0, 0, 0};
float E[3] = {0, 0, 0};

float C1 = 0.0;
float C2 = 0.0;
float E0 = 0.0;
float E1 = 0.0;
float E2 = 0.0;

float Ts = 0.0;
float Freq = 0.0;

float t = 0.0;
int Cnt = 0;

hw_timer_t *ControlTimer = NULL;

//  CAN FUNCTIONS 

void Pack32(long value, byte data[4])
{
    data[0] = (byte)(value >> 24);
    data[1] = (byte)(value >> 16);
    data[2] = (byte)(value >> 8);
    data[3] = (byte)value;
}

long Unpack32(byte data[4])
{
    return ((long)data[0] << 24) |
           ((long)data[1] << 16) |
           ((long)data[2] << 8) |
           (long)data[3];
}

//  ENCODER FUNCTIONS 

signed long int Read_Encoder()
{
    float elbow_angle = getSensorAngle();
    float angle_rad = elbow_angle * pi / 180.0;

    return (signed long int)(angle_rad * H_Gain);
}

//  MOTOR FUNCTIONS 

void Ready_Motor()
{
    delay(100);
}

void Move_Motor(float duty)
{
    duty = constrain(duty, -0.95, 0.95);

    long command = (long)(duty * 100000.0);

    byte data[4];
    Pack32(command, data);

    CAN.sendMsgBuf(MOTOR_CAN_ID, 1, 4, data);
}

void Stop_Motor()
{
    Move_Motor(0.0);
}

//  PID FUNCTIONS 

void CLStep()
{
    E[0] = Reference_Input_Encoder - Encoder_Count;

    CO[0] = -C1 * CO[1] - C2 * CO[2]
            + E0 * E[0] + E1 * E[1] + E2 * E[2];

    Motor_Duty = CO[0] / V_in;

    CO[2] = CO[1];
    CO[1] = CO[0];

    E[2] = E[1];
    E[1] = E[0];
}

//  INTERRUPTS 
void IRAM_ATTR Timer_ISR()
{
    Timer_Go = true;
}

void IRAM_ATTR STOP()
{
    Stop = true;
}

void IRAM_ATTR START()
{
    if (!Stop && digitalRead(ESTOP_PIN) == LOW)
    {
        Go = true;
    }
}

//  SETUP 
void setup()
{
    Serial.begin(115200);

    pinMode(ESTOP_PIN, INPUT);
    pinMode(START_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), STOP, RISING);
    attachInterrupt(digitalPinToInterrupt(START_PIN), START, RISING);

    SPI.begin();

    while (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK)
    {
        Serial.println("CAN initialization failed!");
        delay(100);
    }

    CAN.setMode(MCP_NORMAL);

    if (!initializeEncoder())
    {
        Serial.println("Encoder initialization failed!");
        Stop_Motor();

        while (true)
        {
            delay(100);
        }
    }

    Ready_Motor();

    Ts = Period / 1000000.0;
    Freq = 1.0 / Ts;

    C1 = 0.0;
    C2 = 0.0;

    E0 = Kp + Kd * N;
    E1 = -Kp + Ki * Ts - 2.0 * Kd * N;
    E2 = Kd * N;

    Reference_Input_Encoder =
        (signed long int)(Reference_Input * I_Gain);

    Serial.println("Exoskeleton controller initialized.");
    Serial.print("Sampling period: ");
    Serial.print(Ts);
    Serial.println(" seconds");

    Serial.print("Sampling frequency: ");
    Serial.print(Freq);
    Serial.println(" Hz");

    ControlTimer = timerBegin(1000000);
    timerAttachInterrupt(ControlTimer, &Timer_ISR);
    timerAlarm(ControlTimer, Period, true, 0);

    timerStop(ControlTimer);
}

//  MAIN LOOP 
void loop()
{
    if (Stop || digitalRead(ESTOP_PIN) == HIGH)
    {
        Stop = true;
        Go = false;

        timerStop(ControlTimer);
        Stop_Motor();

        return;
    }

    if (!Go)
    {
        return;
    }

    CO[0] = 0;
    CO[1] = 0;
    CO[2] = 0;

    E[0] = 0;
    E[1] = 0;
    E[2] = 0;

    t = 0.0;
    Cnt = 0;

    Timer_Go = false;

    timerWrite(ControlTimer, 0);
    timerStart(ControlTimer);

    while (t < Time && !Stop)
    {
        if (Timer_Go)
        {
            Timer_Go = false;

            if (digitalRead(ESTOP_PIN) == HIGH)
            {
                Stop = true;
                break;
            }

            Encoder_Count = Read_Encoder();

            float elbowAngle = getElbowAngle();

            if (elbowAngle >= MAX_ELBOW_ANGLE)
            {
                Stop = true;
                break;
            }

            CLStep();

            if (digitalRead(ESTOP_PIN) == HIGH || Stop)
            {
                Stop = true;
                break;
            }

            Move_Motor(Motor_Duty);

            Cnt++;
            t = Cnt * Ts;
        }
    }

    timerStop(ControlTimer);
    Stop_Motor();

    Go = false;
}
