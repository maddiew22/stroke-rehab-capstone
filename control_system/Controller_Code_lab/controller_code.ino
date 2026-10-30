// Declare Mode of Operation where:                         
//      1 = Closed Loop Step with PID Controller -- applies a discrete PID Controller based 
//                                                    on Kp, Ki,and Kd values for a step input
//                            Inputs  ---------------------------------------------------
//                              Kp -- Proportional gain value
//                              Ki -- Integral gain value
//                              Kd -- Derivative gain value
//                              N -- Filter Coefficient value for Derivative output calculation in rad/s
//                              Reference_Input -- gain of step input in radians
//                              H_Gain -- gain of the Feedback function H(s) in encoder counts/radians
//                              I_Gain -- gain of the Input Filter function I(s) in encoder counts/radians
//                              V_in -- voltage supplied to Motor Controller in volts
//                              Time -- sets the duration of the test in seconds  
//                              Period -- sets the Control Loop Sample Period in milli seconds
//                            Outputs ---------------------------------------------------
//                              Freq -- sample frequency at which the Control Loop is run in Hertz
//                              Time -- duration of test in seconds
//                              Reference_Input -- rotational position in radians
//                              Encoder_Count -- encoder count; a signed integer value 
//
//      2 = Closed Loop Continuous Input

// Declare Mode of Operation
static byte Mode = 1; // Closed loop step mode

// Declare Step Input PWM output (0-1023)
static float Step_Input = 1;

// Declare PID Gains 
static float Kp = 2.2881;                     
static float Ki = 0;                
static float Kd = 0.0582836;                
static float N = 100;                 

// Declare Desired Input Value (radians)
float Reference_Input = 0.0872665; // 5 degrees         

// Declare Feedback Function H(s) (assumed to be a gain)
float H_Gain = 651.9;

// Declare Input Filter Function I(s) (assumed to be a gain)
float I_Gain = 651.9;

// Declare Voltage Input (volts)
float V_in = 24;

// Desired control-loop period in microseconds
const unsigned int Period = 508;

// Declare Test Duration in seconds
static float Time = 5;

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

#include <TimerOne.h>
#include <math.h>
#include <mcp_can.h>
#include <SPI.h>

// Setup MCP2515 CAN Module Chip Select Pin
const int SPI_CS_PIN = 10; 
MCP_CAN CAN(SPI_CS_PIN);

#define MOTOR_CAN_ID 0x01
#define CAN_PACKET_SET_DUTY 0  // Servo mode command ID for Duty Cycle

// // Setup Motor Controller 1 Pins
// static byte Reset = 4;         // Motor Controller Reset assigned to Digital Pin 4
// static byte PWM_Out = 5;       // Motor Controller PWM assigned to Digital Pin 5 
// static byte DIR = 6;           // Motor Controller DIR assigned to Digital Pin 6  

// Setup Encoder Counter 1 Pins
static byte SEL1 = 38;         // Select 1 output signal assigned to Pin 38
static byte OE = 39;           // Output Enable signal assinged to Pin 39
static byte RST = 40;          // Reset output signal assigned to Pin 40
static byte SEL2 = 41;         // Select 2 output signal assinged to Pin 41

// Declare Contstants
static float pi = 3.141593;

// Setup ISR Global Variables
volatile byte Go = 0;                         // Interrupt flag for START
volatile byte Timer_Go = 0;                   // Interrupt flag for timer
volatile byte Stop = 0;                       // Interrupt flag for E-STOP  
volatile byte Stop_bit = 0;                   // E-Stop input flag
volatile signed long int Reference_Input_Encoder = 0;   // Reference Input Signal in encoder counts (R(s)*I(s))
volatile signed long int Encoder_Count = 0;   // Current Encoder position in encoder counts
volatile float Controller_Output = 0;         // Capture/Control Loop Controller_Output in encoder counts
volatile float CO[3] = {0,0,0};               // Controller Output array in volts 
volatile float E[3] = {0,0,0};                // Error array in encoder counts
volatile float C1 = 0;                        // Controller Output coefficient @ t-1 
volatile float C2 = 0;                        // Controller Output coefficient @ t-2 
volatile float E0 = 0;                        // Error coefficient @ t  
volatile float E1 = 0;                        // Error coefficient @ t-1 
volatile float E2 = 0;                        // Error coefficient @ t-2 
volatile float Ts = 0;                        // Capture/Control Loop Sample Period in seconds
volatile float Freq = 0;                      // Capture/Control Loop Sample Frequency in hertz
volatile float t = 0;                         // Time counter in seconds
volatile long unsigned int Cnt = 0;           // Counter variable for Capture/Control Sample Loop 


// Helper function to unpack 32-bit integers from incoming CAN messages
int32_t buffer_get_int32(uint8_t* buffer, int32_t *index) {
  int32_t res = ((uint32_t)buffer[*index] << 24) |
                ((uint32_t)buffer[*index + 1] << 16) |
                ((uint32_t)buffer[*index + 2] << 8) |
                ((uint32_t)buffer[*index + 3]);
  *index += 4;
  return res;
}

// Helper function to pack 32-bit integers into outgoing CAN buffers
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}


// Read Encoder Position Function
signed long int Read_Encoder()
{
  // Replace with Sophia's code
  signed long int Count = 0;     // encoder position total
  return Count;
} 

// Reset Encoder Counter Function
void Reset_Encoder_Counter()
{
  // Reset Encoder Counter by toggling RST pin
  digitalWrite(RST, HIGH); 
  delay(100);
  digitalWrite(RST, LOW);          
  delay(100);
  digitalWrite(RST, HIGH); 
}

// Ready Motor Function
void Ready_Motor()
{
  // No explicit reset pin toggle needed
  delay(100);
}

// Stop Motor Function
void Stop_Motor()
{
  // Send 0 duty cycle to safely idle the motor
  Move_Motor(0.0);
}

// Move Motor Function using Native Duty Cycle Mode over CAN
void Move_Motor(float duty_value)
{  
  // Clamp duty cycle to safe driver limits (-0.95 to 0.95)
  if (duty_value > 0.95) duty_value = 0.95;
  if (duty_value < -0.95) duty_value = -0.95;

  int32_t send_index = 0;
  uint8_t buffer[4];
  
  // Protocol specification: Duty cycle scaled by 100,000 as int32
  buffer_append_int32(buffer, (int32_t)(duty_value * 100000.0), &send_index);

  // Construct Extended CAN ID: Motor_ID | (Command_ID << 8)
  uint32_t ext_id = MOTOR_CAN_ID | ((uint32_t)CAN_PACKET_SET_DUTY << 8);

  // Send via Extended Frame (1 = Extended Frame)
  CAN.sendMsgBuf(ext_id, 1, 4, buffer);
}


// Closed Loop Step Function with PID Controller
void CLStep(signed long int Target_Position)
{
  E[0] = Target_Position - Encoder_Count; // Calculate current error value 
  
  CO[0] = -C1*CO[1] - C2*CO[2] + E0*E[0] + E1*E[1] + E2*E[2];  // Calculate new Controller Output in volts           
  Controller_Output = CO[0]/V_in  // Convert from volts to duty cycle

  CO[2] = CO[1];  // save Controller Output @ t-1 as Controller Output @ t-2
  CO[1] = CO[0];  // save Controller Output @ t as Controller Output @ t-1
  E[2] = E[1];    // save Error @ t-1 as Error @ t-2
  E[1] = E[0];    // save Error @ t as Error @ t-1
}  

// Main Setup Routine
void setup() 
{
  // Setup serial output to 115,200 bps (bits per second)
  Serial.begin(115200); 
     
  // Initialize MCP2515 CAN Bus at 1000kbps (1 Mbps required for AK series)
  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN Bus Initialized Successfully!");
  } else {
    Serial.println("CAN Initialization Failed!");
    while (1);
  }
  CAN.setMode(MCP_NORMAL);

  // Setup Data input pins from Encoder Counter as inputs
  DDRA = 0; // Set pins 22-29 as inputs

  // // Setup Motor Controller 1 pins as Outputs
  // pinMode(Reset, OUTPUT);        // Setup digital pin 4 as an output
  // pinMode(PWM_Out, OUTPUT);      // Setup digital pin 5 as an output
  // pinMode(DIR, OUTPUT);          // Setup digital pin 6 as an output
  
  Stop_Motor();
  
  // Setup Encoder 1 Counter pins as Outputs
  pinMode(SEL1, OUTPUT);         // Setup digital pin 38 as an output 
  pinMode(OE, OUTPUT);           // Setup digital pin 39 as an output  
  pinMode(RST, OUTPUT);          // Setup digital pin 40 as an output
  pinMode(SEL2, OUTPUT);         // Setup digital pin 41 as an output   
  
  // Setup Interrupt Service Routine driven by a rising edge on int 0(Pin2) for E-Stop
  pinMode(2, INPUT);                     // Setup digital pin 2 as an input
  attachInterrupt(0,STOP,RISING);        // Setup 'STOP' interrupt to begin on RISING edge
  
  // Setup Interrupt Service Routine driven by a rising edge on int 1(Pin3) for Start Button
  pinMode(3, INPUT);                     // Setup digital pin 3 as an input
  attachInterrupt(1,START,RISING);       // Setup 'START' interrupt to begin on RISING edge
  
  // Adjust PWM Timer 3 for Pin 5 to work in Fast PWM mode with 10-bit resolution at 15.6kHz; 
  // set clock prescaler to 1  
  TCCR3A |= 3;           // Set WGM1 & WGM0 to 1 
  TCCR3B &= ~15;         // Set bits 3,2,1 and 0 to 0 
  TCCR3B |= 9;           // Set WGM2 & CS0 to 1 
  
  // if(Period < 2)         // Check to see if the Sample Period set by the user is less than 2 ms
  //   Period = 2;          // If it is, reset back to 2 ms
  
  // Setup Timer for Capture/Control Loop Frequency
  Timer1.initialize(Period);
  Timer1.attachInterrupt(Timer_ISR);
  Timer1.stop();  // Keep the control timer stopped until the test begins 
}


void loop()
{  
  // Define Local Variables  
  long unsigned int Cnt_Max = 0;        // Number of iterations to be preformed

  // Reset Global Variables  
  Go = 0;                               // Reset START flag      
  Stop = 0;                             // Reset E-STOP flag    
  CO[0] = 0;                            // Reset Controller Output at t
  CO[1] = 0;                            // Reset Controller Output at t-1
  CO[2] = 0;                            // Reset Controller Output at t-2
  E[0] = 0;                             // Reset Error at t
  E[1] = 0;                             // Reset Error at t-1
  E[2] = 0;                             // Reset Error at t-2

  // Precalculate Catpture/Control Loop Variables to speed up execuation time
  Ts = (float)Period/1000000;              // Catpture/Control Loop Sample Period converted to seconds from microseconds
  Freq = 1/Ts;                          // Calculate Catpture/Control Loop Sample Frequency in Hz
  Reference_Input_Encoder = int(Reference_Input * I_Gain);     // Convert Reference Input from radians to encoder counts
  Cnt_Max = Freq * Time;                // Calculate number of interations to be performed
  
  C1 = -(2 + N*Ts)/(1 + N*Ts);          // Calculate Controller Output coefficient @ t-1
  C2 = 1/(1 + N*Ts);                    // Calculate Controller Output coefficient @ t-2  
  E0 = (Kp*(1 + N*Ts) + Ki*Ts*(1 + N*Ts) + Kd*N)/(1 + N*Ts);      // Calculate Error coefficient @ t
  E1 = -(Kp*(2 + N*Ts) + Ki*Ts + 2*Kd*N)/(1 + N*Ts);              // Calculate Error coefficient @ t-1
  E2 = (Kp + Kd*N)/(1 + N*Ts);                                    // Calculate Error coefficient @ t-2

  // Send out initial settings
  Serial.println(Mode);                 // Send Mode value out serially  
  Serial.println(Freq);                 // Send Freq value out serially
  Serial.println(Time);                 // Send Time value out serially
  Serial.println(H_Gain);               // Send H_Gain value out serially
    
  Stop_Motor();
    
  while(!Go);                           // Wait for START push button to be pressed 
  delay(100);                           // Set delay of 100ms to debounce switch
  Go = 0;                               // Reset START flag 
     
  Reset_Encoder_Counter();              // Reset Encoder Counter to position 0
  Ready_Motor();                        // Get motor ready
  Timer1.start();                       // Start timer  
  
  for(Cnt=0;Cnt <= Cnt_Max;Cnt++)     
  { 
    while(Timer_Go == 0);               // Wait for next Timer iteration`          
    Timer_Go = 0;                       // Reset Timer flag

    // if(Mode == 1)
    //   Serial.println(Reference_Input);    // Send Reference_Input value out serially
    // else
    //   Serial.println(Controller_Output);  // Send Controller_Output value out serially
    // Serial.println(Encoder_Count);        // Send Encoder_Count value out serially    

    if((Stop) || (Stop_bit))              // If E-Stop has been pressed, break from loop
      break;
  }
  
  Stop_Motor();                         
  Timer1.stop();                  
  delay(100);                           // Set delay of 100ms to debounce switch
} 

// Timer Interrupt Service Routine trigger by MsTimer2 function
void Timer_ISR()
{ 
  Encoder_Count = Read_Encoder();  // Read in current encoder position   
  
  // Closed Loop Step Mode
  if(Mode == 1)                     
    CLStep(Reference_Input_Encoder);
  // Continuous input
  else if(Mode == 2)
  {

  } 
            
    
  Stop_bit = digitalRead(2);            // Read in status of E-Stop   
  if((!Stop) && (!Stop_bit))            // If the E-Stop has not been pressed then...
    Move_Motor(Controller_Output);      // Call Move Motor Function 
  else                                  // Else...
    Stop_Motor();                       // Call Stop Motor Function 
 
  Timer_Go = 1;                         // Set Timer flag
}  

// Interrupt Service Rountine triggered by RISING edge from user activated
// momentary push button (Start Button)
void START()
{
  Go = 1;                              // Set Push Button flag  
}   
       
// Interrupt Service Rountine triggered by RISING edge from E-Stop
void STOP()
{
  Stop = 1;                           // Set STOP flag
  Stop_Motor();                       // Call Stop Motor Function 
}
