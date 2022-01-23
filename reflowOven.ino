/* 
*
*     Reflow Oven v0.2
*
*     WORK IN PROGRESS
*
*
*
*     1. Preheat          ( 0 - 5 min   |  Temperature 0 - 120°C )
*     1. Ramp to Soak     ( 0 - 60 s    |  Temperature  120 150°C )
*     2. Soak "Preheat"   ( 60 - 120 s  |  Temperature 150 - 200°C : ) 
*     3. Ramp to Peak     ( 60 s  |  Time Above 220°C :       60-150 sec Max. )
*     4. Dwell [ Reflow ] ( 10 s        |  Peak Temp. : 260 +/-5°C, Max. 10 s )
*     5. Go Down          ( 60 s)       |  Time Above 220°C :       60-150 sec Max. )
*     6. Cooling           (            |  Max temp gradient in cooling -6°C/s Max. )
*
*
*
* 
*   235-|                                                 x
*       |                                               x   x  within 5°C [10-30s]
*       |                                            x       x
*       |                                          x    |         x
*       |                                       x       |            x
*   183-|------------------------------------x          |               x
*       |                              x                |                   x   
*       |                         x         |           |              |       x
*       |                    x              |                          |
*       |               x                   |                          |
*       |             x |                   |                          |
*       |           x   |                   |                          | 
*   100-|---------x-----|                   |                          | 
*       |       x       |                   |                          | 
*       |     x         |                   |                          |
*       |   x           |                   |                          |
*       | x  max. 3°C/s |                   |                          |
*       |<   0 - 60 s  >|<    60 - 120 s   >|<60 - 150 s              >|   max. 6°C/s
*       | Preheat Stage |   Soaking Stage   |             Reflow Stage | Cool
*       |  100°C-150°C  |    max. 183°C     |                          |
*    0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
*
*
*
*
*
*    OVEN
*
*
*
*
*
*
*/


const char * ver = "0.3";
//-------------------------------------------------------------
/*                     1.  INCLUDES                          */
//-------------------------------------------------------------

//#include <SPI.h>

#include <avr/io.h>
#include <avr/interrupt.h>

//------ PID Library ------------------------------------------

#include <PID_v1.h>
//#include <PID_v2.h>
//#include <PID_AutoTune_v0.h>

#include <pidautotuner.h>

//------ Thermocouple -----------------------------------------

#include <MAX31855.h>
//#include <max6675.h>

//------ Menu, Encoder & Timer --------------------------------
//#include <ClickEncoder.h>
//#include <TimerOne.h>
//#include <Menu.h>
//#include "helpers.h"

//------ Display & TouchScreen --------------------------------
#include <registers.h>
#include <pin_magic.h>
#include <SPFD5408_Adafruit_GFX.h>
#include <SPFD5408_Adafruit_TFTLCD.h>
#include <SPFD5408_TouchScreen.h>

//-------------------------------------------------------------
/*                     2.   DEFINES                          */
//-------------------------------------------------------------d

// Colors --------------------------------------------------

// Basic
#define BLACK     0x0000
#define BLUE      0x001F
#define TEAL      0x0438
#define GREEN     0x07E0
#define CYAN      0x07FF
#define RED       0xF800

// Plus
#define MAGENTA   0xF81F
#define YELLOW    0xFFE0
#define ORANGE    0xFC00
#define PINK      0xF81F
#define PURPLE    0x8010
#define GREY      0xC618
#define WHITE     0xFFFF

#define DKBLUE    0x000D
#define DKTEAL    0x020C
#define DKGREEN   0x03E0
#define DKCYAN    0x03EF
#define DKRED     0x6000
#define DKMAGENTA 0x8008
#define DKYELLOW  0x8400
#define DKORANGE  0x8200
#define DKPINK    0x9009
#define DKPURPLE  0x4010

// Special
#define DKGREY    0x4A49
#define CLOUDS    0xEF9E
#define TURQUOISE 0x1DF3
#define AMETHYST  0x9AD6
#define MYBROWN   0xA301
#define CONCRETE  0x9534
#define ORANGE    0xFC00
#define POMEGRANATE 0xC1C5


//------ Reflow Parameters  ----------------------------------


//------ Reflow Profile Stages -------------------------


// 1 " Preheat"     // Preheat our oven and "boards&components" slowly to evaporate any moisture trapped inside components
// 2 " RampToSoak"  // Volatilization of solvents
// 3 " Soak"        // Flux action and plate temperature balance
// 4 " RampToPeak"  // Preheat/ Ramp to Melting point
// 5 " Dwell"       // Melting of solder paste
// 6 " GoDown"      // 
// 7 " Cool "

// 1 " Preheat"     // Very useful for cold rooms, or alminium PCB's
// 2 " RampToSoak"  // Ramp up to soak min from preheat
// 3 " Soak"        // Soaking stage, lift temperature slowly but surely
// 4 " RampToPeak"  // get to peak temp fast
// 5 " Dwell"       // Reflow Stage - max 10 s - come down quick to go down max
// 6 " GoDown"      // Controlled ramping down, useful for 


// Reflow Lead Free (ROHS)Profile Temperature Settings


// --- Idle Temperature
#define TEMPERATURE_ROOM 20 //
#define TEMPERATURE_IDLE 80 //

// --- 1. Preheat Temperature
#define TEMPERATURE_PREHEAT_MIN 100  //
#define TEMPERATURE_PREHEAT_MAX 120 //
#define PREHEAT_TEMPERATURE_STEP 1.6

// Holding Temperature

// --- 2. Ramp to Soak Temperature
#define TEMPERATURE_RAMP_TO_SOAK_MIN 120  // 
#define TEMPERATURE_RAMP_TO_SOAK_MAX 150  // 
#define RAMP_TO_SOAK_TEMPERATURE_STEP 1

// --- 3. Soak Temperature
#define TEMPERATURE_SOAK_MIN 150  // 
#define TEMPERATURE_SOAK_MAX 200  //
#define SOAK_TEMPERATURE_STEP 1.2

// Reflow temperature

#define TEMPERATURE_ABOVE_EUTECTIC_TEMPERATURE  220 // 217-220 °C Eutectic Temperature of solder paste 
//( lowest possible melting temp for all of the mixing ratios of the component substances of the solder paste )

// Lead-Free (ROHS) 217°C
// Leaded    (SnPb) 168°C

// --- 4. Ramp to Peak Temperature
#define TEMPERATURE_RAMP_TO_PEAK_MIN 220  //
#define TEMPERATURE_RAMP_TO_PEAK_MAX 240  //
#define RAMP_TO_PEAK_TEMPERATURE_STEP 2

// Peak Temperature

// --- 5. Reflow Temperature
#define TEMPERATURE_REFLOW_DWELL_MIN 240  // spec’d wet region start
#define TEMPERATURE_REFLOW_DWELL_MAX 260  // Reflow !
#define REFLOW_DWELL_TEMPERATURE_STEP 3


// --- 6. Go Down Temperature (min. max. is reversed here)
#define TEMPERATURE_GO_DOWN_MIN 220 // maybe use this for something else later
#define TEMPERATURE_GO_DOWN_MAX 200 // like controlled going down ()
#define GO_DOWN_TEMPERATURE_STEP 3     // preferred go down rate
#define GO_DOWN_TEMPERATURE_STEP_MAX 6 // max 6°C /s

// --- Cool Temperature
#define TEMPERATURE_COOL_MIN 80 //  Temperature at which safe to let it cool by it's own.
#define TEMPERATURE_COOL_MAX 60 //  Temperature at which safe to let it cool by it's own. (TEST)
#define COOL_TEMPERATURE_STEP 6 // max 6°C /s

// --- Complete Temperature
#define TEMPERATURE_COMPLETE_MIN 160 //  Temperature at which all controls are off
#define TEMPERATURE_COMPLETE_MAX 120 // it's up to you now to open door.

// --- MAX TEMP !!!!! TOO HOT
#define TEMPERATURE_ABSOLUTE_MAX_TEMP_MIN 261 // start kicking the fan gently, if goes down, show closable error
#define TEMPERATURE_ABSOLUTE_MAX_TEMP_MAX 263 // hit the fan 100% if doesnt go down, stop ssr, error

// --- Temperature Slope
#define TEMPERATURE_SLOPE_MIN 3 // Minimum rate of cooling 
#define TEMPERATURE_SLOPE_MAX 6 // Maximum rate of cooling




// ----- Reflow Profile Duration (Time) Settings

#define REFLOW_DURATION 720 // 12 MINUTES / two pages for the graph

// --- 1 .Preheat
#define PREHEAT_DURATION 60 // 1 and a half mintues to soak


// Heat Preservation ( 2min 30 s )
// --- 2. Ramp to Soak
#define RAMP_TO_SOAK_DURATION 30 // 30 s to soak from preheat

// --- 3. Soak // 1 and a half minutes to soak FOR TEST (/60000)
#define SOAK_DURATION 90 // 1 and a half mintues to soak


// Time Above Eutectic temperature ( 217-220 °C )
#define MAX_TIME_ABOVE_ETECTIC_TEMPERATURE 120 // 60 - 150 max

// --- 4. Ramp To Peak         
#define RAMP_TO_PEAK_DURATION 30 // 30 seconds to soak

// --- 5. Reflow Dwell  // 10 seconds max at peak / reflow / dwell temperature.
#define DWELL_DURATION 10 // 1 and a half mintues to soak

// --- 6. Go Down // max 6°C /s
#define GO_DOWN_DURATION 90 // 1 and a half mintues to soak

// --- Cool
#define COOL_DURATION 360 //

// soak-to-reflow beeper "on" duration in mS
//#define BEEPDURATION_REFLOW 250
// reflow-to-cool beeper "on" duration in mS
//#define BEEPDURATION_COOL 2000
// cool-to-complete beeper "on" duration in mS
//#define BEEPDURATION_COMPLETE 3000

#define DEBOUNCE_PERIOD_MIN 50
#define THERMOCOUPLE_DISCONNECTED 10000

// Reading sensor data (max speed. 100ms / read / MAX31855K )
#define TEMPERATURE_SAMPLING_TIME 100  // 500 ms - check twice every second / 100 ms - max read from max31855k
//#define PROCESS_SAMPLING_TIME 500 // 250ms for averaging at least 4 readings per second // 500 ms check twice every second

//#define      PID_SAMPLE_TIME 500  // 500 ms - check twice every second
#define PROCESS_SAMPLING_TIME 500 // updating the graph plot and pid output once a second (could be faster if wanted) 

//------ PID Presets -------------------------------------------------

// Kp - Proportional Gain - Present      - Amplifies the error proportionally (                                                                     ( Quick reaction to error )             | Kp+ - Increase speed of response        & Kp- - Reduce oscillations
// Ki - Integral     Gain - History   - Output is relative to history of error ( even smaller error over long time will have impact ) History   ( Reduce steady state error to 0 )      | Ki+ - Reduce steady state error         & Ki- - Reduce overshoot, settling time
// Kd - Derivative   Gain - future    - Output is relative to the future of error ( fast rate of change, output changes quickly )     Future    ( Improves stability and ... )          | Kd+ - Reduce overshoot, settling time   & Kd- - Noisy conditions

// Kp = 2,   Ki = 0.05,  Kd = 0.3;  // 4 - success, close
// Kp = 1,   Ki = 0.05,  Kd = 0.3;  // 5 - not so

// Kp = 100, Ki = 0.025, Kd = 20 ;  // 6 - 
// Kp = 10 , Ki = 1    , Kd = 20 ;  // 8 - 
// Kp =  1 , Ki = 0.05 , Kd = 20 ;  // 7 - 


// Kp = 100, Ki = 0.025, Kd = 20;  // 0xPIT pid settings
// Kp = 300, Ki = 0.05,  Kd = 250;  // 
// Kp = 300, Ki = 0.05,  Kd = 350;  // 


double aggKp =  10;
double aggKi =   1;
double aggKd =  20;

// --- 1. PRE-HEAT STAGE 
//#define PID_KP_PREHEAT      1         // 2 
//#define PID_KI_PREHEAT      0.05      // 0.05
//#define PID_KD_PREHEAT      0.25       // 0.3

// --- 2. RAMP TO SOAK STAGE 
//#define PID_KP_RAMP_TO_SOAK 1         // 2 
//#define PID_KI_RAMP_TO_SOAK 0.05      // 0.05
//#define PID_KD_RAMP_TO_SOAK 0.25       // 0.3

// --- 3. SOAKING STAGE 
//#define PID_KP_SOAK         1         // 2
//#define PID_KI_SOAK         0.05      // 0.05 
//#define PID_KD_SOAK         0.25       // 0.3

// --- 4. RAMP TO PEAK STAGE 
//#define PID_KP_RAMP_TO_PEAK 1         // 2
//#define PID_KI_RAMP_TO_PEAK 0.05      // 0.05
//#define PID_KD_RAMP_TO_PEAK 0.25       // 0.3

// --- 5. REFLOW DWELL STAGE 
//#define PID_KP_REFLOW_DWELL 1         // 2 
//#define PID_KI_REFLOW_DWELL 0.05      // 0.05
//#define PID_KD_REFLOW_DWELL 0.25       // 0.3

// --- 6. GO DOWN STAGE 
//#define PID_KP_GO_DOWN      2         // 
//#define PID_KI_GO_DOWN      0.05      // 
//#define PID_KD_GO_DOWN      0.3       // 

// --- 0. BAKE 
//#define PID_KP_BAKE 2       // 
//#define PID_KI_BAKE 0       // 
//#define PID_KD_BAKE 0       // 


//------ Transistor - Solid State Relay - Zero Crossing -------

// We actually use a Zero-Crossing SolidStateRelay, all we have to to is modulate the voltage.

#define INT_ZX  1   // interrupt for zero crossing detector
#define ZCSPin  2   // pin for zero crossing detector
#define SSRPin  3   // pin for switching the SSRelay

#define PULSE 4     //trigger pulse width (counts)

// Rotary Encoder Inputs ------------------------------------------

bool analogCLK, analogDT;

#define inputCLK  49//A6
#define inputDT   49//A7
#define inputBTN  49//12

int counter = 0; 
int currentStateCLK;
int previousStateCLK; 

int PreviousSensorValue;
int threshold;
String encdir ="";


// ---------------------------------------------------------------------
//                        Touchscreen
// ---------------------------------------------------------------------

#define MINPRESSURE 10
#define MAXPRESSURE 1000

#define YP A1
#define XM A2
#define YM 7
#define XP 6

#define TS_MINX 125
#define TS_MINY 85
#define TS_MAXX 958
#define TS_MAXY 905

  // Touchscreen
  TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
  
  // Touch point
  TSPoint p;


// ---------------------------------------------------------------------
//                          Display
// ---------------------------------------------------------------------

#define tft_RST 2
#define tft_CS A3
#define tft_CD A2
#define tft_WR A1
#define tft_RD A0

  Adafruit_TFTLCD tft(tft_CS, tft_CD, tft_WR, tft_RD, tft_RST);


// ---------------------------------------------------------------------
//                        MAX31855K Thermocouple
// ---------------------------------------------------------------------

  const  unsigned  char thermocoupleSO = A4;
  const  unsigned  char thermocoupleCS = 5;
  const  unsigned  char thermocoupleCLK = A5;
  
  MAX31855  MAX31855(thermocoupleSO, thermocoupleCS, thermocoupleCLK);


// ---------------------------------------------------------------------
//                          PID Algorithm
// ---------------------------------------------------------------------

// Current time
unsigned long now;

//------ PID Control Variables --------------------------------

double setpoint;
double input;
double p_input;
double output;

double slope;
double gap;

// Assign preheat pid settings at beginning
//double kp = PID_KP_PREHEAT;
//double ki = PID_KI_PREHEAT;
//double kd = PID_KD_PREHEAT;

//double kp = aggKp;
//double ki = aggKi;
//double kd = aggKd;

//  PID Algorithm -------------------------------------------

  // V1
  //PID reflowOvenPID(&input, &output, &setpoint, PID_KP_PREHEAT, PID_KI_PREHEAT, PID_KD_PREHEAT, DIRECT);
  PID reflowOvenPID(&input, &output, &setpoint, aggKp , aggKi, aggKd, DIRECT);
  
  // V2
  //PID_v2 reflowOvenPID( kp, ki, kd, PID::Direct );


// ---------------------------------------------------------------------
//                      PID Auto-Tuning Algorithm
// ---------------------------------------------------------------------

byte ATuneModeRemember = 0;                     //  0 - 2   ?
double kpmodel = 1.5, taup = 100, theta[50];    //  math?!
double outputStart = 5;                         // start
double reflowOvenPIDTuneStep = 50,              // steps of increase
       reflowOvenPIDTuneNoise = 1,              // allowed noisefloor (C ?)
       reflowOvenPIDTuneStartValue = 100;       // is set to output, i.e. 0-100 % of Heater

unsigned int reflowOvenPIDTuneLookBack = 30;              // look back 30 reads ?

boolean tuning = false;
unsigned long modelTime, serialTime; // Auto-Tuning Variables

//set to false to connect to the real world
boolean useSimulation = false;
  
//  PID Auto-Tune -------------------------------------------

  //PID_ATune reflowOvenPIDTune(&input, &output);

  PIDAutotuner tuner = PIDAutotuner();

  
//--------------------------------------------------------------------------------------
//                                   3.   VARIABLES                                   //
//--------------------------------------------------------------------------------------

//------ Graph coordinate and data variables ------------------

// Redraw
boolean display1 = true;

uint16_t runTimes = 1;

// Graph Grid Variables 
double ox , oy;
double ox2, oy2;
double ox3, oy3;
double ox4, oy4;

// Graph Variables
double x = 1.0 ;
double y, y2, y3, y4;


//------ PID Control Variables --------------------------------
// Zero Crossing
volatile uint32_t timerTicks     = 0;
volatile uint32_t zeroCrossTicks = 0;
volatile uint8_t  phaseCounter   = 0;


//------ Reflow Parameters  ----------------------------------------


//------ Reflow Oven States -------------------------
//
//1 " Ready/IDLE"
//2 " Pre-heat"
//3 " RampToSoak"
//4 " Soak"
//5 " RampToPeak"
//6 " Reflow/DWELL"
//7 " GoDown"
//8 " Cool"
//9 " Complete"
//10 " Error"
//8 " Tuning"
//12 " Wait, HOT"




//------ Reflow Profile Timing -------------------------


// do we use this anymore ?
uint16_t windowSize;
unsigned long windowStartTime;

// read sensors, apply pid, output to heater control circuitry ( Optocoupler -> NPN Transistor -> SSR relay )
unsigned long nextGraphPID;
unsigned long nextTempRead;

boolean eutecticTempFlag = false;

// Temp Reading Smoothing for PID
const int numReadings = 5;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average every second ?


// Temperature Measurements
double temperature;
char tempString[4];
String temperatureInt;

// Junction Temperature Measurements
double junctionTemp;
char junctionTempString[3];
String junctionTempInt;

char outputPowerString[3];
String outputPowerInt;

char setpointString[3];
String setpointInt;

// Temperature Slope
char slopeString[3];
String slopeInt;

// Bake Tempertature
double bakeSetTemp;
char bakeTempString[4];
String bakeTempInt;



// Graph, reflow and timing variables

//const long interval = 1000; // 1px / s
unsigned long startMillis;          // 
unsigned long currentMillis;        //
unsigned long previousMillis = 0;   //
unsigned long elapsedMillis;        //




// Reflow Profile Time Keeping
unsigned long timerReflow;    // store total reflow process timer

unsigned long timerPreheat;   // store preheat timer
unsigned long timerRampToSoak;
unsigned long timerSoak;

unsigned long timerAboveEutecticTemp; // store seconds when above this temp, min. 60 s , max. 150 s

unsigned long timerRampToPeak;
unsigned long timerReflowDwell;

unsigned long timerGoDown;
unsigned long timerCool;


// Reflow Profile Time Settings

// question is use defines of ints ? -

unsigned int ReflowTime = REFLOW_DURATION;            // total reflow time
unsigned int PreheatTime = PREHEAT_DURATION;          // total preheat time 
unsigned int RampToSoakTime = RAMP_TO_SOAK_DURATION;  // total RampToSoak Time
unsigned int SoakTime = SOAK_DURATION ;               // Soak time

unsigned int RampToPeakTime = RAMP_TO_PEAK_DURATION;
unsigned int ReflowDwellTime = DWELL_DURATION;
unsigned int GoDownTime = GO_DOWN_DURATION;
unsigned int CoolTime = COOL_DURATION;


// Reflow Profile Timings (for future )
unsigned long ReflowStartTime;

unsigned long PreheatStartTime;
unsigned long RampToSoakStartTime;
unsigned long SoakStartTime;

unsigned long RampToPeakStartTime;
unsigned long ReflowDwellStartTime;
unsigned long GoDownStartTime;
unsigned long CoolStartTime;

//unsigned long buzzerPeriod;


//------ Reflow State Machine -------------------------
// Reflow oven controller state machine state variable

// Reflow Stages
typedef enum REFLOW_STATE {
  REFLOW_STATE_IDLE,          // 1  - Idle          State ( Idle state of reflow oven, resets and takes back to MainMenu )
  REFLOW_STATE_PREHEAT,       // 2  - Preheat       Stage ( Very good for cold rooms, or aluminium PCB's)
  REFLOW_STATE_RAMP_TO_SOAK,  // 3  - Ramp To Soak  Stage ( Normally this is the preheat stage, we are ramping up to soaking temp )
  REFLOW_STATE_SOAK,          // 4  - Soak          Stage ( We stay here for a while, especially for Alu PCB's )
  REFLOW_STATE_RAMP_TO_PEAK,  // 5  - Ramp To Peak  Stage ( Ramping up fast to peak temperature )
  REFLOW_STATE_REFLOW_DWELL,  // 6  - Reflow[Dwell] Stage ( We are at dwell temperature, sit here for max. 10 s , here is everything reflows )
  REFLOW_STATE_GO_DOWN,       // 7  - Go Down       Stage ( Used for tightly controlling the ramp down temperature, could be useful for hefty aluminium pcb's to prevent warpage due to temperature differences )
  REFLOW_STATE_COOL,          // 8  - Cooling       State ( Used for monitoring the ramp down temperature )
  REFLOW_STATE_COMPLETE,      // 9  - Complete      State ( Enter this state after a successfull reaflow, maybe for future automation )
  REFLOW_STATE_TOO_HOT,       // 10 - Too Hot       State ( If things are too hot, take some measures, TODO. add exhaust fan )
  REFLOW_STATE_ERROR,         // 11 - Error         State ( Disconnected thermocouple or other errors )
  REFLOW_STATE_TUNE,          // 12 - Tune          Stage ( Tuning the PID Parameters, and uploading new settings )

}

reflowState_t;

// ----------------------------
typedef enum REFLOW_STATUS {
REFLOW_STATUS_OFF,
REFLOW_STATUS_ON
}
reflowStatus_t;

// Reflow oven controller state machine state variable
reflowState_t reflowState;

// Reflow oven controller status
reflowStatus_t reflowStatus;





//------ Baking Oven States -------------------------
//
//1 " Idle/Ready"
//2 " Pre-heat"
//4 " Bake"
//11 " GoDown"
//5 " Cool"
//6 " Complete"
//7 " Error"
//8 " Tuning"
//12 " Wait, HOT"
//14 " Other" // Can be used for whatever you want



// Store seconds and minutes for display
int minutesElapsed;
int secondsElapsed;

String minutesString;
String secondsString;

char reflowStatusChar[10];
String reflowStatusString;
String bakeStatusString;








// ----- Switch / Button WIP -----------------------



typedef  enum SWITCH
{
  SWITCH_NONE,
  SWITCH_1, 
  SWITCH_2
} switch_t;

// ----------------------------
typedef enum DEBOUNCE_STATE {
  DEBOUNCE_STATE_IDLE,
  DEBOUNCE_STATE_CHECK,
  DEBOUNCE_STATE_RELEASE
}
debounceState_t;

// Button debounce state machine state variable
debounceState_t debounceState;

// Button debounce timer
long lastDebounceTime;

// Button press status
boolean buttonPressStatus;
// Switch press status
switch_t switchStatus;

// Seconds timer
uint16_t timerSeconds;


// ----- Pages -----------------------
uint16_t page;
bool profileType = 1;
//bool firstStart = 0;

//boolean tuning;

//char buffer[10]; // make sure this is large enough for the largest string it must hold


// DEGREE SYMBOL FOR LCD
//unsigned char degree[8]  = {140,146,146,140,128,128,128,128};





//-----------------------------------------------------------------------
//                 4.   Zero Crossing Functions                        //
//-----------------------------------------------------------------------

int i=483;
//Interrupt Service Routines

void zeroCrossingInterrupt(){ //zero cross detect   
  TCCR1B=0x04; //start timer with divide by 256 input
  TCNT1 = 0;   //reset timer - count from zero
}

ISR(TIMER1_COMPA_vect){ //comparator match
  //digitalWrite(SSRPin,HIGH);  //set TRIAC gate to high
  TCNT1 = 65536-PULSE;      //trigger pulse width
}

ISR(TIMER1_OVF_vect){ //timer1 overflow
  //digitalWrite(SSRPin,LOW); //turn off TRIAC gate
  TCCR1B = 0x00;          //disable timer stopd unintended triggers
}






//-----------------------------------------------------------------------
//                5.   GUI Functions for HMI                        //
//-----------------------------------------------------------------------

// Reset the Arduino after a succesful reflow.
void(*resetFunc)(void) = 0;


// Wait one touch ---------------------------------------------
TSPoint waitOneTouch() {

  //TSPoint p;

  do {
    p = ts.getPoint();

    pinMode(XM, OUTPUT); //Pins configures again for TFT control
    pinMode(YP, OUTPUT);

  } while ((p.z < MINPRESSURE ) || (p.z > MAXPRESSURE) );

  return p;
}

//---------- Return Touch Coordinates -------------------------
TSPoint getTouch() {
  
    p = ts.getPoint();
  
   // if sharing pins, you'll need to fix the directions of the touchscreen pins
      //pinMode(XP, OUTPUT);
        pinMode(XM, OUTPUT); //Pins configures again for TFT control
        pinMode(YP, OUTPUT);
      //pinMode(YM, OUTPUT);
 
   if ((p.z < MINPRESSURE ) || (p.z > MAXPRESSURE)){      
                  
      //  Serial.print("X = "); Serial.print(p.x);
      //  Serial.print("\tY = "); Serial.print(p.y);
      //  Serial.print("\tPressure = "); Serial.println(p.z);
         
        // if(p.y< (TS_MINY -5)){
        //  Serial.println("erase");
          // press the bottom of the screen to erase
        //  tft.fillRect(0, BOXSIZE, tft.width(), tft.height() - BOXSIZE, BLACK);
        // }     
        p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.width());
        p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());
  }
}

// ------ Click Encoder --------------------------------
void checkClickEncoder(){

  // read state of button and debounce

  digitalRead(inputBTN);

  // Read the current state of inputCLK
  currentStateCLK = digitalRead(inputCLK);

  // read the input on analog pin 0:
  int sensorValue = analogRead(A7);
  
  analogCLK = map(sensorValue,0 , 1010, 0, 1);
  
  if (abs(PreviousSensorValue - sensorValue) > threshold) {

    int sensor2Value = analogRead(A6);
    analogDT = map(sensor2Value,0 , 1010, 0, 1);

    // If the previous and the current state of the inputCLK are different then a pulse has occured
    if (currentStateCLK != previousStateCLK){
      // If the inputDT state is different than the inputCLK state then 
      // the encoder is rotating counterclockwise
      if (digitalRead(inputDT) != currentStateCLK){
        counter --;
        encdir ="CCW";
        //Serial.println("CCW"); 
      } else {
          // Encoder is rotating clockwise
          counter ++;
          encdir ="CW";
          //Serial.println("CW");
        }
      //Serial.print("Direction: ");
      //Serial.print(encdir);
      //Serial.print(" -- Value: ");
      //Serial.println(counter);
    } 
      // Update previousStateCLK with the current state
      previousStateCLK = currentStateCLK;
  }

}



//-------------------------------------------------------------------------
//                       PID Auto-Tune Functions                               //
//-------------------------------------------------------------------------

void reflowOvenPIDAutoTuner() {
    // Set the target value to tune to
    // This will depend on what you are tuning. This should be set to a value within
    // the usual range of the setpoint. For low-inertia systems, values at the lower
    // end of this range usually give better results. For anything else, start with a
    // value at the middle of the range.
    tuner.setTargetInputValue(setpoint);

    // Set the loop interval in microseconds
    // This must be the same as the interval the PID control loop will run at
    tuner.setLoopInterval(PROCESS_SAMPLING_TIME);

    // Set the output range
    // These are the minimum and maximum possible output values of whatever you are
    // using to control the system (Arduino analogWrite, for example, is 0-255)
    tuner.setOutputRange(0, 255);

    // Set the Ziegler-Nichols tuning mode
    // Set it to either PIDAutotuner::ZNModeBasicPID, PIDAutotuner::ZNModeLessOvershoot,
    // or PIDAutotuner::ZNModeNoOvershoot. Defaults to ZNModeNoOvershoot as it is the
    // safest option.
    tuner.setZNMode(PIDAutotuner::ZNModeBasicPID);

    // This must be called immediately before the tuning loop
    // Must be called with the current time in microseconds
    tuner.startTuningLoop(micros());

    // Run a loop until tuner.isFinished() returns true
    long microseconds;
/*    while (!tuner.isFinished()) {

        reflowState = REFLOW_STATE_COMPLETE;
        // This loop must run at the same speed as the PID control loop being tuned
        long prevMicroseconds = microseconds;
        microseconds = micros();

        // Get input value here (temperature, encoder position, velocity, etc)
        double input = doSomethingToGetInput();

        // Call tunePID() with the input value and current time in microseconds
        double output = tuner.tunePID(input, microseconds);

        // Set the output - tunePid() will return values within the range configured
        // by setOutputRange(). Don't change the value or the tuning results will be
        // incorrect.
        doSomethingToSetOutput(output);

        // This loop must run at the same speed as the PID control loop being tuned
        while (micros() - microseconds < loopInterval) delayMicroseconds(1);
    }
*/  
}



/*
void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    output=reflowOvenPIDTuneStartValue;
    reflowOvenPIDTune.SetNoiseBand(reflowOvenPIDTuneNoise);
    reflowOvenPIDTune.SetOutputStep(reflowOvenPIDTuneStep);
    reflowOvenPIDTune.SetLookbackSec((int)reflowOvenPIDTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    reflowOvenPIDTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = reflowOvenPID.GetMode();
  else
    reflowOvenPID.SetMode(ATuneModeRemember);
}


void SerialSend()
{
  Serial.print("setpoint: ");Serial.print(setpoint); Serial.print(" ");
  Serial.print("input: ");Serial.print(input); Serial.print(" ");
  Serial.print("output: ");Serial.print(output); Serial.print(" ");
  if(tuning){
    Serial.println("tuning mode");
  } else {
    Serial.print("kp: ");Serial.print(reflowOvenPID.GetKp());Serial.print(" ");
    Serial.print("ki: ");Serial.print(reflowOvenPID.GetKi());Serial.print(" ");
    Serial.print("kd: ");Serial.print(reflowOvenPID.GetKd());Serial.println();
  }
}

void SerialReceive()
{
  if(Serial.available())
  {
   char b = Serial.read(); 
   Serial.flush(); 
   if((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();
  }
}

void DoModel()
{
  //cycle the dead time
  for(byte i=0;i<49;i++)
  {
    theta[i] = theta[i+1];
  }
  //compute the input
  input = (kpmodel / taup) *(theta[0]-outputStart) + input*(1-1/taup) + ((float)random(-10,10))/100;

}

*/


//-------------------------------------------------------------------------
//                        6. MENU Functions                              //
//-------------------------------------------------------------------------


// Page 0  ( Splash Screen ) ----------------------------------------------------

void splashScreen()
{
  uint16_t width = 240; //tft.width() - 1;
  uint16_t height = 320; //tft.height() - 1;
  uint8_t border = 10;

  tft.fillScreen(BLACK);
  tft.fillRect(border, border, (width - border * 2), (height - border * 2), MYBROWN);
  tft.drawRect(border, border, (width - border * 2), (height - border * 2), AMETHYST);

  tft.setCursor (24, 50);
  tft.setTextSize (4);
  tft.setTextColor(WHITE);
  tft.println("floatLab");
  tft.setCursor (26, 85);
  tft.setTextSize (3);
  tft.println("Reflow Oven");
  tft.setCursor (87, 150);
  tft.setTextSize (2);
  tft.setTextColor(WHITE);
  tft.println(ver);
  //tft.println("v 1.0");
  tft.fillRoundRect(20,200,200,100,16,TURQUOISE);
  tft.setCursor (68, 250);
  tft.setTextSize (1);
  tft.setTextColor(WHITE);
  tft.println("TOUCH to Proceed");

  // Set page number to 0
  //page = 0;
  // wait for one touch
  waitOneTouch();
    
}


// Page 1  ( Main Menu ) ----------------------------------------------------
void enterMenu(){

  page = 1;
  //Serial.println("Main Menu");
  tft.fillScreen(BLACK);
  
  // Reflow !
  tft.fillRoundRect( 20, 20, 200, 160, 12, WHITE );
  tft.fillRoundRect( 25, 25, 190, 150, 12, BLACK );
  tft.setCursor (38, 80);
  tft.setTextSize (4);
  tft.setTextColor(WHITE);
  tft.println("Reflow!");
  
  // Settings
  tft.fillRect( 110, 250, 110, 50, AMETHYST ); 
  tft.setCursor (116, 268);
  tft.setTextSize (2);
  tft.setTextColor(WHITE);
  tft.println("Settings");
  
  // BAKE
  tft.fillRect( 20, 250, 70, 50, TURQUOISE );
  tft.setCursor (38, 268);
  tft.setTextSize (2);
  tft.setTextColor(WHITE);
  tft.println("BAKE");
  
  
  waitOneTouch();
 
}  


// Page 2  ( Profile Select ) ----------------------------------------------------
void profileSelect(){
  
  page = 2;
  //Serial.println("Profile Select");
  tft.fillScreen(BLACK);

  // Lead Free [P\b]
  tft.fillRect( 20, 20, 200, 120, TURQUOISE );
  tft.setCursor (50, 60);
  tft.setTextSize (6);
  tft.setTextColor(WHITE);
  tft.println("RoHS");
    
  // Leaded Solder [Pb]
  tft.fillRect( 20, 170, 200, 120, CONCRETE );
  tft.setCursor (50, 205);
  tft.setTextSize (6);
  tft.setTextColor(WHITE);
  tft.println("SnPb");
  
  waitOneTouch();

  //reflow();

}


// Page 4  ( Settings ) ----------------------------------------------------
void settings(){

  
  page = 4;
  //Serial.println("Settings");
  tft.fillScreen(BLACK);
  
  tft.drawRoundRect(20,20, 200,200, 20, WHITE);
  tft.setCursor (100, 160);
  tft.setTextSize (2);
  tft.setTextColor(WHITE);
  tft.println("Settings");
  
  waitOneTouch();

  enterMenu();
 
}
// ----- Error / Emergency Stop Page --------------------------
void emergencyStop(){

  page = 404;
  //Serial.println("E-STOP");
  
  tft.fillScreen(RED);
  tft.setCursor (40, 180);
  tft.drawRoundRect(10,10, 220,300, 20, WHITE);
  tft.setCursor (100, 160);
  tft.setTextSize (5);
  tft.setTextColor(BLACK);
  tft.println("E-STOP");
  
  waitOneTouch();
  resetFunc();

}

// ----- Error / Emergency Stop Page --------------------------
void tooHot(){

  page = 303;
  //Serial.println("TOO HOT");
  
  tft.fillScreen(RED);
  tft.setCursor (40, 180);
  tft.drawRoundRect(10,10, 220,300, 20, WHITE);
  tft.setCursor (100, 160);
  tft.setTextSize (5);
  tft.setTextColor(BLACK);
  tft.println("TOO HOT");
  
  waitOneTouch();
  resetFunc();
  
}



//------------------------------------------------------------------------
//                    7.  Reflow Graphing Function                      //
//------------------------------------------------------------------------



/*  function to draw a cartesian coordinate systed and plot whatever data you want
    just pass x and y and the graph will be drawn

    

    1.  &d name of your display object        
    2.  x = x data point                      // Time
    3.  y = y datapont                        // Temperature  
    4.  gx = x graph location (lower left)    // Graph Location X (lower left)
    5.  gy = y graph location (lower left)    // Graph Location Y (lower left)
    6.  w = width of graph                    // Width of Graph
    7.  h = height of graph                   // Height of Graph
    8.  xlo = lower bound of x axis           // 
    9.  xhi = upper bound of x asis           //
    10  xinc = division of x axis (distance not count) // X Divisions ( Time )
    11  ylo = lower bound of y axis           //          
    12  yhi = upper bound of y asis           //
    13  yinc = division of y axis (distance not count) // Y Divisions ( Temperature )
 -  14  title = title of graph                // Graph Title
 -  15  xlabel = x asis label                 // X Axis Label
 -  16  ylabel = y asis label                 // Y Axis Label
    17  gcolor = graph line colors            // Graph Line Color
    18  acolor = axi ine colors               // Axis Line Color
    19  pcolor = color of your plotted data   // Color of Plotted Data
 -  20  tcolor = text color                   // Text Color
 -  21  bcolor = background color             // Background Color
    22  &redraw = flag to redraw graph on fist call only  // Redraw flag
*/



//---------------------------------------------------------------------
//                           1         2          3         4          5         6          7           8           9            10          11         12            13             14            15                   16                   17                   18            19    
void reflowGraphOnce( double x, double y, double gx, double gy, double w, double h, double xlo, double xhi, double xinc, double ylo, double yhi, double yinc, unsigned int gcolor, unsigned int acolor, unsigned int pcolor, unsigned int tcolor, unsigned int bcolor, boolean &redraw)
{
  double ydiv, xdiv;
  // initialize old x and old y in order to draw the first point of the graph
  // but save the transformed value
  // note my transform funcition is the same as the map function, except the map uses long and we need doubles
  //static double ox = (x - xlo) * ( w) / (xhi - xlo) + gx;
  //static double oy = (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
  double i;
  double temp;
  int rot, newrot;

  if (redraw == true)
  {
    redraw = false;
    tft.fillScreen(BLACK);
    //ox = (x - xlo) * ( w) / (xhi - xlo) + gx;
    //oy = (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
    
    // draw y scale
    for ( i = ylo; i <= yhi; i += yinc) {
      // compute the transform
      temp =  (i - ylo) * (gy - h - gy) / (yhi - ylo) + gy;

      if (i == 0) {
        tft.drawLine(gx, temp, gx + w, temp, gcolor);
      }
      else {
        tft.drawLine(gx, temp, gx + w, temp, gcolor);
      }
    /*tft.setTextSize(1);
      tft.setTextColor(tcolor, bcolor);
      tft.setCursor(gx - 50, temp);
      // precision is default Arduino--this could really use some format control
      tft.println(i);*/
    }
    // draw x scale
    for (i = xlo; i <= xhi; i += xinc) {

      // compute the transform
      temp =  (i - xlo) * ( w) / (xhi - xlo) + gx;
      if (i == 0) {
        tft.drawLine(temp, gy, temp, gy - h, gcolor);
      }
      else {
        tft.drawLine(temp, gy, temp, gy - h, gcolor);
      }
      
      //tft.setTextSize(1);
      //tft.setTextColor(tcolor, bcolor);
      //tft.setCursor(temp, gy + 10);
      // precision is default Arduino--this could really use some format control
      //tft.println(i);

    }

    if ( profileType == 0) {
      
        // Plot the Reflow Profile for Lead Free Solder (1 min/ x division)
        
        tft.setCursor(190,5);
        tft.setTextSize(2);
        tft.setTextColor(GREEN, bcolor);
        tft.println("RoHS");


        tft.drawLine( 0, 320, 40, 208, gcolor);     // Preheat

        
        tft.drawLine( 40, 208, 60, 160 ,gcolor);    // Ramp To Soak
        
        tft.drawLine( 60, 160, 100, 96 ,gcolor);    // Soak

        
        tft.drawLine( 100, 96, 140, 54 ,gcolor);    // Ramp to Peak

        
        tft.drawLine( 140, 53, 140, 43 ,gcolor);    // DANGER ZONE START - MAX. 10s

        tft.drawLine( 140, 53, 145, 43 , RED);      // 
        
        tft.drawLine( 140, 43, 147, 43 ,gcolor);    // PEAK

        tft.drawLine( 145, 43, 147, 53, RED );      //
        
        tft.drawLine( 147, 43, 147, 53 ,gcolor);    // DANGER ZONE END

        
        tft.drawLine( 147, 54, 167, 96 ,gcolor);    // Go Down
        
        tft.drawLine( 167, 96, 240, 160  ,gcolor);  // Cool
        
        
      } else {

        // Plot the Reflow Profile for SN60Pb39Ag1 (2 min / division)
        
        tft.setCursor(191,5);
        tft.setTextSize(2);
        tft.setTextColor(GREY, bcolor);
        tft.println("SnPb");
        
        tft.drawLine(0,   320, 10,  216, gcolor);
        tft.drawLine(10,  216, 20,  160, gcolor);
        tft.drawLine(20,  160, 60,   96, gcolor);
        tft.drawLine(60,  96,  80,   48 , gcolor);
        tft.drawLine(80,  48,  84,   48 , gcolor);//Peak
        tft.drawLine(84,  48,  104,  96 , gcolor);
        tft.drawLine(104, 96,  120, 107 , gcolor);
        tft.drawLine(120, 107, 200, 160 , gcolor);

        }

    //---------------------------------
    //Title
    //tft.setTextSize(2.5);
    //tft.setTextColor(RED, bcolor);
    //tft.setCursor( gx + 5 , gy - h + 5);
    //tft.println("Reflowing");

    // Celsius Marks on grid
    tft.setTextSize(1); 
    tft.setTextColor(WHITE, bcolor);
    tft.setCursor(220,45);
    tft.println("250");
    tft.setCursor(220,98);
    tft.println("200");
    //tft.setCursor(220,153);
    //tft.println("150");
    tft.setCursor(220,203);
    tft.println("100");
    //tft.setCursor(220,258);
    //tft.println(" 50");
   
  }

  //---------------------------------
  //Title

  if ( reflowStatus = REFLOW_STATUS_ON) {
    tft.setTextSize(2.5);
    tft.setTextColor(RED, bcolor);
    tft.setCursor( gx + 5 , gy - h + 5);
    tft.println("Reflowing");
  }


}


//---------------------------------------------------------------------
//                         1         2          3         4          5         6          7           8           9            10          11         12            13             14            15                   16                   17                   18                    19                  20              21
void reflowGraphTemp(double x, double y, double gx, double gy, double w, double h, double xlo, double xhi, double xinc, double ylo, double yhi, double yinc, String reflowStageStr, String labelTime, String labelTemp, unsigned int gcolor, unsigned int acolor, unsigned int pcolor, unsigned int tcolor, unsigned int bcolor, boolean &redraw)
{
  // Plot the temperature data
  // recall that ox and oy are initialized as static above
  x =  (x - xlo) * ( w) / (xhi - xlo) + gx;
  y =  (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
  tft.drawLine(ox, oy, x, y, pcolor);
  //tft.drawLine(ox, oy + 1, x, y + 1, pcolor);
  tft.drawLine(ox, oy - 1, x, y - 1, pcolor);
  ox = x;
  oy = y;

  // Reflow Stage
  tft.setTextSize(1.5);
  tft.setTextColor(RED, bcolor);
  tft.fillRect(gx + 116 , gy - h + 12, 74, 7, BLACK);
  tft.setCursor( gx + 116 , gy - h + 12);
  tft.println(reflowStageStr);

  // Time Ealapsed ( s )
  tft.setTextSize(3); 
  tft.setTextColor(TURQUOISE, bcolor);
  tft.setCursor(gx + 5 , gy - h + 27 );
  tft.println("ET:" + labelTime + "s");

  // Temperature ( °C )
  tft.setTextSize(2.5);
  tft.setTextColor(pcolor, bcolor);
  tft.fillRect(gx + 55 , gy - h + 56, 20, 14, BLACK);
  tft.setCursor(gx + 5 , gy - h + 56);
  tft.println(labelTemp + "*C");

  

}


//---------------------------------------------------------------------
//                        1         2           3         4         5         6           7             8           9            10          11         12            13             14                   15                   16                   17                   18               19     20       21      22     23
void reflowGraphJunction( double x, double y, double gx, double gy, double w, double h, double xlo, double xhi, double xinc, double ylo, double yhi, double yinc, String junctionTempLabel, String slopeLabel, unsigned int gcolor, unsigned int acolor, unsigned int pcolor, unsigned int bcolor, boolean &redraw) {
  
  // ------------------------------------------------
  // Plot the junctionTemperature data
  x =  (x - xlo) * ( w) / (xhi - xlo) + gx;
  y =  (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
  tft.drawLine(ox2, oy2, x, y, pcolor);
  //tft.drawLine(ox2, oy2 + 1, x, y + 1, pcolor);
  tft.drawLine(ox2, oy2 - 1, x, y - 1, pcolor);
  ox2 = x;
  oy2 = y;

  // ------------------------------------------------
  // Junction Temp ( °C )
  tft.setTextSize(2);
  tft.setTextColor(pcolor, bcolor);
  tft.setCursor(gx + 5 , gy - h + 75);
  tft.println(junctionTempLabel+" *C");

  // ------------------------------------------------
  // Temperature Rise/Fall Slope ( °C/s ) 
  tft.setTextSize(2); 
  tft.setTextColor(PURPLE, bcolor);
  tft.setCursor(gx + 175 , gy - h + 27 );
  tft.fillRect(gx + 170 , gy - h + 27, 40, 14, BLACK);
  tft.println(slopeLabel);
  tft.setTextSize(1.5);
  tft.setCursor(gx + 175 , gy - h + 45 );
  tft.println("*C/s");


}

//---------------------------------------------------------------------
//                        1         2           3         4         5         6           7             8           9            10          11         12            13             14                   15                   16                   17                   18               19     20       21      22     23
void reflowGraphOutputPower( double x, double y, double gx, double gy, double w, double h, double xlo, double xhi, double xinc, double ylo, double yhi, double yinc, String outputPowerLabel, String slopelabel, unsigned int gcolor, unsigned int acolor, unsigned int pcolor, unsigned int bcolor, boolean &redraw) {
  
  // ------------------------------------------------
  // Plot the junctionTemperature data
  x =  (x - xlo) * ( w) / (xhi - xlo) + gx;
  y =  (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
  tft.drawLine(ox3, oy3, x, y, pcolor);
  //tft.drawLine(ox3, oy3 + 1, x, y + 1, pcolor);
  tft.drawLine(ox3, oy3 - 1, x, y - 1, pcolor);
  ox3 = x;
  oy3 = y;

  // ------------------------------------------------
  // Output Power ( % )
  tft.setTextSize(2);
  tft.setTextColor(pcolor, bcolor);
  tft.setCursor(gx + 5 , gy - h + 94);
  tft.println(outputPowerLabel + " %");


}

//---------------------------------------------------------------------
//                        1         2           3         4         5         6           7             8           9            10          11         12            13             14                   15                   16                   17                   18               19     20       21      22     23
void reflowGraphSetpoint( double x, double y, double gx, double gy, double w, double h, double xlo, double xhi, double xinc, double ylo, double yhi, double yinc, String setpointLabel, String ylabel, unsigned int gcolor, unsigned int acolor, unsigned int pcolor, unsigned int bcolor, boolean &redraw) {
  
  // ------------------------------------------------
  // Plot the temperature setpoint data
  x =  (x - xlo) * ( w) / (xhi - xlo) + gx;
  y =  (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
  tft.drawLine(ox4, oy4, x, y, pcolor);
  //tft.drawLine(ox4, oy4 + 1, x, y + 1, pcolor);
  tft.drawLine(ox4, oy4 - 1, x, y - 1, pcolor);
  ox4 = x;
  oy4 = y;

  // ------------------------------------------------
  // Temperature Setpoint ( *C )
  tft.setTextSize(1);
  tft.setTextColor(pcolor, bcolor);
  tft.setCursor(gx + 5 , gy - h + 109);
  tft.println(setpointLabel + " *C");


}

//-----------------------------------------------------------------
/*                     8. Functionality                          */
//-----------------------------------------------------------------



// ----- Start Reflowing --------------------------------------
void reflow(){

  // Set Page number to 3 ( tracking for touches )
  page = 3;
  
  //Serial.println("Reflow started !");
  
  tft.fillScreen(BLACK);

  // wait a sec for everything to draw and initialize
  //delay(1000);

  // Set reflow status ON
  reflowStatus = REFLOW_STATUS_ON;
  reflowState = REFLOW_STATE_IDLE;

  // Start our PID_v2 / not used in PID
  //reflowOvenPID.Start(input, 0, 100);

  // --- Initialize time keeping variables ----------
  
  // Start our main timer (secondsElapsed)
  startMillis = millis();

  // Initialize temp reading timer
  nextTempRead = millis();

  // Initialize graphing timer
  nextGraphPID = millis();



} // Reflow Loop End


// ----- Start Baking ----------------------------------------
void ovenBake(){
  
  page = 5;
  //Serial.println("BAKE");
  tft.fillScreen(BLACK);
  tuning = true;
  
  tft.drawRoundRect(20,20, 200,200, 20, WHITE);
  tft.setCursor (90, 160);
  tft.setTextSize (2);
  tft.setTextColor(WHITE);
  tft.println("TUNE");
  
  waitOneTouch();

  reflow();
  //enterMenu();
  
}



//-----------------------------------------------------------------------
/*                          9. SETUP                                   */
//-----------------------------------------------------------------------

void setup() {

  // --- Zero Crossing ------------------------

  // Initialize ssrPin with default zero.
  digitalWrite(SSRPin, LOW);
  pinMode(SSRPin, OUTPUT);
  digitalWrite(SSRPin, LOW);

  //pinMode(ZCSPin, INPUT );
  //digitalWrite(ZCSPin, HIGH); //enable pull-up resistor

  // set up Timer1 
  //(see ATMEGA 328 data sheet pg 134 for more details)
  //OCR1A = 100;      //initialize the comparator
  //TIMSK1 = 0x03;    //enable comparator A and overflow interrupts
  //TCCR1A = 0x00;    //timer control registers set for
  //TCCR1B = 0x00;    //normal operation, timer disabled

  // set up zero crossing interrupt
  //attachInterrupt(0,zeroCrossingInterrupt, RISING);    
  //IRQ0 is pin 2.
  //Call zeroCrossingInterrupt on rising signal

  // --- Reflow State Initialization ----------
  reflowStatus = REFLOW_STATUS_OFF;
  reflowState = REFLOW_STATE_IDLE;

  // Initialize all the readings to 20-22 ( presumed ambient temperature)
  for (int thisReading = 20; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  
  // --- Touchscreen Pins ---------------------
  pinMode(XP, OUTPUT);
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);
  pinMode(YM, OUTPUT);

  // --- PID Initial Setting ------------------
  reflowOvenPID.SetOutputLimits(0,255); // Max. Output: 100%  (0-255)
  reflowOvenPID.SetMode(AUTOMATIC); // AUTOMATIC, MANUAL, DIRECT, REVERSE
  reflowOvenPID.SetSampleTime(PROCESS_SAMPLING_TIME); // Sampling time ( usually same as sensor sampling time )
  
  //reflowOvenPID.SetTunings(PID_KP_PREHEAT, PID_KI_PREHEAT, PID_KD_PREHEAT);
  reflowOvenPID.SetTunings( aggKp, aggKi, aggKd );

  
  // --- PID Auto-Tuner ------------------

  PIDAutotuner tuner = PIDAutotuner();

  // --- Serial ( only for DEBUG ) ------------------
  //Serial.begin(115200);  

  // --- Screen Initialization ------------------
  tft.reset();
  tft.begin(0x9341);
  tft.setRotation(0); // Portrait Mode

  // --- Click Encoder ------------------
  // Set encoder pins as inputs
  pinMode (inputCLK,INPUT);
  pinMode (inputDT,INPUT);

  // Draw welcome screen
  //splashScreen();

  // enter main menu
  enterMenu();
}



//-------------------------------------------------------------------------
//                           10.  LOOP                                   //
//-------------------------------------------------------------------------

void loop() {

  //-----------------------------------------------------------------------
  //                          REFLOW LOOP                          
  //-----------------------------------------------------------------------

  // If reflow process is on going and it' s time to check things
  while (reflowStatus == REFLOW_STATUS_ON){

    //now = millis();

    int i = 0;

    if (millis() > nextTempRead && i <=4){

      // Check input in the next 100 milliseconds
      nextTempRead += TEMPERATURE_SAMPLING_TIME;

      i++;
      
      // --- Thermocouple Temperature and Junction Temperature -----------------------

      // Read thermocouple temperature and feed it into our graph
      temperature = MAX31855.readThermocouple(CELSIUS);
      //Serial.println("Temperature: ");
      //Serial.println(temperature);


      // --- Smooth -------------------------------------------- 
      
      // Smooth values for our temperature
      // subtract the last reading:
      total = total - readings[readIndex];
      // read from the sensor:
      readings[readIndex] = temperature;
      // add the reading to the total:
      total = total + readings[readIndex];
      // advance to the next position in the array:
      readIndex = readIndex + 1;
    
      // if we're at the end of the array...
      if (readIndex >= numReadings) {
        // ...wrap around to the beginning:
        readIndex = 0;
      }
    
      // calculate the average:
      average = total / numReadings;
      // send it to the computer as ASCII digits
      //Serial.println(average);
      //delay(1);        // delay in between reads for stability

      //Serial.println("Average: ");
      //Serial.print(average);
      
      
    }

    if (millis() > nextGraphPID){

      i = 0;

      //-----------------------------------------------------------------------
      //                          Reflow and Graph                          
      //-----------------------------------------------------------------------

      // Check input in the next seconds
      nextGraphPID += PROCESS_SAMPLING_TIME;

      // Read our timer and update seconds
      currentMillis = millis();
      elapsedMillis = currentMillis - startMillis;
      secondsElapsed = elapsedMillis / 1000;
      secondsString = String(secondsElapsed);

      minutesElapsed = (secondsElapsed / 60 );
      minutesString = String (minutesElapsed);
      //seconds = int(temp % 60 );


      // Assign X for our seconds and divide by ( 4s per pixel) / this way we get 12 minutes on the x axis divided into 6, so 2 minutes per 
      x = secondsElapsed/4;



      //if (secondsElapsed >  360)  { x = x + (60* 6);  }   // 6 minutes
    
      // -----------------------------------------------------------------------------------------
      //              1  2  3   4   5    6    7  8    9   10  11  12     13     14      15     16      17     18
      reflowGraphOnce(x, y, 0, 319, 239, 319, 0, 180, 30, 0, 300, 50, DKGREY, GREEN, ORANGE, ORANGE, BLACK, display1);

/*

      // --- Thermocouple Temperature and Junction Temperature -----------------------

      // Read thermocouple temperature and feed it into our graph
      temperature = MAX31855.readThermocouple(CELSIUS);


      // --- Smooth -------------------------------------------- 
      
      // Smooth values for our temperature
      // subtract the last reading:
      total = total - readings[readIndex];
      // read from the sensor:
      readings[readIndex] = temperature;
      // add the reading to the total:
      total = total + readings[readIndex];
      // advance to the next position in the array:
      readIndex = readIndex + 1;
    
      // if we're at the end of the array...
      if (readIndex >= numReadings) {
        // ...wrap around to the beginning:
        readIndex = 0;
      }
    
      // calculate the average:
      average = total / numReadings;
      // send it to the computer as ASCII digits
      //Serial.println(average);
      //delay(1);        // delay in between reads for stability
  */
      
      // Read junction temperature and feed it into our graph
      junctionTemp = MAX31855.readJunction(CELSIUS);


      // --- PID Loop -------------------------------------------- 
      
      p_input = input ; //save previous reading

      input = average;
      //input = temperature; // (int)temperature;
      
      slope = (input - p_input); // calculate the slope

      // Calculate gap between setpoint and actual temperature to figure out 
      gap = abs( setpoint - input ); //distance away from setpoint

      // PID computation
      reflowOvenPID.Compute();
      // SSR Control
      //analogWrite(SSRPin, output); 

      //Serial.println("Output: ");
      //Serial.print(output);


      // Thermocouple Temp Graph Plot ( Y )
      // ---------------------------------------------------------
      
      // Convert our double slope to string for the display
      dtostrf(slope, 2, 0, slopeString);
      // dtostrf( [doubleVar] , [sizeBeforePoint] , [sizeAfterPoint] , [WhereToStoreIt] )
      slopeInt = String(slopeString);  // cast it to string from char


      // Assign out thermocouple temperature to the reflowGraphTemp Graph Plot
      
      //y = temperature; // without smoothing;
      y = average;       // with smoothing ( 5 reads each 500ms )
       
      // Convert our double Oven Temperature to string for the display
      dtostrf(y, 3, 0, tempString);
      // dtostrf( [doubleVar] , [sizeBeforePoint] , [sizeAfterPoint] , [WhereToStoreIt] )
      //temperatureInt = String(tempString);  // cast it to string from char
      
      // -----------------------------------------------------------------------------------------
      //               1   2  3   4  5   6    7    8   9  10   11  12  13  14      15           16             17        18      19     20       21      22     23
      reflowGraphTemp(x, y, 0, 319, 239, 319, 0, 180, 30, 0, 300, 50, reflowStatusString, secondsString , tempString , DKGREY, GREEN, ORANGE, ORANGE, BLACK, display1);



      // Junction Temp Graph Plot ( Y2 )
      // ---------------------------------------------------------
      
      // Assign our junction temperature to the reflowGraphJunction Graph Plot
      y2 = junctionTemp;
       
      // Convert our double Junctiontemperature to string for the display
      dtostrf(y2, 2, 0, junctionTempString);

      
      // -----------------------------------------------------------------------------------------
      //               1   2  3   4   5    6    7   8  9    10 11  12   13   14                   15     16     17    18      19     20
      reflowGraphJunction(x, y2, 0, 319, 239, 319, 0, 180, 30, 0, 300, 50, junctionTempString , slopeString , DKGREY, GREEN, BLUE, BLACK, display1);


      // Output Power Graph Plot ( Y3 )
      // ---------------------------------------------------------

      // Assign our output power to the reflowGraphOutput Graph Plot
      //y3 = output;
      y3 = map (output, 0, 255, 0, 100);
      
      // Convert our double Junctiontemperature to string for the display
      dtostrf(y3, 3, 0, outputPowerString);
      
      // -----------------------------------------------------------------------------------------
      //               1   2  3   4   5    6    7   8  9    10 11  12   13   14                   15     16     17    18      19     20
      reflowGraphOutputPower(x, y3, 0, 319, 239, 319, 0, 180, 30, 0, 300, 50, outputPowerString , slopeString , DKGREY, GREEN, WHITE, BLACK, display1);


      // Output Setpoint Graph Plot ( Y3 )
      // ---------------------------------------------------------

      // Assign our output power to the reflowGraphOutput Graph Plot
      //y3 = output;
      y4 = setpoint;
      
      // Convert our double Junctiontemperature to string for the display
      dtostrf(y4, 3, 0, setpointString);
      
      // -----------------------------------------------------------------------------------------
      //               1   2  3   4   5    6    7   8  9    10 11  12   13   14                   15     16     17    18      19     20
      reflowGraphSetpoint(x, y4, 0, 319, 239, 319, 0, 180, 30, 0, 300, 50, setpointString , slopeString , DKGREY, GREEN, GREEN, BLACK, display1);


      // --------------------------------------
      
      //Serial.println("Reflowing...");
      //Serial.println(secondsString);
      
      //Serial.println(temperature);

      //Serial.println(input);
   
      //Serial.println(y);

      
      //Serial.println(y2);
      
      //Serial.println(y3);
      
      //Serial.println(slopeString);
     

      //Serial.println();
      //Serial.println();

      
      //delay(1);

      //-----------------------------------------------------------------------
      //                           REFLOW STATE                          
      //-----------------------------------------------------------------------

      
      if (temperature >= TEMPERATURE_ABSOLUTE_MAX_TEMP_MIN) {
        reflowState = REFLOW_STATE_TOO_HOT;
      } else if (temperature >= 10000){
          reflowState = REFLOW_STATE_ERROR;
        }

      // Maximum allowed time for whole process (monitored) if a reflow process has indeed occurred.
      if (secondsElapsed >= REFLOW_DURATION && eutecticTempFlag == true ){
        reflowState = REFLOW_STATE_COMPLETE;
      }


      // ----- Reflow oven controller state machine
      switch (reflowState){
        
        // 1 ----- Case Reflow State IDLE -------------------------------------- 
        case REFLOW_STATE_IDLE:

          // 
          //timerReflow = millis() + ReflowTime;
          //ReflowStartTime

          if ((input <= TEMPERATURE_ROOM) ){
            
            // Proceed to preheat stage
            reflowState = REFLOW_STATE_PREHEAT;
            reflowStatusString = "PREHEAT";
            
          } else if (input >= TEMPERATURE_ROOM || input >= 10000){

            // Stop if something is wrong
            reflowState = REFLOW_STATE_TOO_HOT;
            reflowStatusString = "TOO HOT";
          }

          setpoint = 22;
  
          
          //Serial.println("IDLE");
          //reflowStatusString = "IDLE";
          
        break;
    
        // 2 ----- Case Reflow State PREHEAT -------------------------------------- 
        case REFLOW_STATE_PREHEAT:
          
          //reflowStatus = REFLOW_STATUS_ON;
          
          // Intialize seconds timer for serial debug information
          timerSeconds = 0;
            
          // Initialize PID control window starting time
          timerReflow = millis();
            
          // Set less agressive PID parameters for preheat
          //reflowOvenPID.SetTunings(PID_KP_PREHEAT, PID_KI_PREHEAT, PID_KD_PREHEAT);
          //reflowOvenPID.SetTunings( aggKp, aggKi, aggKd );

          if ( input <= TEMPERATURE_PREHEAT_MIN){

            // Set fixed temperature to ramp up to minimum preheat temperature
            //setpoint = TEMPERATURE_PREHEAT_MIN;
            
            // Ramp up to minimum preheat temperature
            if (setpoint < TEMPERATURE_PREHEAT_MIN ){
              setpoint += PREHEAT_TEMPERATURE_STEP/2;
            }
          }
          
          // Tell the PID to range between 0 and the full window size
          //reflowOvenPID.SetOutputLimits(0, 255);
          //reflowOvenPID.SetSampleTime(PROCESS_SAMPLING_TIME);
            
          // Turn the PID on
          //reflowOvenPID.SetMode(AUTOMATIC);
          
          // If minimum Preheat temperature is achieve
          if (input >= TEMPERATURE_PREHEAT_MIN ){
            
            // Set fixes temperature to ramp up to maximum preheat temperature
            //setpoint = TEMPERATURE_PREHEAT_MAX;
            
            // // Ramp up to minimum preheat temperature
            if ( setpoint < TEMPERATURE_PREHEAT_MAX ){
              setpoint += PREHEAT_TEMPERATURE_STEP/2;
            }
            
          }
          
          // Set our PID parameters for preheat stage.
          //reflowOvenPID.SetTunings(PID_KP_PREHEAT_, PID_KI_PREHEAT, PID_KD_PREHEAT);
  
          // Proceed to Ramp to soak stage if time and temp is in check
          if ( secondsElapsed >= ( PreheatTime ) && input >= TEMPERATURE_PREHEAT_MAX ){ // 

            // Set our PID parameters for preheat stage.
            //reflowOvenPID.SetTunings(PID_KP_RAMP_TO_SOAK, PID_KI_RAMP_TO_SOAK, PID_KD_RAMP_TO_SOAK);
            //reflowOvenPID.SetTunings( aggKp, aggKi, aggKd );

            // Ramp up to first section of Ramp To Soak Stage
            //setpoint = TEMPERATURE_RAMP_TO_SOAK_MIN;
            //setpoint += RAMP_TO_SOAK_TEMPERATURE_STEP/2;

            // Activate Ramp to Soak Stage
            reflowState = REFLOW_STATE_RAMP_TO_SOAK;
            reflowStatusString = "RAMP TO SOAK";

            // Follow timings, can test without setting time, by temperature
            //SoakStartTime = timerSeconds ; // set Soak Start Time
            
          
          }
          //Serial.println("PREHEAT");
          //reflowStatusString = "PREHEAT";
                    
        break;
    
        // 3 ----- Case Reflow State RAMP TO SOAK -------------------------------------- 
        case REFLOW_STATE_RAMP_TO_SOAK:

          //reflowStatus = REFLOW_STATUS_ON;
          
          //timerRampToSoak = millis() + RampToSoakTime;
          //RampToSoakStartTime

          if ( input <= TEMPERATURE_RAMP_TO_SOAK_MIN){

            // Set fixed temperature to ramp up to minimum ramp to soak temperature
            //setpoint = TEMPERATURE_RAMP_TO_SOAK_MIN;

             // Ramp up to max  ramp to soak temperature
            if ( setpoint < TEMPERATURE_RAMP_TO_SOAK_MIN ) {           
              setpoint += RAMP_TO_SOAK_TEMPERATURE_STEP/2;
            }
            
          }
          
          // If minimum soak temperature is achieve
          if (input >= TEMPERATURE_RAMP_TO_SOAK_MIN){
              
            //timerRampToSoak = millis() + RampToSoakTime;

            // Ramp up to second section of Ramp To Soak Stage
            //setpoint = TEMPERATURE_RAMP_TO_SOAK_MAX;

            if ( setpoint < TEMPERATURE_RAMP_TO_SOAK_MAX ){
              setpoint += SOAK_TEMPERATURE_STEP/2;
            }
            // If time and temperature is ok, proceed to soak stage
            if ( secondsElapsed >= ( PreheatTime + RampToSoakTime ) && input >= TEMPERATURE_RAMP_TO_SOAK_MAX ){

              // Set less agressive PID parameters for soaking ramp
              //reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
              //reflowOvenPID.SetTunings( aggKp, aggKi, aggKd );
              
              // Ramp up to first section of Soak Stage
              //setpoint = TEMPERATURE_SOAK_MIN;
              
              //setpoint += SOAK_TEMPERATURE_STEP/2;
              
              // Proceed to ramp to soak state
              reflowState = REFLOW_STATE_SOAK;
              reflowStatusString = "SOAK";
              
              //SoakStartTime = timerSeconds ; // set Soak Start Time
            }
                
          } 
          
          //Serial.println("RAMP TO SOAK");
          //reflowStatusString = "RAMP TO SOAK";

        break;

        // 4 ----- Case Reflow State SOAK -------------------------------------- 
        case REFLOW_STATE_SOAK:

          //if (gap < 10) {   //we're close to setpoint, use conservative tuning parameters
          //  reflowOvenPID.SetTunings(consKp, consKi, consKd);
          //} else {          //we're far from setpoint, use aggressive tuning parameters
          // reflowOvenPID.SetTunings(aggKp, aggKi, aggKd);
          //  }

          //timerSoak = millis() + SoakTime;
          //SoakStartTime

          if ( input < TEMPERATURE_SOAK_MIN ) {

            // Set fixed temperature to ramp to 
            //setpoint = TEMPERATURE_SOAK_MIN;
          
            // Ramp up to max  ramp to soak temperature
            if ( setpoint < TEMPERATURE_SOAK_MIN ) {           
              setpoint += SOAK_TEMPERATURE_STEP/2;
            }
            
          }
          
          // If minimum soak temperature is achieved
          if (input >= TEMPERATURE_SOAK_MIN ){

            //timerSoak = millis() + SoakTime;

            // set fixed temperature to ramp up to second section of soaking stage
            //setpoint = TEMPERATURE_SOAK_MAX;

            if (setpoint < TEMPERATURE_SOAK_MAX ){
              setpoint += SOAK_TEMPERATURE_STEP/2;
            }
            
            // If soak time is achieved and temperature is ok, proceed to ramp to soak stage
            if ( secondsElapsed >= ( PreheatTime + RampToSoakTime + SoakTime ) && input >= TEMPERATURE_SOAK_MAX ){

              // Set agressive PID parameters for reflow ramp
              //reflowOvenPID.SetTunings(PID_KP_RAMP_TO_PEAK, PID_KI_RAMP_TO_PEAK, PID_KD_RAMP_TO_PEAK);
              //reflowOvenPID.SetTunings( aggKp, aggKi, aggKd );
                
              // Increment micro setpoint
              //setpoint = TEMPERATURE_RAMP_TO_PEAK_MIN;
              //setpoint += RAMP_TO_PEAK_TEMPERATURE_STEP/2;
              
              // Proceed to reflowing state
              reflowState = REFLOW_STATE_RAMP_TO_PEAK;
              reflowStatusString = "RAMP TO PEAK";
                
              //ReflowStartTime = 0;

            }
          }
          
          //
          //Serial.println("SOAK");
          //reflowStatusString = "SOAK";

        break;


        // 5 ----- Case Reflow State RAMP TO PEAK -------------------------------------- 
        case REFLOW_STATE_RAMP_TO_PEAK:

          //reflowStatus = REFLOW_STATUS_ON;
          //digitalWrite( SSRPin,HIGH );

          // ! ! ! ! ! ! !
          // We need to avoid hovering above 220°C for too long, max 120-150 s as per datasheet
          // Crude method that works like a charm and safe for the components
          //if (input >= TEMPERATURE_RAMP_TO_PEAK_MIN) {

            //Start our above critical temp (220°C) Timer
           // timer 
          //}

          //timerAboveEutecticTemp; // store seconds when above this temp, min. 60 s , max. 150 s
          //MAX_TIME_ABOVE_ETECTIC_TEMPERATURE // 120 // 60 - 150  max
          //TEMPERATURE_ABOVE_EUTECTIC_TEMPERATURE // 217-220 

          //RampToPeakTime = millis() + RamptoPeakTime;
          //RampToPeakStartTime

          // 
          if (input < TEMPERATURE_RAMP_TO_PEAK_MIN){

            // Set fixed temperature to ramp to ramp to peak minimum
            //setpoint = TEMPERATURE_RAMP_TO_PEAK_MIN;
            
            // Ramp to 
            if ( setpoint < TEMPERATURE_RAMP_TO_PEAK_MIN){
              setpoint += RAMP_TO_PEAK_TEMPERATURE_STEP/2;
            }
            
          }

          if (input >= TEMPERATURE_ABOVE_EUTECTIC_TEMPERATURE && eutecticTempFlag == false ){
            timerAboveEutecticTemp = secondsElapsed;
            eutecticTempFlag = true;
          }
          
          // If minimum ramp to peak temperature is achieve
          if (input >= TEMPERATURE_RAMP_TO_PEAK_MIN){

            // Ramp up to second section of ramp to peak stage
            //setpoint = TEMPERATURE_RAMP_TO_PEAK_MAX;

            if ( setpoint < TEMPERATURE_RAMP_TO_PEAK_MAX ){
              setpoint += RAMP_TO_PEAK_TEMPERATURE_STEP/2;
            }
          }
          
          //
          if ( secondsElapsed >= ( PreheatTime + RampToSoakTime + SoakTime + RampToPeakTime ) && input >= TEMPERATURE_RAMP_TO_PEAK_MAX){
            
            // Set PID parameters for reflow dwell
            // Set agressive PID parameters for reflow ramp
            
            //reflowOvenPID.SetTunings( PID_KP_REFLOW_DWELL, PID_KI_REFLOW_DWELL, PID_KD_REFLOW_DWELL);
            //reflowOvenPID.SetTunings( aggKp, aggKi, aggKd );

            
            //reflowOvenPID.SetTunings(PID_KP_REFLOW_DWELL, PID_KI_REFLOW_DWELL, PID_KD_REFLOW_DWELL);
            
            // Ramp up to first section of soaking temperature
            //setpoint = TEMPERATURE_REFLOW_DWELL_MIN;
            //setpoint += REFLOW_DWELL_TEMPERATURE_STEP;
            
            // Proceed to soaking state
            reflowState = REFLOW_STATE_REFLOW_DWELL;
            reflowStatusString = "DWELL";
            
            //SoakStartTime = timerSeconds ; // set Soak Start Time
          }
          
          //
          //Serial.println("RAMP TO PEAK");
          //reflowStatusString = "RAMP TO PEAK";
          
        break;
    
        // 6 ----- Case Reflow State REFLOW -------------------------------------- 
        case REFLOW_STATE_REFLOW_DWELL:

          //timerReflowDwell = secondsElapsed + ReflowDwellTime;
          //ReflowDwellStartTime

          // ! ! ! ! ! ! !
          // We need to avoid hovering at peak temperature for too long, max 10s as per datasheet
          // Crude method that works like a charm and safe for the components

          
          if ( input < TEMPERATURE_REFLOW_DWELL_MIN ){
            if ( setpoint < TEMPERATURE_REFLOW_DWELL_MIN ){
              setpoint += REFLOW_DWELL_TEMPERATURE_STEP/2;
            }
          }
          
          if (input >= (TEMPERATURE_REFLOW_DWELL_MIN)){

            // Ramp up to second section of reflow (dwell) stage
            //setpoint = TEMPERATURE_REFLOW_DWELL_MAX;

            //
            if ( setpoint < TEMPERATURE_REFLOW_DWELL_MAX ){
              setpoint += REFLOW_DWELL_TEMPERATURE_STEP/2;
            }

            // Flag, there was actually a reflow.
            eutecticTempFlag = false;
          }
          
          if ( secondsElapsed >= ( PreheatTime + RampToSoakTime + SoakTime + RampToPeakTime + ReflowDwellTime ) && input >= TEMPERATURE_REFLOW_DWELL_MAX -1 ){

            // Set PID parameters for cooling ramp ( maybe a second pid for fan, future )
            //reflowOvenPID.SetControllerDirection(DIRECT);
            //reflowOvenPID.SetTunings( aggKp, aggKi, aggKd );
            //reflowOvenPID.SetTunings(PID_KP_GO_DOWN, PID_KI_GO_DOWN, PID_KD_GO_DOWN);
            
            // Ramp down to minimum go down temperature
            //setpoint -= GO_DOWN_TEMPERATURE_STEP/2;
            //setpoint = TEMPERATURE_GO_DOWN_MIN;
            
            // Proceed to cooling state
            reflowState = REFLOW_STATE_GO_DOWN;
            reflowStatusString = "GO DOWN";

          }
          //
          //Serial.println("REFLOW");
          //reflowStatusString = "DWELL";
          
        break;
    
         // 7 ----- Case Reflow State GO DOWN -------------------------------------- 
        case REFLOW_STATE_GO_DOWN:

          // Set PID parameters for cooling ramp ( maybe a second pid for fan, future )
          //reflowOvenPID.SetControllerDirection(REVERSE);

          //

            if (input <= TEMPERATURE_ABOVE_EUTECTIC_TEMPERATURE && eutecticTempFlag == false ){
            timerAboveEutecticTemp = secondsElapsed;
            eutecticTempFlag = true;
            }

            // Ramp up to second section of go down stage
            //setpoint = TEMPERATURE_GO_DOWN_MAX;
            setpoint -= GO_DOWN_TEMPERATURE_STEP/2;
            
            if ( secondsElapsed >= ( PreheatTime + RampToSoakTime + SoakTime + RampToPeakTime + ReflowDwellTime + GoDownTime ) && input >= TEMPERATURE_GO_DOWN_MAX  ){

              // Set PID parameters for cooling ramp ( maybe a second pid for fan, future )
              //reflowOvenPID.SetTunings(PID_KP_GO_DOWN, PID_KI_GO_DOWN, PID_KD_GO_DOWN);
              //reflowOvenPID.SetControllerDirection(DIRECT);
              //reflowOvenPID.SetTunings( aggKp, aggKi, aggKd );
              
              // Ramp down to minimum go down temperature
              //setpoint = TEMPERATURE_COOL_MIN;
              setpoint -= COOL_TEMPERATURE_STEP/2;
              
              //setpoint = 0;
              //setpoint -= TEMPERATURE_SLOPE_MAX;
              
              // Proceed to cooling state
              reflowState = REFLOW_STATE_COOL;
              reflowStatusString = "COOL";

            }

          //
          //Serial.println("GO DOWN");
          //reflowStatusString = "GO DOWN";
       
        break;
    
        // 8 ----- Case Reflow State COOL -------------------------------------- 
        case REFLOW_STATE_COOL:

          
          // If minimum cool temperature is achieve
          if (input <= TEMPERATURE_COOL_MIN){

            setpoint = TEMPERATURE_COOL_MAX;

            if ( secondsElapsed >= ( PreheatTime + RampToSoakTime + SoakTime + RampToPeakTime + ReflowDwellTime + GoDownTime ) && input >= TEMPERATURE_COOL_MAX  ){
              
              // Ramp down to minimum go down temperature
              setpoint = TEMPERATURE_COOL_MIN;
              //setpoint -= COOL_TEMPERATURE_STEP;;
              
              // Proceed to reflow Completion state
              reflowState = REFLOW_STATE_COMPLETE;
              //reflowStatusString = "COMPLETE";

            }
           
          }

          // Check if correct time has passed, and an actual dwell stage was done
          if ( secondsElapsed >= ReflowTime && eutecticTempFlag == true ){
            
            // Reflow process ended
            reflowStatus = REFLOW_STATUS_OFF;
            reflowStatusString = "COMPLETE";
          }
          
          //Serial.println("COOL");
          //reflowStatusString = "COOL";
          
        break;
    
        // 9 ----- Case Reflow State COMPLETE -------------------------------------- 
        case REFLOW_STATE_COMPLETE:

          // timer

          // Set everything low.
          setpoint = 0;
          digitalWrite( SSRPin, LOW );

  
          delay(15000);
          //enterMenu();
          resetFunc();
            
        break;
    
    
        // 10 ----- Case Reflow State ERROR -------------------------------------- 
        case REFLOW_STATE_ERROR:
          
          reflowStatus == REFLOW_STATUS_OFF;
          setpoint = 0;
          digitalWrite( SSRPin, LOW );
          
          // If thermocouple is still not connected
          if (input == THERMOCOUPLE_DISCONNECTED){

            reflowStatus == REFLOW_STATUS_OFF;
            setpoint = 0;
            digitalWrite( SSRPin, LOW );
            
            // Wait until thermocouple wire is connected
            reflowState = REFLOW_STATE_ERROR;
            
            //digitalWrite(SSRPin,LOW);
            //Serial.println("Thermocouple disconnected");
            reflowStatusString = "            ";
            reflowStatusString = "THERM DISCO";
            
          } else {
                  // Clear to perform reflow process
                  reflowState = REFLOW_STATE_IDLE;
                  //reflowState = REFLOW_STATE_ERROR;

                  //Serial.println("IDLE");
                  reflowStatusString = "IDLE";
                }
          
          //
          //emergencyStop();
          //Serial.println("Other ERROR");
          //reflowStatusString = "ERROR";
          
        break;
        

        // 11 ----- Case Reflow State ERROR -------------------------------------- 
        case REFLOW_STATE_TOO_HOT:
          
          reflowStatus == REFLOW_STATUS_OFF;
          setpoint = 0;

          if ( input == TEMPERATURE_ABSOLUTE_MAX_TEMP_MIN ) {
              
            // Error Mode, Stop Heaters
            reflowState = REFLOW_STATE_ERROR;
            reflowStatus == REFLOW_STATUS_OFF;
            setpoint = 0;

            //Start Fan gently 
            //analogWrite(FANPin, 127);
            
            //Serial.println("TOO HOT");
            reflowStatusString = "TOO HOT";
              
            } else if ( input == TEMPERATURE_ABSOLUTE_MAX_TEMP_MAX ){
  
                // Error Mode, Stop Heaters, Prompt to Open Door 
                reflowState = REFLOW_STATE_ERROR;
                reflowStatus == REFLOW_STATUS_OFF;
                setpoint = 0;

                //Start Fan 100% otherwise something burns.
                //analogWrite(FANPin, 255);

                //Serial.println("TOO HOT");
                reflowStatusString = "TOO HOT";
                
                //tooHot();
                
              } else {
                  // Clear to perform reflow process
                  reflowState = REFLOW_STATE_IDLE;
                  //reflowState = REFLOW_STATE_ERROR;

                  //Serial.println("IDLE");
                  reflowStatusString = "IDLE";
                }
          
          //
          //tooHot();
          //Serial.println("TOO HOT");
          reflowStatusString = "TOO HOT";
          
        break;

        // 12 ----- Case Reflow State TUNE -------------------------------------- 
        case REFLOW_STATE_TUNE:

          // Start Auto-Tuning Process
          //changeAutoTune();
          reflowOvenPIDAutoTuner();
          // Clear to perform reflow process
          reflowState = REFLOW_STATE_IDLE;
          
          //
          //Serial.println("TUNE");
          reflowStatusString = "TUNE";

        break;
        
        // 13 ----- Case Reflow State Default -------------------------------------- 
        // Default, fallback state
        default:
          //Serial.println("OTHER Case Achieved");
          //enterMenu();
          //bakeState = BAKE_STATE_IDLE;
          
          // if nothing else matches, do the default
          // default is optional
        break;


      } // Reflow State Machine Switch End

      
    } // millis > nextGraphPID END



    // SSR Control
    analogWrite(SSRPin, output); 




    //-----------------------------------------------------------------------
    //                          Switch/Button/Encoder ?                        
    //-----------------------------------------------------------------------



    
/*    
    // If switch 1 is pressed
    if (switchStatus == SWITCH_1)
    {
      // If currently reflow process is on going
      if (reflowStatus == REFLOW_STATUS_ON)
      {
        // Button press is for cancelling
        // Turn off reflow process
        reflowStatus = REFLOW_STATUS_OFF;
        // Reinitialize state machine
        reflowState = REFLOW_STATE_IDLE;
      }
    } 

    // Simple switch debounce state machine (for switch #1 (both analog & digital
    // switch supported))
    switch (debounceState){
      case DEBOUNCE_STATE_IDLE:
        // No valid switch press
        switchStatus = SWITCH_NONE;
        // If button is pressed
          if (digitalRead(inputBTN) == 1)
          {
            // Intialize debounce counter
            lastDebounceTime = millis();
            // Proceed to check validity of button press
            debounceState = DEBOUNCE_STATE_CHECK;
          } 
        break;
    
      case DEBOUNCE_STATE_CHECK:
    
     
          if (digitalRead(inputBTN) == 0){
            // If minimum debounce period is completed
            if ((millis() - lastDebounceTime) > DEBOUNCE_PERIOD_MIN)
            {
              // Proceed to wait for button release
              debounceState = DEBOUNCE_STATE_RELEASE;
            }
          } else { // False trigger
            // Reinitialize button debounce state machine
            debounceState = DEBOUNCE_STATE_IDLE; 
          }
        break;
    
      case DEBOUNCE_STATE_RELEASE:
      
          if (digitalRead(inputBTN) == LOW){
          // Valid switch 1 press
          switchStatus = SWITCH_1;
          // Reinitialize button debounce state machine
          debounceState = DEBOUNCE_STATE_IDLE; 
        }
        break;
    }
*/


    
    
  } // while REFLOW status






  

  //Serial.println("Reflow Ended");
  //digitalWrite(SSRPin, LOW);
  
  //enterMenu();
  //resetFunc();















  //-----------------------------------------------------------------------
  //                           OVEN BAKE                          
  //-----------------------------------------------------------------------





  // -----------------------------------------------------------
  // If we are before a reflow or we are not baking check for touch buttons
  if (reflowStatus == REFLOW_STATUS_OFF){

    // check Encoder (while in menu)
    //checkClickEncoder();
  
    // Touchscreen & Menu Stuff
    getTouch();
    
    // Get Touches on Menu Pages
    switch (page) {

      // Get Touches on splash screen Page 0
      case 0:
        //if (p.x > 20 && p.x < 200 && p.y > 100 && p.y < 200){
        //  enterMenu();
        //}
      break;

      
      // Get Touches on Main Menu
      case 1:
        if (p.x > 20 && p.x < 200 && p.y > 20 && p.y < 200 ){
  
        profileSelect();
          } else if (p.x > 150 && p.x < 220 && p.y > 200 && p.y < 320){
              settings();
            } else if (p.x > 20 && p.x < 70 && p.y > 250 && p.y < 320)
                ovenBake();
      break;

      // Get Touches on Reflow Menu ( Select Profile )
      case 2:
        if(p.x > 20 && p.x < 220 && p.y > 20 && p.y < 130){
          //Serial.println("Lead-Free Profile Selected");
          profileType = 0;
          reflow();
        } else if(p.x > 20 && p.x < 220 && p.y > 170 && p.y < 280){
            //Serial.println("Leaded Profile Selected");
            profileType = 1;
            reflow();
          }
      break;

      // Reflow Page
      case 3:
        if(p.x > 20 && p.x < 220 && p.y > 20 && p.y < 100) {
          enterMenu();
        }
      break;

      //
      case 4:
        if(p.x > 20 && p.x < 220 && p.y > 20 && p.y < 100){
                enterMenu();
        }
      break;

      // Bake Page (  )
      case 5:
        // Bake
      break;

      //
      case 6:
        // 
      break;

      //
      case 303:
        // Too HOT ! - disable ssr, and prompt to open door or bring fire extinguisher.
      break;

      //
      case 404:
        // In case of Emergency stop, disable ssr, and prompt to open door or bring fire extinguisher.
      break;

      //
      default:
      //enterMenu();
      // if nothing else matches, do the default
      // default is optional
      break;

    }// Looking for touch on pages switch end

  } // if not reflowing or baking END


//  // Zero Crossing Interrupt ( Solid State Relay FOTEK SSR-25 DA already switches on zero crossing )
//  if (micros() >= nextCheck ) {  
//    i--;
//    OCR1A = i;     //set the compare register brightness desired.
//    if (i<65){i=483;}                      
//    //delay(15);   
//  }
//
}// loop end



// ------------------ LOOP END ------------------------------------------------------
