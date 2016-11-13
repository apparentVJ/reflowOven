/* 
*
*     floatLab reflowOven    
*    
*
* 
* 235-|                                                 x
*     |                                               x   x  within 5°C [10-30s]
*     |                                            x        x
*     |                                          x              x
*     |                                       x                    x
* 183-|------------------------------------x                          x
*     |                              x                                    x   
*     |                         x         |                          |       x
*     |                    x              |                          |
*     |               x                   |                          |
*     |             x |                   |                          |
*     |           x   |                   |                          | 
* 100-|---------x-----|                   |                          | 
*     |       x       |                   |                          | 
*     |     x         |                   |                          |
*     |   x           |                   |                          |
*     | x  max. 3°C/s |                   |                          |
*     |<  60 - 90 s  >|<    90 - 120 s   >|<       90 - 120 s       >|   max. 6°C/s
*     | Preheat Stage |   Soaking Stage   |       Reflow Stage       | Cool
*     |  100°C-150°C  |    max. 183°C     |                          |
*  0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
*
*
*
*
*
*
*
*
*/











//-------------------------------------------------------------
/*                     1.  INCLUDES                          */
//-------------------------------------------------------------

//#include <SPI.h>
//#include <avr/eeprom.h>
//#include <EEPROM.h>

//------ PID Library ------------------------------------------
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

//------ Thermocouple -----------------------------------------
#define USE_MAX31855

#ifdef  USE_MAX31855
  #include <MAX31855.h>
#else
  #include <max6675.h>
#endif

//------ Menu, Encoder & Timer --------------------------------
#include <ClickEncoder.h>
#include <TimerOne.h>
#include <Menu.h>

//------ Display & TouchScreen --------------------------------
#include <SPFD5408_Adafruit_TFTLCD.h>
#include <SPFD5408_TouchScreen.h>

//------ Transistor - Solid State Relay -----------------------

  const unsigned char ssrPin = 2;











//-------------------------------------------------------------
/*                     2.   DEFINES                          */
//-------------------------------------------------------------


//------ Touchscreen ------------------------------------------

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

  TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

//  Display -------------------------------------------------

#define LCD_RST A4
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0

  Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RST);


// 3. Thermocouple --------------------------------------------

#ifdef  USE_MAX31855
  const  unsigned  char thermocoupleSO = 12;
  const  unsigned  char thermocoupleCS = 10;
  const  unsigned  char thermocoupleCLK = 13;
  
    MAX31855  MAX31855(thermocoupleSO, thermocoupleCS, thermocoupleCLK);
#else

  const unsigned char thermocoupleSOPin = 12;
  const unsigned char thermocoupleCSPin = 10;
  const unsigned char thermocoupleCLKPin = 13;

    MAX6675 thermocouple(thermocoupleCLKPin, thermocoupleCSPin, thermocoupleSOPin);
    
#endif
 


//  PID Algorithm -------------------------------------------

double Setpoint, Input, Output; // variables
  
double aggKp=4, aggKi=0.2, aggKd=1;         // aggressive 
double consKp=1, consKi=0.05, consKd=0.25;  // conservative 
  
  PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
 
  
// Rotary Encoder ------------------------------------------

#define clampValue(val, lo, hi) if (val > hi) val = hi; if (val < lo) val = lo;
#define maxValue(a, b) ((a > b) ? a : b)
#define minValue(a, b) ((a < b) ? a : b)


  ClickEncoder Encoder(10, 11, 12);

    
// Colors --------------------------------------------------

#define DKGREY    0x4A49
#define CLOUDS    0xEF9E
#define TURQUOISE 0x1DF3
#define AMETHYST  0x9AD6
#define MYBROWN   0xA301
#define CONCRETE  0x9534
#define ORANGE    0xFC00
#define POMEGRANATE 0xC1C5


// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE
{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_PREHEAT,
  REFLOW_STATE_SOAK,
  REFLOW_STATE_REFLOW,
  REFLOW_STATE_COOL,
  REFLOW_STATE_COMPLETE,
  REFLOW_STATE_TOO_HOT,
  REFLOW_STATE_ERROR
} reflowState_t;

typedef enum REFLOW_STATUS
{
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
} reflowStatus_t;

typedef enum SWITCH
{
  SWITCH_NONE,
  SWITCH_1, 
  SWITCH_2
} switch_t;

typedef enum DEBOUNCE_STATE
{
  DEBOUNCE_STATE_IDLE,
  DEBOUNCE_STATE_CHECK,
  DEBOUNCE_STATE_RELEASE
} debounceState_t;

// ***** CONSTANTS *****
#define TEMPERATURE_ROOM 21
#define TEMPERATURE_SOAK_MIN 150
#define TEMPERATURE_SOAK_MAX 200
#define TEMPERATURE_REFLOW_MAX 200
#define TEMPERATURE_COOL_MIN 100
#define SENSOR_SAMPLING_TIME 500
#define SOAK_TEMPERATURE_STEP 5
#define SOAK_MICRO_PERIOD 9000
#define DEBOUNCE_PERIOD_MIN 50

// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 100
#define PID_KI_PREHEAT 0.025
#define PID_KD_PREHEAT 20
// ***** SOAKING STAGE *****
#define PID_KP_SOAK 300
#define PID_KI_SOAK 0.05
#define PID_KD_SOAK 250
// ***** REFLOW STAGE *****
#define PID_KP_REFLOW 300
#define PID_KI_REFLOW 0.05
#define PID_KD_REFLOW 350
#define PID_SAMPLE_TIME 1000

// ***** LCD MESSAGES *****
const char* lcdMessagesReflowStatus[] = {
  "Ready",
  "Pre-heat",
  "Soak",
  "Reflow",
  "Cool",
  "Complete",
  "Wait,hot",
  "Error"
};

// ***** DEGREE SYMBOL FOR LCD *****
unsigned char degree[8]  = {
  140,146,146,140,128,128,128,128};

// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
int windowSize;
unsigned long windowStartTime;
unsigned long nextCheck;
unsigned long nextRead;
unsigned long timerSoak;
unsigned long buzzerPeriod;

// Reflow oven controller state machine state variable
reflowState_t reflowState;
// Reflow oven controller status
reflowStatus_t reflowStatus;
// Switch debounce state machine state variable
debounceState_t debounceState;
// Switch debounce timer
long lastDebounceTime;
// Switch press status
switch_t switchStatus;
// Seconds timer
int timerSeconds;

// Specify PID control interface
//PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);








//-------------------------------------------------------------
/*                  3.   VARIABLES                           */
//-------------------------------------------------------------

// Display variables
boolean display1 = true;

double ox , oy ;
double ox2, oy2 ;

double x = 1.0 ;
double y, y2;

uint16_t page = 0;


// Temperature measurements
double temperature;
double junctionTemp;


//Graph timings
const long interval = 1000; // 1px / s
unsigned long currentMillis;
unsigned long previousMillis = 0;


// Touch point
TSPoint p;


// Encoder Menu
int16_t encMovement;
int16_t encAbsolute;
int16_t encLastAbsolute = -1;

bool updateMenu = false;


uint8_t menuItemsVisible = 8;
uint8_t menuItemHeight = 40;

bool menuUpdateRequest = true;
bool initialProcessDisplay = false;

//unsigned char degree[8] = {140,146,146,140,128,128,128,128};










//-----------------------------------------------------------------------
/*            4.   Custom Functions (little helpers)                   */
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

  } while ((p.z < MINPRESSURE ) || (p.z > MAXPRESSURE));

  return p;
}

//----------Return Touch Coordinates -------------------------
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

// Border ----------------------------------------------------

void drawBorder()
{
  uint16_t width = tft.width() - 1;
  uint16_t height = tft.height() - 1;
  uint8_t border = 10;

  tft.fillScreen(BLACK);
  tft.fillRect(border, border, (width - border * 2), (height - border * 2), MYBROWN);//BLACK);
  tft.drawRect(border, border, (width - border * 2), (height - border * 2), AMETHYST);//WHITE);
  
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
  tft.println("v_0.1");
  tft.fillRoundRect(20,200,200,100,16,TURQUOISE);
  tft.setCursor (68, 250);
  tft.setTextSize (1);
  tft.setTextColor(WHITE);
  tft.println("Touch to proceed");
    
}

// Read Thermocouple Temperature ( MAX31866 / MAX6675 )

void readThermo(int delayInterval)
{
  
  temperature = MAX31855.readThermocouple(CELSIUS);
  Serial.print("Thermocouple temperature: ");
  Serial.println(temperature);
  //Serial.println(" Degree Celsius");
  
  // Retrieve cold junction temperature in Degree Celsius
  //junctionTemp = MAX31855.readJunction(CELSIUS);
  //Serial.print("Junction temperature: ");
  //erial.println(junctionTemp);
  //Serial.println(" Degree Celsius");
   
  delay(delayInterval);

  //Serial.println(delayInterval);


  
}




// Encoder ----------------------------------------------------

class ScopedTimer {
public:
  ScopedTimer(const char * Label)
    : label(Label), ts(millis())
  {
  }
  ~ScopedTimer() {
    Serial.print(label); Serial.print(": ");
    Serial.println(millis() - ts);
  }
private:
  const char *label;
  const unsigned long ts;
};

void timerIsr(void) {
  Encoder.service();
}











//-------------------------------------------------------------
/*                       ENCODER MENU                        */
//-------------------------------------------------------------

Menu::Engine *engine;

namespace State {  
  typedef enum SystemMode_e {
    None      = 0,
    Default   = (1<<0),
    Settings  = (1<<1),
    Edit      = (1<<2)
  } SystemMode;
};

uint8_t systemState = State::Default;
bool lastEncoderAccelerationState = true;
uint8_t previousSystemState = State::None;

bool menuExit(const Menu::Action_t a) {
  Encoder.setAccelerationEnabled(lastEncoderAccelerationState);  
  systemState = State::Default;
  return true;
}

bool menuDummy(const Menu::Action_t a) {
  return true;
}
/*
bool menuBack(const Menu::Action_t a) {
  if (a == Menu::actionDisplay) {
    engine->navigate(engine->getParent(engine->getParent()));
  }
  return true;
}
*/

bool menuBack(const Menu::Action_t a) {
  if (a == Menu::actionDisplay) {
    engine->navigate(engine->getParent());
  }
  return true;
}

void renderMenuItem(const Menu::Item_t *mi, uint8_t pos) {
  //ScopedTimer tm("  render menuitem");

  uint8_t y = pos * menuItemHeight + 2;

  tft.setCursor(10, y);

  // a cursor
  tft.drawRect(8, y - 2, 90, menuItemHeight, (engine->currentItem == mi) ?   CLOUDS : BLACK);
  tft.print(engine->getLabel(mi));

  // mark items that have children
  if (engine->getChild(mi) != &Menu::NullItem) {
    tft.print(" >   ");
  }
}

// Name, Label, Next, Previous, Parent, Child, Callback
MenuItem(miExit, "MENU", Menu::NullItem, Menu::NullItem, Menu::NullItem, miSettings, menuExit);

// Settings
MenuItem(miSettings, "Settings", miEditProfile, Menu::NullItem, miExit, Menu::NullItem, menuDummy);

// Reflow Profile
MenuItem(miEditProfile, "Reflow Profiles",  miPIDSettings,    miEditProfile,        miExit,          miRampUpRate,     menuDummy);

        // Name,           Label,             Next,             Previous,           Parent,          Child,            Callback
  MenuItem(miRampUpRate,   "RampUpRate" ,     miRampDownRate,   Menu::NullItem,     miEditProfile,   Menu::NullItem,   menuDummy);
  MenuItem(miRampDownRate, "RampDownRate",    miPreheatTime,    miRampUpRate,       miEditProfile,   Menu::NullItem,   menuDummy);
  
  MenuItem(miPreheatTime,  "Preheat Time",    miPreheatTemp,    miRampDownRate,     miEditProfile,   Menu::NullItem,   menuDummy);
  MenuItem(miPreheatTemp,  "Preheat Temp",    miSoakTime,       miPreheatTime,      miEditProfile,   Menu::NullItem,   menuDummy);
  
  MenuItem(miSoakTime,     "Soak Time",       miSoakTemp,       miPreheatTemp,      miEditProfile,   Menu::NullItem,   menuDummy);
  MenuItem(miSoakTemp,     "Soak Temp",       miPeakTime,       miSoakTime,         miEditProfile,   Menu::NullItem,   menuDummy);
  
  MenuItem(miPeakTime,     "Peak Time",       miPeakTemp,       miSoakTemp,         miEditProfile,   Menu::NullItem,   menuDummy);
  MenuItem(miPeakTemp,     "Peak Temp",       miLoadProfile,    miPeakTime,         miEditProfile,   Menu::NullItem,   menuDummy);

  MenuItem(miLoadProfile,  "Load Profile",    miSaveProfile,    miPeakTemp,         miEditProfile,   Menu::NullItem,   menuDummy);
  MenuItem(miSaveProfile,  "Save Profile",    miFactoryReset,   miLoadProfile,      miEditProfile,   Menu::NullItem,   menuDummy);
  
  MenuItem(miFactoryReset, "FactoryReset",    miCalibBack,      miSaveProfile,      miEditProfile,   Menu::NullItem,   menuDummy);
  MenuItem(miCalibBack,    "Back",            Menu::NullItem,   miFactoryReset,     miEditProfile,   Menu::NullItem,   menuBack);
  
 
// PID Calibration
MenuItem(miPIDSettings, "PID",              Menu::NullItem,   miEditProfile,        miExit,        miCalibrateP,      menuDummy);

  MenuItem(miCalibrateP, "Calibrate P",       miCalibrateI,     Menu::NullItem,      miPIDSettings,     Menu::NullItem,   menuDummy );
  MenuItem(miCalibrateI, "Calibrate I",       miCalibrateD,     miCalibrateP,        miPIDSettings,     Menu::NullItem,   menuDummy );
  MenuItem(miCalibrateD, "Calibrate D",       miCalibBack1,     miCalibrateI,        miPIDSettings,     Menu::NullItem,   menuDummy );
  MenuItem(miCalibBack1,  "Back",             Menu::NullItem,   miCalibrateD,        miPIDSettings,     Menu::NullItem,   menuBack);
  














//-----------------------------------------------------------------------
/*                             SETUP                                   */
//-----------------------------------------------------------------------

void setup() {

  // Initialize ssrPin with default zero.
  digitalWrite(ssrPin, LOW);
  pinMode(ssrPin, OUTPUT);

  Serial.begin(9600);  

  // Screen Initialization
  tft.reset();
  tft.begin(0x9341);
  tft.setRotation(0);

  // Draw welcome screen
  drawBorder();

  // Set page number to 0
  page = 0;

  // wait for one touch
  waitOneTouch(); 

  // enter main menu
  enterMenu();

    
  // Encoder stuff
  
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr); 
  
  pinMode(MISO, OUTPUT);
  pinMode(MOSI, OUTPUT);
  
  // enable pull up, otherwise display flickers
  PORTB |= (1 << MOSI) | (1 << MISO);  
  engine = new Menu::Engine(&Menu::NullItem);
  menuExit(Menu::actionDisplay); // reset to initial state
  
  //----------------------
  pinMode(XP, OUTPUT);
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);
  pinMode(YM, OUTPUT);
}

//-----------------------------------------------------------------------
/*                             LOOP                                   */
//-----------------------------------------------------------------------

void loop() {
  
  currentMillis = millis();
  
  getTouch();
  
  // Get button touches on different pages
  if (page == 1){
    if (p.x > 20 && p.x < 200 && p.y > 20 && p.y < 200 )
    {
    profileSelect();
    
      } else if (p.x > 150 && p.x < 220 && p.y > 200 && p.y < 320)
      {
        settings();
      }
  } else if (page == 2){
     
      if(p.x > 20 && p.x < 220 && p.y > 20 && p.y < 130){
          //Serial.println("Leaded Profile Selected");
        reflow();
    
      } else if(p.x > 20 && p.x < 220 && p.y > 170 && p.y < 280){
          //Serial.println("Lead-Free Profile Selected");
      
          reflow();
      }
    } else if (page == 3){
      
      
       if(p.x > 20 && p.x < 220 && p.y > 20 && p.y < 100){
            enterMenu();
            }
    
    } else if (page == 4){
             if(p.x > 20 && p.x < 220 && p.y > 20 && p.y < 100){
            enterMenu();
            }
            
    }else if (page == 5){/*
            if(p.x > 20 && p.x < 220 && p.y > 20 && p.y < 100){
            enterMenu();
            }    */
            
            
            
            
            
            
    // handle encoder
  encMovement = Encoder.getValue();
  if (encMovement) {
    encAbsolute += encMovement;

    if (systemState == State::Settings) {
      engine->navigate((encMovement > 0) ? engine->getNext() : engine->getPrev());
      updateMenu = true;
    }
  }

  // handle button
  switch (Encoder.getButton()) {

    case ClickEncoder::Clicked:
      if (systemState == State::Settings) {
        engine->invoke();
        updateMenu = true;
      }
      break;

    case ClickEncoder::DoubleClicked:
      if (systemState == State::Settings) {
        engine->navigate(engine->getParent());
        updateMenu = true;
      }

      if (systemState == State::Default) {
          Encoder.setAccelerationEnabled(!Encoder.getAccelerationEnabled());
          tft.setTextSize(1);
          tft.setCursor(10, 42);
          tft.print("Acceleration: ");
          tft.print((Encoder.getAccelerationEnabled()) ? "on " : "off");
      }
      break;

    case ClickEncoder::Held:
      if (systemState != State::Settings) { // enter settings menu

        // disable acceleration, reset in menuExit()
        lastEncoderAccelerationState = Encoder.getAccelerationEnabled();
        Encoder.setAccelerationEnabled(false);

        tft.fillScreen(BLACK);
        tft.setTextColor(WHITE, BLACK);

        engine->navigate(&miSettings);

        systemState = State::Settings;
        previousSystemState = systemState;
        updateMenu = true;
      }
      break;
  }

  if (updateMenu) {
    updateMenu = false;
    
    if (!encMovement) { // clear menu on child/parent navigation
      tft.fillRect(8, 1, 120, 320, BLACK);
    }

    // simple scrollbar
    Menu::Info_t mi = engine->getItemInfo(engine->currentItem);
    uint8_t sbTop = 0, sbWidth = 8, sbLeft = 232;
    uint8_t sbItems = minValue(menuItemsVisible, mi.siblings);
    uint8_t sbHeight = sbItems * menuItemHeight;
    uint8_t sbMarkHeight = sbHeight * sbItems / mi.siblings;
    uint8_t sbMarkTop = ((sbHeight - sbMarkHeight) / mi.siblings) * (mi.position -1);
    tft.fillRect(sbLeft, sbTop,     sbWidth, sbHeight,     WHITE);
    tft.fillRect(sbLeft, sbMarkTop, sbWidth, sbMarkHeight, POMEGRANATE );

    // debug scrollbar values
#if 0
    char buf[30];
    sprintf(buf, "itms: %d, h: %d, mh: %d, mt: %d", sbItems, sbHeight, sbMarkHeight, sbMarkTop);
    Serial.println(buf);
#endif

    // render the menu
    {
      ScopedTimer tm("render menu");
      engine->render(renderMenuItem, menuItemsVisible);
    }

    {
      ScopedTimer tm("helptext");
      tft.setTextSize(1);
      tft.setCursor(10, 260);
      //tft.print("Doubleclick to ");
      if (engine->getParent() == &miExit) {
        tft.print("exit. ");
      }
      else {
        tft.print("go up.");
      }
    }
  }

  // dummy "application"
  if (systemState == State::Default) {
    if (systemState != previousSystemState) {
      previousSystemState = systemState;
      encLastAbsolute = -999; // force updateMenu
      tft.fillScreen(WHITE);
      tft.setTextColor(BLACK, WHITE);
      tft.setCursor(10, 10);
      tft.setTextSize(2);
      tft.print("Main Screen");

      tft.setTextSize(1);
      tft.setCursor(10, 110);
      tft.print("Hold button for setup");
    }

    if (encAbsolute != encLastAbsolute) {
      encLastAbsolute = encAbsolute; 
      tft.setCursor(10, 30);
      tft.setTextSize(1);
      tft.print("Position:");
      tft.setCursor(70, 30);
      char tmp[10];
      sprintf(tmp, "%4d", encAbsolute);
      tft.print(tmp);
    }
  }
 }
}



















//-------------------------------------------------------------
/*                    CUSTOM MENU                            */
//-------------------------------------------------------------

void enterMenu(){
  
  page = 1;
  
  tft.fillScreen(BLACK);
  
  // Reflow !
  tft.fillRoundRect( 20, 20, 200, 210, 12, WHITE );
  tft.fillRoundRect( 25, 25, 190, 200, 12, BLACK );
  tft.setCursor (36, 100);
  tft.setTextSize (4);
  tft.setTextColor(WHITE);
  tft.println("Reflow!");
  
  // Settings
  tft.fillRect( 110, 250, 110, 50, AMETHYST ); 
  tft.setCursor (116, 268);
  tft.setTextSize (2);
  tft.setTextColor(WHITE);
  tft.println("Settings");
  
/* // PID
  tft.fillRect( 20, 250, 70, 50, TURQUOISE );
  tft.setCursor (38, 268);
  tft.setTextSize (2);
  tft.setTextColor(WHITE);
  tft.println("PID");
  */
  
  waitOneTouch();  
 
 }  

void profileSelect(){
  
  page = 2;
  
  tft.fillScreen(BLACK);

  // Leaded [Pb]
  tft.fillRect( 20, 20, 200, 110, CONCRETE );
  tft.setCursor (85, 55);
  tft.setTextSize (6);
  tft.setTextColor(WHITE);
  tft.println("Pb");
    
  // Lead Free [P\b]
  tft.fillRect( 20, 170, 200, 110, TURQUOISE );
  tft.setCursor (62, 200);
  tft.setTextSize (6);
  tft.setTextColor(WHITE);
  tft.println("RoHS");
  
  waitOneTouch();  

}
  
  

//-------------------------------------------------------------
/*                    CUSTOM MENU                            */
//-------------------------------------------------------------
  
void reflow(){
  
  page = 3;
  tft.fillScreen(BLACK);


  readThermo(500);



  
  // Graph
  for (x = 0; x <= 360; x += 0.001)
  {
     if (currentMillis - previousMillis > interval){
    // previousMillis = currentMillis;
     
    //temperature = MAX31855.readThermocouple(CELSIUS);
    //Serial.println(temperature);    
    y = log(x)*10;
    
    //         1   2  3  4   5    6    7    8  9   10  11 12   13   14            15         16        17        18      19     20     21     22
    GraphTemp(tft, x, y, 0, 319, 239, 319, 0, 180, 30, 0, 300, 50, "Oven Temp", "Time", "Temp" , DKGREY, GREEN, BLUE, ORANGE, BLACK, display1);
  
    y = y - 20;
    
    //         1   2  3  4   5    6    7    8  9   10  11 12   13   14     15     16     17      18     
   GraphError(tft, x, y, 0, 319, 239, 319, 0, 180, 30, 0, 300, 50, DKGREY, GREEN, WHITE, ORANGE, display1);

     }
   }

  resetFunc();

  
}


void settings(){
  page = 4;
  tft.fillScreen(BLACK);
  tft.drawRoundRect(20,20, 200,200, 20, WHITE);
  tft.setCursor (100, 160);
  tft.setTextSize (2);
  tft.setTextColor(WHITE);
  tft.println("Settings");
  waitOneTouch();
 // enterMenu();
}





//--------------------------------------------------------------------
/*                   Custom Graphing Function                       */
//--------------------------------------------------------------------

/*  function to draw a cartesian coordinate systed and plot whatever data you want
    just pass x and y and the graph will be drawn

    1.  &d name of your display object
    2.  x = x data point
    3.  y = y datapont
    4.  gx = x graph location (lower left)
    5.  gy = y graph location (lower left)
    6.  w = width of graph
    7.  h = height of graph
    8.  xlo = lower bound of x axis
    9.  xhi = upper bound of x asis
    10  xinc = division of x axis (distance not count)
    11  ylo = lower bound of y axis
    12  yhi = upper bound of y asis
    13  yinc = division of y axis (distance not count)
 -  14  title = title of graph
 -  15  xlabel = x asis label
 -  16  ylabel = y asis label
    17  gcolor = graph line colors
    18  acolor = axi ine colors
    19  pcolor = color of your plotted data
 -  20  tcolor = text color
 -  21  bcolor = background color
    22  &redraw = flag to redraw graph on fist call only
*/

//---------------------------------------------------------------------
//void GraphTemp(Adafruit_TFTLCD &d, double x, double y, double gx, double gy, double w, double h, double xlo, double xhi, double xinc, double ylo, double yhi, double yinc, String title, String xlabel, String ylabel, unsigned int gcolor, unsigned int acolor, unsigned int pcolor, unsigned int tcolor, unsigned int bcolor, boolean &redraw)

void GraphTemp(Adafruit_TFTLCD &d, double x, double y, double gx, double gy, double w, double h, double xlo, double xhi, double xinc, double ylo, double yhi, double yinc, String title, String xlabel, String ylabel, unsigned int gcolor, unsigned int acolor, unsigned int pcolor, unsigned int tcolor, unsigned int bcolor, boolean &redraw)
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

  if (redraw == true) {

    redraw = false;
    ox = (x - xlo) * ( w) / (xhi - xlo) + gx;
    oy = (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
    // draw y scale
    for ( i = ylo; i <= yhi; i += yinc) {
      // compute the transform
      temp =  (i - ylo) * (gy - h - gy) / (yhi - ylo) + gy;

      if (i == 0) {
        d.drawLine(gx, temp, gx + w, temp, gcolor);
      }
      else {
        d.drawLine(gx, temp, gx + w, temp, gcolor);
      }
    /*d.setTextSize(1);
      d.setTextColor(tcolor, bcolor);
      d.setCursor(gx - 50, temp);
      // precision is default Arduino--this could really use some format control
      d.println(i);*/
    }
    // draw x scale
    for (i = xlo; i <= xhi; i += xinc) {

      // compute the transform
      temp =  (i - xlo) * ( w) / (xhi - xlo) + gx;
      if (i == 0) {
        d.drawLine(temp, gy, temp, gy - h, gcolor);
      }
      else {
        d.drawLine(temp, gy, temp, gy - h, gcolor);
      }
      /*
      d.setTextSize(1);
      d.setTextColor(tcolor, bcolor);
      d.setCursor(temp, gy + 10);
      // precision is default Arduino--this could really use some format control
      d.println(i);*/
    }

    //---------------------------------
    /* //Title
    d.setTextSize(2);
    d.setTextColor(tcolor, bcolor);
    d.setCursor(130 , gy - h + 12);
    d.println(title);
    // Time S
    d.setTextSize(1);
    d.setTextColor(acolor, bcolor);
    d.setCursor(gx + 2 , gy );
    d.println(xlabel);
    // Temp C
    d.setTextSize(1);
    d.setTextColor(acolor, bcolor);
    d.setCursor(gx + 2 , gy - h + 2);
    d.println(ylabel);
   
    // Reflow Zone
    d.setTextSize(1);
    d.setTextColor();
    d.setCursor(gx +2, gy + 6);
    d.println(reflowZone);*/

  }
  //graph drawn now plot the data
  // the entire plotting code are these few lines...
  // recall that ox and oy are initialized as static above
  x =  (x - xlo) * ( w) / (xhi - xlo) + gx;
  y =  (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
  d.drawLine(ox, oy, x, y, pcolor);
  d.drawLine(ox, oy + 1, x, y + 1, pcolor);
  d.drawLine(ox, oy - 1, x, y - 1, pcolor);
  ox = x;
  oy = y;

}

//---------------------------------------------------------------------
void GraphError(Adafruit_TFTLCD &d, double x, double y, double gx, double gy, double w, double h, double xlo, double xhi, double xinc, double ylo, double yhi, double yinc, unsigned int gcolor, unsigned int acolor, unsigned int pcolor, unsigned int bcolor, boolean &redraw)
{
  x =  (x - xlo) * ( w) / (xhi - xlo) + gx;
  y =  (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
  d.drawLine(ox2, oy2, x, y, RED);
  d.drawLine(ox2, oy2 + 1, x, y + 1, RED);
  d.drawLine(ox2, oy2 - 1, x, y - 1, RED);
  ox2 = x;
  oy2 = y;

}  
