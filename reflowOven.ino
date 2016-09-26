/* 
*    floatLab - ReflowOven @ 2016   
*/

//------ Includes ----------------------------------------------------

//#include <SPI.h>


//------ PID Library --------------------------------------------------

//#include <PID_v1.h>

//double Setpoint, Input, Output; // variables

//double aggKp=4, aggKi=0.2, aggKd=1;         // aggressive 
//double consKp=1, consKi=0.05, consKd=0.25;  // conservative 

//PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

//------ Thermocouple --------------------------------------------------
/*
#include<MAX31855.h>


const  unsigned  char thermocoupleSO = 12;
const  unsigned  char thermocoupleCS = 10;
const  unsigned  char thermocoupleCLK = 13;

MAX31855  MAX31855(thermocoupleSO, thermocoupleCS, thermocoupleCLK);
*/

//------ Menu, Encoder & Timer --------------------------------------------------
#include <ClickEncoder.h>
#include <TimerOne.h>
#include <Menu.h>

//------ Display & TouchScreen --------------------------------------------------

#include <SPFD5408_Adafruit_GFX.h>    // Core graphics library
#include <SPFD5408_Adafruit_TFTLCD.h> // Hardware-specific library
#include <SPFD5408_TouchScreen.h>

#define YP A1  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 7   // can be a digital pin
#define XP 6   // can be a digital pin

#define TS_MINX 125
#define TS_MINY 85
#define TS_MAXX 958
#define TS_MAXY 905

/*
#if defined(__SAM3X8E__)
#undef __FlashStringHelper::F(string_literal)
#define F(string_literal) string_literal
#endif
*/

#define LCD_RST A4
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0

#define LTBLUE    0xB6DF
#define LTTEAL    0xBF5F
#define LTGREEN   0xBFF7
#define LTCYAN    0xC7FF
#define LTRED     0xFD34
#define LTMAGENTA 0xFD5F
#define LTYELLOW  0xFFF8
#define LTORANGE  0xFE73
#define LTPINK    0xFDDF
#define LTPURPLE  0xCCFF
#define LTGREY    0xE71C

#define BLUE      0x001F
#define TEAL      0x0438
#define GREEN     0x07E0
#define CYAN      0x07FF
#define RED       0xF800
#define MAGENTA   0xF81F
#define YELLOW    0xFFE0
#define ORANGE    0xFC00
#define PINK      0xF81F
#define PURPLE    0x8010
#define GREY      0xC618
#define WHITE     0xFFFF
#define BLACK     0x0000

//UI Color Palette

#define CLOUDS    0xEF9E
#define TURQUOISE 0x1DF3
#define CARROT    0xE3E4
#define AMETHYST  0x9AD6
#define CONCRETE  0x9534
#define ALIZARIN  0xE267
#define POMEGRANATE 0xC1C5
#define MIDNIGHT  0x29EA
#define QUARTZ    0xF659
#define SERENITY  0x955A

#define MYBROWN   0xA301
#define MYGREEN   0x04E7

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
#define DKGREY    0x4A49

#define MINPRESSURE 10
#define MAXPRESSURE 1000

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RST);

//-------------------------------------------------------------
/*                    VARIABLES                       */
//-------------------------------------------------------------

boolean display1 = true;

double ox , oy ;
double ox2, oy2 ;
double ox3, oy3 ;
double ox4, oy4 ;
double ox5, oy5 ;

double x;
double y, y2, y3, y4, y5;

uint16_t page = 0;

TSPoint p;
/*

// data type for the values used in the reflow profile
typedef struct profileValues_s {
  int16_t soakTemp;
  int16_t soakDuration;
  int16_t peakTemp;
  int16_t peakDuration;
  double  rampUpRate;
  double  rampDownRate;
  uint8_t checksum;
} Profile_t;

*/


//-------------------------------------------------------------
/*                    ENCODER                                */
//-------------------------------------------------------------

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

#define clampValue(val, lo, hi) if (val > hi) val = hi; if (val < lo) val = lo;
#define maxValue(a, b) ((a > b) ? a : b)
#define minValue(a, b) ((a < b) ? a : b)

// ------------
ClickEncoder Encoder(10, 11, 12);

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

bool menuBack(const Menu::Action_t a) {
  if (a == Menu::actionDisplay) {
    engine->navigate(engine->getParent(engine->getParent()));
  }
  return true;
}

//------------------------------

uint8_t menuItemsVisible = 8;
uint8_t menuItemHeight = 24;

void renderMenuItem(const Menu::Item_t *mi, uint8_t pos) {
  //ScopedTimer tm("  render menuitem");

  uint8_t y = pos * menuItemHeight + 2;

  tft.setCursor(10, y);

  // a cursor
  tft.drawRect(8, y - 2, 90, menuItemHeight, (engine->currentItem == mi) ? RED : BLACK);
  tft.print(engine->getLabel(mi));

  // mark items that have children
  if (engine->getChild(mi) != &Menu::NullItem) {
    tft.print(" >   ");
  }
}

// Name, Label, Next, Previous, Parent, Child, Callback
MenuItem(miExit, "MENU", Menu::NullItem, Menu::NullItem, Menu::NullItem, miSettings, menuExit);

MenuItem(miSettings, "Settings", miTest1, Menu::NullItem, miExit, miCalibrateLo, menuDummy);

  MenuItem(miCalibrateLo,  "Calibrate Lo", miCalibrateHi,  Menu::NullItem,       miSettings, Menu::NullItem, menuDummy);
  MenuItem(miCalibrateHi,  "Calibrate Hi", miChannel0, miCalibrateLo,  miSettings, Menu::NullItem, menuDummy);

  MenuItem(miChannel0, "Channel 0", miChannel1, miCalibrateHi, miSettings, miChView0, menuDummy);
    MenuItem(miChView0,  "Ch0:View",  miChScale0,     Menu::NullItem, miChannel0, Menu::NullItem, menuDummy);    
    MenuItem(miChScale0, "Ch0:Scale", Menu::NullItem, miChView0,      miChannel0, Menu::NullItem, menuDummy);    

  MenuItem(miChannel1, "Channel 1", Menu::NullItem, miChannel0, miSettings, miChView1, menuDummy);
    MenuItem(miChView1,  "Ch1:View",  miChScale1,     Menu::NullItem, miChannel1, Menu::NullItem, menuDummy);    
    MenuItem(miChScale1, "Ch1:Scale", miChBack1,      miChView1,      miChannel1, Menu::NullItem, menuDummy); 
    MenuItem(miChBack1,  "Back",      Menu::NullItem, miChScale1,     miChannel1, Menu::NullItem, menuBack);

MenuItem(miTest1, "Test 1 Menu", miTest2,        miSettings, miExit, Menu::NullItem, menuDummy);
MenuItem(miTest2, "Test 2 Menu", miTest3,        miTest1,    miExit, Menu::NullItem, menuDummy);
MenuItem(miTest3, "Test 3 Menu", miTest4,        miTest2,    miExit, Menu::NullItem, menuDummy);
MenuItem(miTest4, "Test 4 Menu", miTest5,        miTest3,    miExit, Menu::NullItem, menuDummy);
MenuItem(miTest5, "Test 5 Menu", miTest6,        miTest4,    miExit, Menu::NullItem, menuDummy);
MenuItem(miTest6, "Test 6 Menu", miTest7,        miTest5,    miExit, Menu::NullItem, menuDummy);
MenuItem(miTest7, "Test 7 Menu", miTest8,        miTest6,    miExit, Menu::NullItem, menuDummy);
MenuItem(miTest8, "Test 8 Menu", Menu::NullItem, miTest7,    miExit, Menu::NullItem, menuDummy);


//-----------------------------------------------------------------------
/*                        LITTLE HELPERS                               */
//-----------------------------------------------------------------------

//----- Wait one touch ---------------------------------------------
TSPoint waitOneTouch() {

  //TSPoint p;

  do {
    p = ts.getPoint();

    pinMode(XM, OUTPUT); //Pins configures again for TFT control
    pinMode(YP, OUTPUT);

  } while ((p.z < MINPRESSURE ) || (p.z > MAXPRESSURE));

  return p;
}

//------- Return Touch Coordinates --------------------------------
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

//-----------------------------------------------------------------------
/*                             SETUP                                   */
//-----------------------------------------------------------------------

void setup() {

  delay(1000);
  //Serial.begin(9600);  
  tft.reset();
  tft.begin(0x9341);
  tft.setRotation(0);
  
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
  page = 0;

  waitOneTouch();
  
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

int16_t encMovement;
int16_t encAbsolute;
int16_t encLastAbsolute = -1;
bool updateMenu = false;

//-----------------------------------------------------------------------
/*                             LOOP                                   */
//-----------------------------------------------------------------------

void loop() {
  
  getTouch();

  // Get button touches on different pages
  if (page == 1){
    if (p.x > 20 && p.x < 200 && p.y > 20 && p.y < 200 )
    {
    profileSelect();
  
    } else if(p.x > 20 && p.x < 120 && p.y > 200 && p.y < 320)
    {
      pid();
  
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
      tft.fillRect(8, 1, 120, 200, BLACK);
    }

    // simple scrollbar
    Menu::Info_t mi = engine->getItemInfo(engine->currentItem);
    uint8_t sbTop = 0, sbWidth = 8, sbLeft = 232;
    uint8_t sbItems = minValue(menuItemsVisible, mi.siblings);
    uint8_t sbHeight = sbItems * menuItemHeight;
    uint8_t sbMarkHeight = sbHeight * sbItems / mi.siblings;
    uint8_t sbMarkTop = ((sbHeight - sbMarkHeight) / mi.siblings) * (mi.position -1);
    tft.fillRect(sbLeft, sbTop,     sbWidth, sbHeight,     WHITE);
    tft.fillRect(sbLeft, sbMarkTop, sbWidth, sbMarkHeight, RED);

    // debug scrollbar values
#if 0
    char buf[30];
    sprintf(buf, "itms: %d, h: %d, mh: %d, mt: %d", sbItems, sbHeight, sbMarkHeight, sbMarkTop);
    Serial.println(buf);
#endif

    // render the menu
    {
      //ScopedTimer tm("render menu");
      engine->render(renderMenuItem, menuItemsVisible);
    }

    {
      ScopedTimer tm("helptext");
      tft.setTextSize(1);
      tft.setCursor(10, 260);
      tft.print("Doubleclick to ");
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
 } //else if page 5. 
 
}// loop end


//-------------------------------------------------------------
/*                    CUSTOM MENU                            */
//-------------------------------------------------------------

void enterMenu(){
  
  tft.fillScreen(BLACK);
  
  // Reflow !
  tft.fillRoundRect( 20, 20, 200, 210, 12, WHITE );
  tft.fillRoundRect( 25, 25, 190, 200, 12, BLACK );
  tft.setCursor (35, 100);
  tft.setTextSize (4);
  tft.setTextColor(WHITE);
  tft.println("Reflow!");
  
  // PID
  tft.fillRect( 20, 250, 70, 50, TURQUOISE );
  tft.setCursor (38, 268);
  tft.setTextSize (2);
  tft.setTextColor(WHITE);
  tft.println("PID");
  
  // Settings
  tft.fillRect( 110, 250, 110, 50, AMETHYST ); 
  tft.setCursor (116, 268);
  tft.setTextSize (2);
  tft.setTextColor(WHITE);
  tft.println("Settings");
  
  page = 1;
  
  waitOneTouch();  
 
 }  

void profileSelect(){
  
  tft.fillScreen(BLACK);

  // Leaded [Pb]
  tft.fillRect( 20, 20, 200, 110, CONCRETE );
  tft.setCursor (90, 60);
  tft.setTextSize (5);
  tft.setTextColor(WHITE);
  tft.println("Pb");
    
  // Lead Free [P\b]
  tft.fillRect( 20, 170, 200, 110, TURQUOISE );
  tft.setCursor (90, 210);
  tft.setTextSize (5);
  tft.setTextColor(WHITE);
  tft.println("Pb");
  tft.drawLine(90,210, 120, 220, WHITE);
  
  page = 2;

  waitOneTouch();  

}
  
void reflow(){
   
  // Here comes the action, we're graphing a curve basd on the interpolated data from MAX31855K
  tft.fillScreen(BLACK);
  
  for (x = 0; x <= 360; x += 0.01) {
  
    //temperature = MAX31855.readThermocouple(CELSIUS);
    //Serial.println(temperature);    
    
    y = 130;
    //         1   2  3  4   5    6    7    8  9   10  11 12   13   14            15         16        17              18      19     20     21     22
    GraphTemp(tft, x, y, 1, 312, 237, 312, 0, 180, 30, 0, 300, 30, "Oven Temp", "Time [s]", "Temp [C]" , DKGREY, GREEN, WHITE, ORANGE, BLACK, display1);
  
    y = y+ 60;
    GraphError(tft, x, y, 1, 312, 237, 312, 0, 180, 30, 0, 300, 30, "Oven Temp", "Time [s]", "Temp C" , DKGREY, GREEN, WHITE, ORANGE, BLACK, display1);

   /* GraphP(tft, x, y3, 0, 310, 238, 300, 0, 180, 50, 0, 300, 50, "Oven Temp", "Time [s]", "Temp C" , DKGREY, ORANGE, WHITE, WHITE, BLACK, display1);
    GraphI(tft, x, y4, 0, 310, 238, 300, 0, 180, 50, 0, 300, 50, "Oven Temp", "Time [s]", "Temp C" , DKGREY, ORANGE, WHITE, WHITE, BLACK, display1);
    GraphD(tft, x, y5, 0, 310, 238, 300, 0, 180, 50, 0, 300, 50, "Oven Temp", "Time [s]", "Temp C" , DKGREY, ORANGE, WHITE, WHITE, BLACK, display1);
   */
  }
  page = 3;
  
  waitOneTouch();
  
  enterMenu();
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

void pid(){
  
  page = 5;
  
  //engine->render(renderMenuItem, menuItemsVisible);
 /* 
  tft.fillScreen(BLACK);
  tft.drawRoundRect(20,20, 200,200, 20, GREEN);
  tft.setCursor (100, 160);
  tft.setTextSize (2);
  tft.setTextColor(YELLOW);
  tft.println("PID");
  
  waitOneTouch();*/
//  enterMenu();
  waitOneTouch();

}


//--------------------------------------------------------------------
/*                  Custom Graphing Functions                       */
//--------------------------------------------------------------------

/*  function to draw a cartesian coordinate system and plot whatever data you want
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
    14  title = title of graph
    15  xlabel = x asis label
    16  ylabel = y asis label
    17  reflowZone = "ramp to soak", "preheat", "soak", "ramp to peak", "peak", "cooling"
    18  gcolor = graph line colors
    19  acolor = axi ine colors
    20  pcolor = color of your plotted data
    21  tcolor = text color
    22  bcolor = background color
    23  &redraw = flag to redraw graph on fist call only
*/
    
//---------------------------------------------------------------------
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
    /*d.setTextSize(1);
      d.setTextColor(tcolor, bcolor);
      d.setCursor(temp, gy + 10);
      // precision is default Arduino--this could really use some format control
      d.println(i);*/
    }

    //---------------------------------
    //Title
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
/*   
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

}/*  End of graphing functioin  */

//---------------------------------------------------------------------
void GraphError(Adafruit_TFTLCD &d, double x, double y, double gx, double gy, double w, double h, double xlo, double xhi, double xinc, double ylo, double yhi, double yinc, String title, String xlabel, String ylabel, unsigned int gcolor, unsigned int acolor, unsigned int pcolor, unsigned int tcolor, unsigned int bcolor, boolean &redraw)
{
  x =  (x - xlo) * ( w) / (xhi - xlo) + gx;
  y =  (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
  d.drawLine(ox2, oy2, x, y, RED);
  d.drawLine(ox2, oy2 + 1, x, y + 1, RED);
  d.drawLine(ox2, oy2 - 1, x, y - 1, RED);
  ox2 = x;
  oy2 = y;

}/*  End of graphing functioin  */

//---------------------------------------------------------------------
void GraphP(Adafruit_TFTLCD &d, double x, double y3, double gx, double gy, double w, double h, double xlo, double xhi, double xinc, double ylo, double yhi, double yinc, String title, String xlabel, String ylabel, unsigned int gcolor, unsigned int acolor, unsigned int pcolor, unsigned int tcolor, unsigned int bcolor, boolean &redraw)
{
  x =  (x - xlo) * ( w) / (xhi - xlo) + gx;
  y3 =  (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
  d.drawLine(ox2, oy2, x, y, ORANGE);
  d.drawLine(ox2, oy2 + 1, x, y3 + 1, ORANGE);
  d.drawLine(ox2, oy2 - 1, x, y3- 1, ORANGE);
  ox3 = x;
  oy3 = y3;

}/*  End of graphing functioin  */

//---------------------------------------------------------------------
void GraphI(Adafruit_TFTLCD &d, double x, double y4, double gx, double gy, double w, double h, double xlo, double xhi, double xinc, double ylo, double yhi, double yinc, String title, String xlabel, String ylabel, unsigned int gcolor, unsigned int acolor, unsigned int pcolor, unsigned int tcolor, unsigned int bcolor, boolean &redraw)
{
  x =  (x - xlo) * ( w) / (xhi - xlo) + gx;
  y4 =  (y4 - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
  d.drawLine(ox2, oy2, x, y, TEAL);
  d.drawLine(ox2, oy2 + 1, x, y4 + 1, TEAL);
  d.drawLine(ox2, oy2 - 1, x, y4 - 1, TEAL);
  ox4 = x;
  oy4 = y4;

}/*  End of graphing functioin  */

//---------------------------------------------------------------------
void GraphD(Adafruit_TFTLCD &d, double x, double y5, double gx, double gy, double w, double h, double xlo, double xhi, double xinc, double ylo, double yhi, double yinc, String title, String xlabel, String ylabel, unsigned int gcolor, unsigned int acolor, unsigned int pcolor, unsigned int tcolor, unsigned int bcolor, boolean &redraw)
{
  x =  (x - xlo) * ( w) / (xhi - xlo) + gx;
  y5 =  (y5 - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
  d.drawLine(ox2, oy2, x, y, CYAN);
  d.drawLine(ox2, oy2 + 1, x, y5 + 1, CYAN);
  d.drawLine(ox2, oy2 - 1, x, y5 - 1, CYAN);
  ox5 = x;
  oy5 = y5;

}/*  End of graphing functioin  */

//-----------------------------------------------------------------------
/*                             Miscelaneous                            */
//-----------------------------------------------------------------------
void drawBorder()
{
  uint16_t width = tft.width() - 1;
  uint16_t height = tft.height() - 1;
  uint8_t border = 10;

  tft.fillScreen(BLACK);
  tft.fillRect(border, border, (width - border * 2), (height - border * 2), MYBROWN);//BLACK);
  tft.drawRect(border, border, (width - border * 2), (height - border * 2), AMETHYST);//WHITE);
}
