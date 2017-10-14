//
// An experimental wrapper class that implements the improved lux and color temperature from 
// TAOS and a basic autorange mechanism.
//
// Written by ductsoup, public domain
//

// RGB Color Sensor with IR filter and White LED - TCS34725
// I2C 7-bit address 0x29, 8-bit address 0x52
//
// http://www.adafruit.com/product/1334
// http://learn.adafruit.com/adafruit-color-sensors/overview
// http://www.adafruit.com/datasheets/TCS34725.pdf
// http://www.ams.com/eng/Products/Light-Sensors/Color-Sensor/TCS34725
// http://www.ams.com/eng/content/view/download/265215 <- DN40, calculations
// http://www.ams.com/eng/content/view/download/181895 <- DN39, some thoughts on autogain
// http://www.ams.com/eng/content/view/download/145158 <- DN25 (original Adafruit calculations)
// some magic numbers for this device from the DN40 application note
#define TCS34725_R_Coef 0.136 
#define TCS34725_G_Coef 1.000
#define TCS34725_B_Coef -0.444
#define TCS34725_GA 1.0
#define TCS34725_DF 310.0
#define TCS34725_CT_Coef 3810.0
#define TCS34725_CT_Offset 1391.0


#include "U8glib.h"
#define backlight 11
//Delcare the display and assign the pins for Nokia LCD
U8GLIB_PCD8544 u8g(3, 4, 9, 5, 6);  // CLK=8, DIN=4, CE=7, DC=5, RST=6

 short x0 = 0;
 short x1 = 30;
 short x2 = 60;

 short y0 = 18; // RGB line
 short y1 = 27; // dashed line
 short y2 = 35; // data line
 short y3 = 48; // final instruction line
   
//display data with 3 strings
char buf[3];

#include <Wire.h>
#include "Adafruit_TCS34725.h"
/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();

/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
uint16_t r, g, b, c, colorTemp, lux;
uint16_t rAvg, gAvg, bAvg;


// button variables
int inPin = 1;   // choose the input pin (for a pushbutton)
int ledPin = 8;  // pin to toggle state and turn on LED for measurement
boolean iTrigger = false;  // event trigger when button is pushed
unsigned long starttime, endtime, loopcount;


// Autorange class for TCS34725
class tcs34725 {
public:
  tcs34725(void);

  boolean begin(void);
  void getData(void);  

  boolean isAvailable, isSaturated;
  uint16_t againx, atime, atime_ms;
  uint16_t r, g, b, c;
  uint16_t ir; 
  uint16_t r_comp, g_comp, b_comp, c_comp;
  uint16_t saturation, saturation75;
  float cratio, cpl, ct, lux, maxlux;
  
private:
  struct tcs_agc {
    tcs34725Gain_t ag;
    tcs34725IntegrationTime_t at;
    uint16_t mincnt;
    uint16_t maxcnt;
  };
  static const tcs_agc agc_lst[];
  uint16_t agc_cur;

  void setGainTime(void);  
  Adafruit_TCS34725 tcs;    
};

//
// Gain/time combinations to use and the min/max limits for hysteresis 
// that avoid saturation. They should be in order from dim to bright. 
//
// Also set the first min count and the last max count to 0 to indicate 
// the start and end of the list. 
//
const tcs34725::tcs_agc tcs34725::agc_lst[] = {
  { TCS34725_GAIN_60X, TCS34725_INTEGRATIONTIME_700MS,     0, 20000 },
  { TCS34725_GAIN_60X, TCS34725_INTEGRATIONTIME_154MS,  4990, 63000 },
  { TCS34725_GAIN_16X, TCS34725_INTEGRATIONTIME_154MS, 16790, 63000 },
  { TCS34725_GAIN_4X,  TCS34725_INTEGRATIONTIME_154MS, 15740, 63000 },
  { TCS34725_GAIN_1X,  TCS34725_INTEGRATIONTIME_154MS, 15740, 0 }
};
tcs34725::tcs34725() : agc_cur(0), isAvailable(0), isSaturated(0) {
}

// initialize the sensor
boolean tcs34725::begin(void) {
  tcs = Adafruit_TCS34725(agc_lst[agc_cur].at, agc_lst[agc_cur].ag);
  if ((isAvailable = tcs.begin())) 
    setGainTime();
  return(isAvailable);
}

// Set the gain and integration time
void tcs34725::setGainTime(void) {
  tcs.setGain(agc_lst[agc_cur].ag);
  tcs.setIntegrationTime(agc_lst[agc_cur].at);
  atime = int(agc_lst[agc_cur].at);
  atime_ms = ((256 - atime) * 2.4);  
  switch(agc_lst[agc_cur].ag) {
  case TCS34725_GAIN_1X: 
    againx = 1; 
    break;
  case TCS34725_GAIN_4X: 
    againx = 4; 
    break;
  case TCS34725_GAIN_16X: 
    againx = 16; 
    break;
  case TCS34725_GAIN_60X: 
    againx = 60; 
    break;
  }        
}

// Retrieve data from the sensor and do the calculations
void tcs34725::getData(void) {
  // read the sensor and autorange if necessary
  tcs.getRawData(&r, &g, &b, &c);
  while(1) {
    if (agc_lst[agc_cur].maxcnt && c > agc_lst[agc_cur].maxcnt) 
      agc_cur++;
    else if (agc_lst[agc_cur].mincnt && c < agc_lst[agc_cur].mincnt)
      agc_cur--;
    else break;

    setGainTime(); 
    delay((256 - atime) * 2.4 * 2); // shock absorber
    tcs.getRawData(&r, &g, &b, &c);
    break;    
  }

  // DN40 calculations
  ir = (r + g + b > c) ? (r + g + b - c) / 2 : 0;
  r_comp = r - ir;
  g_comp = g - ir;
  b_comp = b - ir;
  c_comp = c - ir;   
  cratio = float(ir) / float(c);

  saturation = ((256 - atime) > 63) ? 65535 : 1024 * (256 - atime);
  saturation75 = (atime_ms < 150) ? (saturation - saturation / 4) : saturation;
  isSaturated = (atime_ms < 150 && c > saturation75) ? 1 : 0;
  cpl = (atime_ms * againx) / (TCS34725_GA * TCS34725_DF); 
  maxlux = 65535 / (cpl * 3);

  lux = (TCS34725_R_Coef * float(r_comp) + TCS34725_G_Coef * float(g_comp) + TCS34725_B_Coef * float(b_comp)) / cpl;
  ct = TCS34725_CT_Coef * float(b_comp) / float(r_comp) + TCS34725_CT_Offset;
}

tcs34725 rgb_sensor;

void drawBG() {
//   x0 ----------- x1 ---------------- x2   
  u8g.setFont(u8g_font_04b_03r);  // select font
  u8g.drawStr(12, 8, "press button");  // put string of display at position X, Y
  
  u8g.setFont(u8g_font_profont11);  // select font
  //u8g.drawStr(0, 15, "Nokia 5110..");  // put string of display at position X, Y
  u8g.drawStr(x0, y0, " R ");  // put string of display at position X, Y
  u8g.drawStr(x0, 24, "---");  // put string of display at position X, Y

  u8g.drawStr(x1, y0, " G ");  // put string of display at position X, Y
  u8g.drawStr(x1, 24, "---");  // put string of display at position X, Y

  u8g.drawStr(x2, y0, " B ");  // put string of display at position X, Y
  u8g.drawStr(x2, 24, "---");  // put string of display at position X, Y
    
  u8g.setFont(u8g_font_profont11);  // select font
  // print a line of RGB values 
  dtostrf(r, 3, 0, buf); 
  u8g.drawStr(x0, y2, buf);
  //u8g.drawStr(x0, y2, "255");
  
  dtostrf(g, 3, 0, buf);
  u8g.drawStr(x1, y2, buf);
  //u8g.drawStr(x1, y2, "255");
  
  dtostrf(b, 3, 0, buf);
  u8g.drawStr(x2, y2, buf);
  //u8g.drawStr(x2, y2, "000");
  
  // print final instructions
  u8g.setFont(u8g_font_04b_03r);  // select font
  u8g.drawStr(8, y3, "record 3 numbers");  
}
 
void setup() {
  Serial.begin(9600); // no serial to be found on Pro Trinket via USB
  // Set Backlight Intensity
  // 0 is Fully bright, 255 is off
  // currently not used - but seems like a good idea
  analogWrite(backlight, 100);
  
  pinMode(inPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);      // sets the digital pin as output
  
  rgb_sensor.begin();
//  if (tcs.begin()) {
//    Serial.println("Found sensor");
//  } 
//  else {
//    Serial.println("No TCS34725 found ... check your connections");
//    while (1);
//  }
  
}
 
void loop() { 
 digitalWrite(ledPin, LOW);   // sets the LED off
 u8g.firstPage(); 
  do 
  {
        rAvg = 0;
        drawBG();
        // detect change of state if button is pushed
        if ((digitalRead(inPin) == LOW) & (iTrigger == false)) {
          iTrigger = true;
        }
        
        // if event triggered - turn on LED, take measurement, redraw LCD
        if (iTrigger==true){
          //Serial.println ('event triggered!');
          starttime = millis();
          endtime = starttime;
          
          while ((endtime - starttime) <=100){ // do this loop for up to 1000mS
          
            // turn LED on for measurement
            digitalWrite(ledPin, HIGH);    // 
            
            // do measurement
            rgb_sensor.getData();

            //tcs.getRawData(&r, &g, &b, &c);
            //colorTemp = tcs.calculateColorTemperature(r, g, b);
            //lux = tcs.calculateLux(r, g, b);
            
            // raw values
            //r = rgb_sensor.r;
            //g = rgb_sensor.g;
            //b = rgb_sensor.b;

            // compensated values - we probably want to use these
            r = rgb_sensor.r_comp;
            g = rgb_sensor.g_comp;
            b = rgb_sensor.b_comp;

            
            Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
            Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
            Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
            Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
            Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
            Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
            Serial.println(" ");
            
            //AVG(n) = [(n-1)*Avg(n-1) + X(n)]/n 
                
            endtime = millis();
            loopcount = loopcount + 1;
            rAvg = ((loopcount -1)*rAvg + r  )/loopcount ;
            }
            Serial.print (loopcount,DEC);
            Serial.print("rAvg: "); Serial.print(rAvg, DEC); Serial.println(" ");
            digitalWrite(ledPin, LOW);   // sets the LED off
            iTrigger = false;
            delay(1000); 
        } //end event
        
                  
  } while( u8g.nextPage() ); 
}
  

