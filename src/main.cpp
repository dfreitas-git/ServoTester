#include <Arduino.h>

// For Nokia OLED display
//#include <Adafruit_GFX.h>
//#include <Adafruit_PCD8544.h>
#include <RotaryEncoder.h>
#include <Servo.h>
#include <U8g2lib.h>
#include <Wire.h>

//################################
// Defines and object declaration
//################################

// Enums
enum servoMode { POT, SWP };  // Either controlled with a potentiometer (POT) or swept (SWP) automatically
enum rotary { LINE, MODE, CURA, MINA, MAXA };  // What the rotary knob is controlling

// Constants
#define OLED_REFRESH_TIME  100   // Time between OLED screen updates
#define SWEEP_REFRESH_TIME 25    // Time between servo sweep steps
boolean PRESSED = false;

// U8G2 for PCD8544 with software SPI
//U8G2_PCD8544_84X48_1_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 12, /* data=*/ 11, /* cs=*/ 7, /* dc=*/ 8, /* reset=*/ 4);
U8X8_PCD8544_84X48_4W_SW_SPI u8x8(/* clock=*/ 12, /* data=*/ 11, /* cs=*/ 7, /* dc=*/ 8, /* reset=*/ 4);

char str[11];  // Array to hold sprintf strings
char modeStr[4];
char selChar[2];


// Rotary Encoder Setup
RotaryEncoder *encoder;         // Create an instance of the rotary encoder object
boolean buttonBeingHeld = false;  // Used to test if rotary button is being held down
#define PIN_IN1 2               // Interrupt pin
#define PIN_IN2 3               // Interrupt pin
#define ENC_SW  A1              // Rotary control switch
rotary rotaryControl;           // Tells us what the rotary know is currently being used for changing
int encoderCurPosition = 0;

// Servos
#define NUM_SERVOS 4
int servoPinArr[NUM_SERVOS] = {10,9,6,5};
int servoCurAngleArr[NUM_SERVOS] = {0,0,0,0};
int servoMinAngleArr[NUM_SERVOS] = {0,0,0,0};
int servoMaxAngleArr[NUM_SERVOS] = {180,180,180,180};
servoMode servoModeArr[NUM_SERVOS] = {POT,POT,POT,POT};  // modes are POT or SWP  (potentiometer or sweep)
Servo servoInstance[NUM_SERVOS]; // Create the servo objects.

// Menu Vars
uint8_t menuLevel = 0;
uint8_t level0SelectRow = 0;
uint8_t level1SelectRow = 0;
uint8_t NUM_LEVEL0_ITEMS = 4;
uint8_t NUM_LEVEL1_ITEMS = 5;
boolean editing = false;   // Set true when editing data in a menu row

// Misc Globals
int sweepAngle = 0;  // Where we are at in the sweep
int sweepDelta = 5;  // Number of degrees we move on each step during a sweep
boolean sweepAngleRising = true;      // CW or CCW rotation
unsigned long lastDisplayUpdate = 0;  // last OLED update
unsigned long lastSweepUpdate = 0;    // last servo step move

// Prototypes
void checkPosition();
boolean checkSwitch(uint8_t pin);
int getEncoderPosition(int count, int limitLow, int limitHigh);
void setEncoderPosition(int count);

// ***************************************************************************
// Setup Section
// ***************************************************************************
void setup() {
   Serial.begin(115200);
   while(!Serial) { }
   delay(1000);

   for(int i = 0; i < NUM_SERVOS; i++) {
      servoInstance[i].attach(servoPinArr[i]);  
   }

   // Set up the oled
   u8x8.begin();
   //u8g2.begin();

   // Initialize the rotary encoder.  Enable the Arduino builtin pullup resistors.
   pinMode(ENC_SW, INPUT_PULLUP);

   // use TWO03 mode when PIN_IN1, PIN_IN2 signals are both LOW or HIGH in latch position.
  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);
  encoder->setPosition(0);

  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);

}

// ***************************************************************************
// Main loop
// ***************************************************************************
void loop() {

   // Check the rotary encoder.  This is our control knob to scroll/select 
   // menu items or change the pot controled servo angle
   encoder->tick(); // just call tick() to check the state.

   encoderCurPosition = encoder->getPosition();

  // Limit the encoder count range depending on which menu we are in (different menus have different number of rows)
  if(menuLevel == 0) {
     encoderCurPosition = getEncoderPosition(encoderCurPosition, 0, NUM_LEVEL0_ITEMS-1);
     level0SelectRow = encoderCurPosition;
  } else if(menuLevel == 1 && rotaryControl == LINE) {
     encoderCurPosition = getEncoderPosition(encoderCurPosition, 0, NUM_LEVEL1_ITEMS-1);
     level1SelectRow = encoderCurPosition;
  } else if(menuLevel == 1 && rotaryControl == MODE) {
     encoderCurPosition = getEncoderPosition(encoderCurPosition, 0, 1);
     if(encoderCurPosition == 0) {
        servoModeArr[level0SelectRow] = POT;
     } else {
        servoModeArr[level0SelectRow] = SWP;
     }
  } else if(menuLevel == 1 && rotaryControl == CURA) {
     encoderCurPosition = getEncoderPosition(encoderCurPosition, servoMinAngleArr[level0SelectRow], servoMaxAngleArr[level0SelectRow]);
     servoCurAngleArr[level0SelectRow] = encoderCurPosition;
  } else if(menuLevel == 1 && rotaryControl == MINA) {
     //encoderCurPosition = getEncoderPosition(encoderCurPosition, servoMinAngleArr[level0SelectRow], servoMaxAngleArr[level0SelectRow]);
     encoderCurPosition = getEncoderPosition(encoderCurPosition, -20, 200 );
     servoMinAngleArr[level0SelectRow] = encoderCurPosition;
  } else if(menuLevel == 1 && rotaryControl == MAXA) {
     //encoderCurPosition = getEncoderPosition(encoderCurPosition, servoMinAngleArr[level0SelectRow], servoMaxAngleArr[level0SelectRow]);
     encoderCurPosition = getEncoderPosition(encoderCurPosition, -20, 200 );
     servoMaxAngleArr[level0SelectRow] = encoderCurPosition;
  }
  setEncoderPosition(encoderCurPosition);

  // ##########################################
  // See if the encoder switch is being pressed
  // ##########################################
  if(checkSwitch(ENC_SW) == PRESSED) {
     // Wait for the switch to be released before going on
     while(checkSwitch(ENC_SW) == PRESSED) {
        delay(1);
     }
     if(menuLevel == 0) {
        level0SelectRow = encoderCurPosition;
        level1SelectRow = 0;
        setEncoderPosition(level1SelectRow);
        menuLevel = 1;
        rotaryControl = LINE;

     } else if(menuLevel == 1 && level1SelectRow == 0 && !editing) {   // This is the "mode" line
        rotaryControl = MODE;
        editing = true;
     } else if(menuLevel == 1 && level1SelectRow == 0 && editing) {   // This is how we exit editing
        rotaryControl = LINE;
        setEncoderPosition(level1SelectRow);
        editing = false;

     } else if(menuLevel == 1 && level1SelectRow == 1 && !editing) {   // This is the "current servo angle" (curA) line
        rotaryControl = CURA;
        setEncoderPosition(servoCurAngleArr[level0SelectRow]);
        editing = true;
     } else if(menuLevel == 1 && level1SelectRow == 1 && editing) {   // This is how we exit editing
        rotaryControl = LINE;
        setEncoderPosition(level1SelectRow);
        editing = false;

     } else if(menuLevel == 1 && level1SelectRow == 2 && !editing) {   // This is the "minimum servo angle" (minA) line
        rotaryControl = MINA;
        setEncoderPosition(servoMinAngleArr[level0SelectRow]);
        editing = true;
     } else if(menuLevel == 1 && level1SelectRow == 2 && editing) {   // This is how we exit editing
        rotaryControl = LINE;
        setEncoderPosition(level1SelectRow);
        editing = false;

     } else if(menuLevel == 1 && level1SelectRow == 3 && !editing) {   // This is the "maximum servo angle" (maxA) line
        rotaryControl = MAXA;
        setEncoderPosition(servoMaxAngleArr[level0SelectRow]);
        editing = true;
     } else if(menuLevel == 1 && level1SelectRow == 3 && editing) {   // This is how we exit editing
        rotaryControl = LINE;
        setEncoderPosition(level1SelectRow);
        editing = false;

     } else if(menuLevel == 1 && level1SelectRow == 4) {   // This is the "back" button
        menuLevel=0;
        setEncoderPosition(level0SelectRow);
     }
  }

   // ####################
   // Update the display
   // ####################
   if(millis() - lastDisplayUpdate > OLED_REFRESH_TIME) {
      lastDisplayUpdate = millis();

      if(menuLevel == 0) {
         u8x8.clearDisplay();
         u8x8.setFont(u8x8_font_pxplustandynewtv_r); 
         u8x8.drawString(0,0, "Sv Mod Ang");  // Servo Mode Angle
         u8x8.drawString(0,1, "----------");
         for(int i = 0; i < NUM_SERVOS; i++) {
            if(servoModeArr[i] == POT) { strcpy(modeStr, "POT"); }
            if(servoModeArr[i] == SWP) { strcpy(modeStr, "SWP"); }
            if(level0SelectRow == i ) {
               strcpy(selChar,"=");
            } else {
               strcpy(selChar," ");
            }
            sprintf(str,"%s%d %s %d", selChar,i, modeStr, servoCurAngleArr[i]);
            u8x8.drawString(0,i+2,str);
         }
      }
      if(menuLevel == 1) {
         // Menu Header
         u8x8.clearDisplay();
         u8x8.setFont(u8x8_font_pxplustandynewtv_r); 
         sprintf(str," Servo %d", level0SelectRow);
         u8x8.drawString(0,0,str);

         // Mode line
         if(servoModeArr[level0SelectRow] == POT) { strcpy(modeStr, "POT"); }
         if(servoModeArr[level0SelectRow] == SWP) { strcpy(modeStr, "SWP"); }
         if(rotaryControl == MODE) {
            u8x8.setFont(u8x8_font_pxplusibmcgathin_r); 
         }
         sprintf(str," Mode: %s", modeStr);
         u8x8.drawString(0,1,str);

         // Current Servo Angle line
         if(rotaryControl == CURA) {
            u8x8.setFont(u8x8_font_pxplusibmcgathin_r); 
         } else {
            u8x8.setFont(u8x8_font_pxplustandynewtv_r); 
         }
         sprintf(str," CurA: %d", servoCurAngleArr[level0SelectRow]);
         u8x8.drawString(0,2,str);

         // Minimum Servo Angle line
         if(rotaryControl == MINA) {
            u8x8.setFont(u8x8_font_pxplusibmcgathin_r); 
         } else {
            u8x8.setFont(u8x8_font_pxplustandynewtv_r); 
         }
         sprintf(str," MinA: %d", servoMinAngleArr[level0SelectRow]);
         u8x8.drawString(0,3,str);

         // Maximum Servo Angle line
         if(rotaryControl == MAXA) {
            u8x8.setFont(u8x8_font_pxplusibmcgathin_r); 
         } else {
            u8x8.setFont(u8x8_font_pxplustandynewtv_r); 
         }
         sprintf(str," MaxA: %d", servoMaxAngleArr[level0SelectRow]);
         u8x8.drawString(0,4,str);

         // Return to level0 menu
         u8x8.setFont(u8x8_font_pxplustandynewtv_r); 
         u8x8.drawString(0,5," Back");

         // Draw the current line indicator (in the left most column)
         u8x8.drawString(0,level1SelectRow+1,"=");
      }
   }

/*
  if(position != newPosition) {
    position = newPosition;
    for(int i = 0; i < NUM_SERVOS; i++) {
       if(servoModeArr[i] == POT) {
          servoInstance[i].write(encoderCurPosition);    // sets the servo position 
          servoCurAngleArr[i] = encoderCurPosition;         // store the position to report later
       }
    }
    delay(20);  // Wait for servo to get there
  } 
*/

/*
   if(millis() - lastSweepUpdate > SWEEP_REFRESH_TIME) {
      lastSweepUpdate = millis();
      if(!sweepAngleRising && sweepAngle <= 0) {
         sweepAngleRising = true;
         sweepAngle = 0;
      }
      if(sweepAngleRising && sweepAngle >= 180) {
         sweepAngleRising = false;
         sweepAngle = 180;
      }
      if(sweepAngleRising && sweepAngle < 180) {
         sweepAngle += sweepDelta;
      }
      if(!sweepAngleRising && sweepAngle > 0) {
         sweepAngle -= sweepDelta;
      }
      for(int i = 0; i < NUM_SERVOS; i++) {
         if(servoModeArr[i] == SWP) {
            servoInstance[i].write(sweepAngle);        // move the servo to the new position
            servoCurAngleArr[i] = sweepAngle;
         }
      }
   }
*/
}

// This interrupt routine will be called on any change of one of the input signals
void checkPosition() {
  encoder->tick(); // just call tick() to check the state.
}

// Set the encoder position count to a new value.
void setEncoderPosition(int count) {
  // Wait until the new encoder count is stored
  while(encoder->getPosition() != count) {
     encoder->setPosition(count);
  }
}

// Return encoder value base on which limits we are using
int getEncoderPosition(int count, int limitLow, int limitHigh) {
   int position;
   if(count < limitLow) {   // roll over to the high side
      position = limitHigh;
   } else if(count > limitHigh) {
      position = limitLow;
   } else {
      position = count;
   }
   return(position);
}

// Check the state of a switch after debouncing it
boolean checkSwitch(uint8_t pin) {
    boolean state;      
    boolean prevState;  
    int debounceDelay = 20;
    prevState = digitalRead(pin);
    for(int counter=0; counter < debounceDelay; counter++) {
        delay(1);       
        state = digitalRead(pin);
        if(state != prevState) {
            counter=0;  
            prevState=state;
        }               
    }                       
    // At this point the switch state is stable
    if(state == HIGH) {     
        return true;    
    } else {            
        return false;   
    }                       
}                       