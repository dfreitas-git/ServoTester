#include <Arduino.h>

/* ServoTester
  
   Up to four servos can be tested at a time.  All four servos can be driven simultaneously (with the same input range/mode) 
   or each servo can have a separate minimum angle, maximum angle and automatic sweep or manual move mode.
  
   Two level menu is used.  The top level menu shows servo-0/1/2/3 and one called "A" (for all).  Rotating the active line
   indicator (an "=" in the left most column) to the desired servo and clicking the rotary switch puts you into the setting
   mode for that servo.  From there you can set the min-angle, max-angle, mode (POT - for potentiometer, or SWP - for auto-
   sweep), and the current angle.  Select the desired line and click to enter edit mode.  In edit mode you turn the rotary
   knob to increase/decrease the angle or select the different modes.  Select "Back" at the bottom of the menu to return to 
   the top level menu.
  
   Selecting the ServoA line puts you into a mode where the setting apply to all the servos at once (overriding any settings
   that were set for each individual servo).  It allows you to set or sweep all servos with the same settings.
  
   DLF 1/18/2025
*/

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
#define OLED_REFRESH_TIME  60    // Time (ms) between OLED screen updates
boolean PRESSED = false;

// U8G2/U8X8 for PCD8544 with software SPI (for Nokia 5110 display)
U8X8_PCD8544_84X48_4W_SW_SPI u8x8(/* clock=*/ 12, /* data=*/ 11, /* cs=*/ 7, /* dc=*/ 8, /* reset=*/ 4);

// Array to hold sprintf strings
char str[11];  
char modeStr[4];
char selChar[2];


// Rotary Encoder Setup
RotaryEncoder *encoder;         // Create an instance of the rotary encoder object
boolean buttonBeingHeld = false;  // Used to test if rotary button is being held down
#define PIN_IN1 2               // Interrupt pin
#define PIN_IN2 3               // Interrupt pin
#define ENC_SW  A1              // Rotary control switch
rotary rotaryControl;           // Tells us what the rotary knob is currently being used for changing
int encoderCurPosition = 0;

// Servos
#define NUM_SERVOS 5
#define SERVO_SWEEP_SPEED_PIN A2       // So we can control how fast the servos sweep
int servoPinArr[NUM_SERVOS] = {10,9,6,5,-1};  // Fifth entry is just a dummy placeholder for the ALL-SERVOS settings.  We don't assign a real servo to it.
int servoCurAngleArr[NUM_SERVOS] = {90,90,90,90,90};        // Current angle setting for each servo
int servoMinAngleArr[NUM_SERVOS] = {0,0,0,0,-20};           // Minimum angle limit for each servo
int servoMaxAngleArr[NUM_SERVOS] = {180,180,180,180,200};   // Maximum angle limit for each servo
servoMode servoModeArr[NUM_SERVOS] = {POT,POT,POT,POT,POT}; // modes are POT or SWP  (potentiometer or sweep-auto)
Servo servoInstance[NUM_SERVOS-1]; // Create the servo objects for the actual servos 

// Menu Vars
uint8_t menuLevel = 0;
uint8_t level0SelectRow = 0;     // The top level row that is being selected to use
uint8_t level1SelectRow = 0;     // The level-1 menu row that is beign selected
uint8_t NUM_LEVEL0_ITEMS = 5;
uint8_t NUM_LEVEL1_ITEMS = 5;
boolean editing = false;         // Set true when editing data in a menu row

// Misc Globals
int sweepAngle = 0;                   // Where we are at in the sweep
int sweepDelta = 1;                   // Number of degrees we move on each step during a sweep
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

   // Initialize the servos
   for(int i = 0; i < NUM_SERVOS-1; i++) {
      servoInstance[i].attach(servoPinArr[i]);  
   }

   // Set up the oled.  This u8x8 is a text only library (no graphics)
   u8x8.begin();

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

   // Set the speed that the servos will sweep
    // Will be in ms as we'll use the raw 0-1023 reading with millis() later
   uint16_t servoSweepSpeed = analogRead(SERVO_SWEEP_SPEED_PIN); 

   // Limit the encoder count range depending on which menu/mode we are in (different menus have different number of choices)
   // This lets us wrap the menu selector from max back to min when rotating the knob
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
      // Top level menu
      if(menuLevel == 0) {
         level0SelectRow = encoderCurPosition;
         level1SelectRow = 0;
         setEncoderPosition(level1SelectRow);
         menuLevel = 1;
         rotaryControl = LINE;
 
      // Individual servo sub-menu
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
         for(int i = 0; i < NUM_SERVOS; i++) {
            if(servoModeArr[i] == POT) { strcpy(modeStr, "POT"); }
            if(servoModeArr[i] == SWP) { strcpy(modeStr, "SWP"); }

            // Indicate the selected row by placing an "=" in the left column.  That will be 
            // the servo whos setup we enter if we click the rotary switch
            if(level0SelectRow == i ) {
               strcpy(selChar,"=");
            } else {
               strcpy(selChar," ");
            }
            // Name "servo4" as servoA  for ALL.  When we select this servo, 
            // its setup page will apply to all the servos at once.
            if(i == 4) {
               sprintf(str,"%sA %s %d", selChar, modeStr, servoCurAngleArr[i]);
            } else {
               sprintf(str,"%s%d %s %d", selChar,i, modeStr, servoCurAngleArr[i]);
            }
            u8x8.drawString(0,i+1,str);
         }
      }
      // This is the servo setup sub-menu.  Here each entry can be edited to change
      // mode/current-angle,min-angle, and max-angle for the servo.
      // When we click on an entry, the font is changed so we can see we are in edit mode.
      // To exit edit mode, click again.
      if(menuLevel == 1) {
         // Menu Header
         u8x8.clearDisplay();
         u8x8.setFont(u8x8_font_pxplustandynewtv_r); 
         if(level0SelectRow == 4) {
            strcpy(str,"All Servos");
         } else {
            sprintf(str," Servo %d", level0SelectRow);
         }
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

   // Now go move the servo(s)
   for(int i = 0; i < NUM_SERVOS-1; i++) {
      // row 4 means all servos driven with the "servoA" settings
      if(level0SelectRow == 4) {
         if(servoModeArr[4] == POT) {  
            servoInstance[i].write(servoCurAngleArr[4]);     // sets the servo position 
         }
      } else {
         if(servoModeArr[i] == POT) {
            servoInstance[i].write(servoCurAngleArr[i]);    // sets the servo position 
         }
      }
   }

   // For any servos being automatically swept
   if(millis() - lastSweepUpdate > servoSweepSpeed) {
      lastSweepUpdate = millis();
      int index;
      for(int i = 0; i < NUM_SERVOS-1; i++) {
            index = i;
         if(level0SelectRow == 4) {   // row 4 means all servos driven with the "servoA" settings
            index = 4;
         }
         // Calculate the next servo position angle.  Ramp up then back down and repeat.
         if(!sweepAngleRising && sweepAngle <= servoMinAngleArr[index]) {
            sweepAngleRising = true;
            sweepAngle = servoMinAngleArr[index];
         }
         if(sweepAngleRising && sweepAngle >= servoMaxAngleArr[index]) {
            sweepAngleRising = false;
            sweepAngle = servoMaxAngleArr[index];
         }
         if(sweepAngleRising && sweepAngle < servoMaxAngleArr[index]) {
            sweepAngle += sweepDelta;
         }
         if(!sweepAngleRising && sweepAngle > servoMinAngleArr[index]) {
            sweepAngle -= sweepDelta;
         }
         if(level0SelectRow == 4 && servoModeArr[4] == SWP) {   // row 4 means all servos driven with the "servoA" settings
            servoInstance[i].write(sweepAngle);                 // move the indexed servo to the new position
            servoCurAngleArr[4] = sweepAngle;
         } else {
            if(servoModeArr[i] == SWP) {
               servoInstance[i].write(sweepAngle);        // move the indexed servo to the new position
               servoCurAngleArr[i] = sweepAngle;
            }
         }
      }
   }
} 

// ########################################
// Functions
// ########################################

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
