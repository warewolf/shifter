#include <Joystick.h>
#include <EEPROM.h>

byte pin2button[4][2] = { 
  {2,8}, // pin 2 == joy button 0
  {3,9},
  {4,10}, 
  {5,11} 
};

struct Calibration {
  unsigned int x_min;
  unsigned int x_max;
  unsigned int y_min;
  unsigned int y_max;
};

// give it some bullshit values
struct Calibration cal = { 512,512,512, 512} ;


// #define DEBUG 


#define X_PIN A0
#define Y_PIN A1

#define INVERT_X false
#define INVERT_Y false

#define EXIT_CALIBRATION 10*1000 // 10 seconds

#define SHIFTER_COLS 4 // side to side, almost always 3 to 4
#define SHIFTER_ROWS 4 // up + down, almost always 4

#define GEAR_0 0 << 3 | SHIFTER_ROWS-1
#define GEAR_1 0 << 3 | 0
#define GEAR_2 1 << 3 | SHIFTER_ROWS-1
#define GEAR_3 1 << 3 | 0
#define GEAR_4 2 << 3 | SHIFTER_ROWS-1
#define GEAR_5 2 << 3 | 0
#define GEAR_6 3 << 3 | SHIFTER_ROWS-1
#define GEAR_7 3 << 3 | 0


Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,
  SHIFTER_COLS*2, 0,                  // Button Count, Hat Switch Count
  false, false, false,     // X and Y, but no Z Axis
  false, false, false,   // No Rx, Ry, or Rz
  false, false,          // No rudder or throttle
  false, false, false);  // No accelerator, brake, or steering


void setup() {
  int MCUSR_copy = MCUSR;
  MCUSR = 0;
  #ifdef DEBUG
  Serial.begin(115200);
  while (!Serial) { }
  #endif

  analogReference(EXTERNAL); // 5v on 5v boards

  #ifdef DEBUG
    delay(7000);
    Serial.print("MCUSR: ");
    if(MCUSR_copy & 1<<PORF)  Serial.print("PORF, ");
    if(MCUSR_copy & 1<<EXTRF) Serial.print("EXTRF, ");
    if(MCUSR_copy & 1<<BORF)  Serial.print("BORF, ");
    if(MCUSR_copy & 1<<WDRF)  Serial.print("WDRF, ");
    if(MCUSR_copy & 1<<JTRF)  Serial.print("JTRF, ");
    Serial.println("");
  #endif
  
 
  // Initialize Button Pins
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(X_PIN, INPUT);
  pinMode(Y_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  Joystick.begin();

  // jump into calibration mode if our reset pin was mashed
  if(MCUSR_copy & 1<<EXTRF) {
    calibrate();
  }

  EEPROM.get(12, cal);

  #ifdef DEBUG
  Serial.print("CAL X_MIN: ");
  Serial.println(cal.x_min);
  Serial.print("CAL X_MAX: ");
  Serial.println(cal.x_max);
  Serial.print("CAL Y_MIN: ");
  Serial.println(cal.y_min);
  Serial.print("CAL Y_MAX: ");
  Serial.println(cal.y_max);
  #endif

  // Initialize Joystick Library
}

void calibrate() {
  #ifdef DEBUG
  Serial.println("Entering calibration: ");
  #endif

  unsigned int x_last = 512, y_last = 512;
  unsigned int x_val = 0, y_val = 0;

  unsigned long last_change;
  unsigned long looptime = millis();
  bool blinky = true;
  

  // turn on LED to signal the start of the calibration period:
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, blinky);

  // timeout after 10 seconds of "no change"
  last_change = looptime;

  // intialize our "last" values to physical reality
  x_last = analogRead(X_PIN);
  y_last = analogRead(Y_PIN);

  // initial "don't do calibration if someone accidentially hits the reset button" phase
  while (looptime - last_change < 5*1000 ) {
    x_val = analogRead(X_PIN);
    y_val = analogRead(Y_PIN);

    Joystick.setXAxis(x_val);
    Joystick.setYAxis(y_val);
    // we only care about one axis at a time for last_change timeout
    if (x_val >> 3 != x_last >> 3) {
      last_change = looptime;
      x_last = x_val;
      break;
    } else if (y_val >> 3 != y_last >> 3) {
      last_change = looptime;
      y_last = y_val;
      break;
    } 
    delay(50);
    looptime = millis();
  }

  if (last_change == looptime) {
    #ifdef DEBUG
    Serial.println("Movement detected, clearing calibration.");
    #endif
  
    // user moved the joystick, blow away calibration
    cal.x_min = 512; 
    cal.x_max = 512; 
    cal.y_min = 512; 
    cal.y_max = 512; 
  } else {
    // user didn't move the joystick, bail.
    #ifdef DEBUG
    Serial.println("No movement detected, skipping calibration.");
    #endif
    return;
  }

  
  while (looptime - last_change < EXIT_CALIBRATION) {
    // fetch time and save it for this time around the loop, so we're not repeatedly fetching it
    looptime = millis();
    
    x_val = analogRead(X_PIN);
    y_val = analogRead(Y_PIN);

    // we only care about one axis at a time for last_change timeout
    if (x_val >> 3 != x_last >> 3) {
      last_change = looptime;
      x_last = x_val;
    } else if (y_val >> 3 != y_last >> 3) {
      last_change = looptime;
      y_last = y_val;
    } 
        
    if (x_val > cal.x_max) {
      cal.x_max = x_val;
    } else if (x_val < cal.x_min) {
      cal.x_min = x_val;
    }

    if (y_val > cal.y_max) {
      cal.y_max = y_val;
    } else if (y_val < cal.y_min) {
      cal.y_min = y_val;
    }

    #ifdef DEBUG
    if (looptime >> 9 & 0b1 == 1) {
      Serial.print("X_MIN: ");
      Serial.println(cal.x_min);
      Serial.print("X_MAX: ");
      Serial.println(cal.x_max);
      Serial.print("Y_MIN: ");
      Serial.println(cal.y_min);
      Serial.print("Y_MAX: ");
      Serial.println(cal.y_max);
    }
    #endif

    if ((looptime >> 3 & 0b1) == blinky) {
      blinky = !blinky;
      digitalWrite(LED_BUILTIN, blinky);
    }

    delay(50);
  }

  #ifdef DEBUG
  Serial.println("Exiting calibration");
  #endif
  EEPROM.put(12, cal);

  // signal the end of the calibration period turning off the LED
  digitalWrite(LED_BUILTIN, false);

}

unsigned int x_pos = 0, y_pos = 0;

// what gear we were last in, so we know which button to release
unsigned char previous_gear = 0;

void neutral() {
  if (previous_gear != 254)
    Joystick.releaseButton(previous_gear);
  digitalWrite(LED_BUILTIN, false);
  previous_gear = 254;
}

// the loop function runs over and over again forever
void loop() {
  

  // adjust to sensor calibration
  x_pos = map(analogRead(X_PIN), cal.x_min, cal.x_max, 0, 1023);
  y_pos = map(analogRead(Y_PIN), cal.y_min, cal.y_max, 0, 1023);

  // set joystick axises
  // Joystick.setXAxis(x_pos);
  // Joystick.setYAxis(y_pos);

  byte x_coord = 0, y_coord = 0;
  #if INVERT_X == true
  x_coord = map ( x_pos, 1023,0, 0, SHIFTER_COLS-1);
  #else
  x_coord = map ( x_pos, 0,1023, 0, SHIFTER_COLS-1);
  #endif

  #if INVERT_Y == true
  y_coord = map ( y_pos, 1023,0, 0, SHIFTER_ROWS-1);
  #else
  y_coord = map ( y_pos, 0,1023, 0, SHIFTER_ROWS-1);
  #endif

  #ifdef DEBUG
  Serial.print("X: ");
  Serial.print(x_coord);
  Serial.print(", Y: ");
  Serial.println(y_coord);
  #endif
  byte gear = x_coord << 3 | y_coord;

/*
 ( X,Y ) coordinates of a 8-gear shifter pattern

 R(0,2)   1(1,2)   3(2,2)   5(3,2)
 |        |        |        |
 |        |        |        |
 xxxxxxxxxxxxxxxxxxxxxxxxxxxx
 |(0,1)   |(1,1)   |(2,1)   |(3,1)
 |        |        |        |
 |        |        |        |
 x(0,0)   2(1,0)   4(2,0)   6(3,0)

 0,0 is the origin!
*/

  // remember, (0,0) is the origin!


  switch (gear) {

    // X = ..., Y = 2
    case GEAR_0 : // 0,2
      if (previous_gear != 0) {
        neutral();
        previous_gear = 0;
        Joystick.pressButton(0);
        digitalWrite(LED_BUILTIN, true);
      }
      break;

    // X = ..., Y = 0
    case GEAR_1 : // 0,0
      if (previous_gear != 1) {
        neutral();
        previous_gear = 1;
        Joystick.pressButton(1);
        digitalWrite(LED_BUILTIN, true);
      }
      break;

    case GEAR_2 : // 1,2
      if (previous_gear != 2) {
        neutral();
        previous_gear = 2;
        Joystick.pressButton(2);
        digitalWrite(LED_BUILTIN, true);
      }
      break;

    case GEAR_3 : // 1,0
      if (previous_gear != 3) {
        neutral();
        previous_gear = 3;
        Joystick.pressButton(3);
        digitalWrite(LED_BUILTIN, true);
      }
      break;
    
    case GEAR_4: // 2,2
      if (previous_gear != 4) {
        neutral();
        previous_gear = 4;
        Joystick.pressButton(4);
        digitalWrite(LED_BUILTIN, true);
      }
      break;

    case GEAR_5: // 2,0
      if (previous_gear != 5) {
        neutral();
        previous_gear = 5;
        Joystick.pressButton(5);
        digitalWrite(LED_BUILTIN, true);
      }
      break;

    case GEAR_6: // 3,2
      if (previous_gear != 6) {
        neutral();
        previous_gear = 6;
        Joystick.pressButton(6);
        digitalWrite(LED_BUILTIN, true);
      }
      break;

    case GEAR_7 : // 3,0
      if (previous_gear != 7) {
        neutral();
        previous_gear = 7;
        Joystick.pressButton(7);
        digitalWrite(LED_BUILTIN, true);
      }
      break;

    default:
      neutral();
      break;
  }

  
  /*
    for ( int i = 0; i < 4; ++i ) {
    int currentButtonState = !digitalRead(pin2button[i][0]);
    if (currentButtonState != lastButtonState[i])
    {
      Joystick.setButton(pin2button[i][1], currentButtonState);
      lastButtonState[i] = currentButtonState;
    }
  }

  */

  delay(50);
}
