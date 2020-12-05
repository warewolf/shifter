#include <Joystick.h>
#include <EEPROM.h>

struct Calibration {
  unsigned int x_min;
  unsigned int x_max;
  unsigned int y_min;
  unsigned int y_max;
};

// give it some bullshit values
struct Calibration cal = { 512,512,512, 512} ;

unsigned int x_pos = 0, y_pos = 0;

#define DEBUG 
// #define IN_GEAR_LED

#define X_PIN A0
#define Y_PIN A1

#define INVERT_X false
#define INVERT_Y false

#define EXIT_CALIBRATION 10*1000 // 10 seconds

#define SHIFTER_X_COLS 4 // side to side, almost always 3 to 4
#define SHIFTER_Y_ROWS 5 // up + down, almost always 4 

unsigned int x_ranges[10]; //  = { 0, 256, 512, 768, 1024};
unsigned int y_ranges[10]; //  = { 0, 342, 684, 1024};

unsigned int gear2button[SHIFTER_X_COLS+1][SHIFTER_Y_ROWS+1];

#define GEAR_X_OFFT 4
#define NEUTRAL 254

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,
  SHIFTER_X_COLS*2, 0,                  // Button Count, Hat Switch Count
  #ifdef DEBUG
  true, true, // X and Y
  #else
  false, false, // X and Y
  #endif
  false,     // but no Z Axis
  false, false, false,   // No Rx, Ry, or Rz
  false, false,          // No rudder or throttle
  false, false, false);  // No accelerator, brake, or steering


void setup() {
  int MCUSR_copy = MCUSR;
  MCUSR = 0;
  #ifdef DEBUG
  Serial.begin(115200);
  while (!Serial) { }
    delay(7000);
  #endif

  // fill in the ranges
  for (unsigned int x_i = 0; x_i <= SHIFTER_X_COLS; x_i++) {
    x_ranges[x_i] = x_i * (1024/SHIFTER_X_COLS);
    #ifdef DEBUG
    Serial.print("X range ");
    Serial.print(x_i);
    Serial.print(": ");
    Serial.println(x_i * (1024/SHIFTER_X_COLS));
    #endif
  }

  for (unsigned int y_i = 0; y_i <= SHIFTER_Y_ROWS; y_i++) {
    y_ranges[y_i] = y_i * (1024/SHIFTER_Y_ROWS);
    #ifdef DEBUG
    Serial.print("Y range ");
    Serial.print(y_i);
    Serial.print(": ");
    Serial.println(y_i * (1024/SHIFTER_Y_ROWS));
    #endif
  }

  // create "gear" to "button" map
  byte button = 0;
  for (int x = 1; x <= SHIFTER_X_COLS+1; x++) {
    // fill in "neutral" for all Y positions
    for (int y = 1; y <= SHIFTER_Y_ROWS+1; y++) {
      gear2button[x][y] = NEUTRAL;
    }
    gear2button[x][SHIFTER_Y_ROWS] = button++; // up, 1st, 3rd, 5th gear
    gear2button[x][1] = button++; // down, 2nd, 4th, 6th gear
  }

  analogReference(EXTERNAL); // 5v on 5v boards

  #ifdef DEBUG
    Serial.print("MCUSR: ");
    if(MCUSR_copy & 1<<PORF)  Serial.print("PORF, ");
    if(MCUSR_copy & 1<<EXTRF) Serial.print("EXTRF, ");
    if(MCUSR_copy & 1<<BORF)  Serial.print("BORF, ");
    if(MCUSR_copy & 1<<WDRF)  Serial.print("WDRF, ");
    if(MCUSR_copy & 1<<JTRF)  Serial.print("JTRF, ");
    Serial.println("");
  #endif
  
  pinMode(X_PIN, INPUT);
  pinMode(Y_PIN, INPUT);
  #ifdef IN_GEAR_LED
  pinMode(LED_BUILTIN, OUTPUT);
  #endif

  // Initialize Joystick Library
  Joystick.begin(false);

  // jump into calibration mode if our reset pin was mashed
  if(MCUSR_copy & 1<<EXTRF) {
    calibrate();
  }

  EEPROM.get(0, cal);

  // sane defaults for when eeprom is blank
  if (cal.x_min > 1023 ) {
    cal.x_min = 0; 
    cal.x_max = 1023; 
    cal.y_min = 0; 
    cal.y_max = 1023; 
  }

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
  #ifdef IN_GEAR_LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, blinky);
  #endif

  // timeout after 10 seconds of "no change"
  last_change = looptime;

  // intialize our "last" values to physical reality
  x_last = analogRead(X_PIN);
  y_last = analogRead(Y_PIN);

  // initial "don't do calibration if someone accidentially hits the reset button" phase
  while (looptime - last_change < 5*1000 ) {
    x_val = analogRead(X_PIN);
    y_val = analogRead(Y_PIN);

    #ifdef DEBUG
    Joystick.setXAxis(x_val);
    Joystick.setYAxis(y_val);
    #endif

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
    Joystick.sendState();
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

    #ifdef DEBUG
    Joystick.setXAxis(x_val);
    Joystick.setYAxis(y_val);
    #endif

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
    if ((looptime >> 12) & 0b1 == 1) {
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

    if (((looptime >> 6) & 0b1) == blinky) {
      blinky = !blinky;
      digitalWrite(LED_BUILTIN, blinky);
    }

    #ifdef DEBUG
    Joystick.sendState();
    #endif
    delay(50);
  }

  #ifdef DEBUG
  Serial.println("Exiting calibration");
  #endif
  EEPROM.put(0, cal);

  // signal the end of the calibration period turning off the LED
  #ifdef IN_GEAR_LED
  digitalWrite(LED_BUILTIN, false);
  #endif

}


// what gear we were last in, so we know which button to release
unsigned char previous_gear = NEUTRAL;

void neutral() {
  if (previous_gear != NEUTRAL) {
    Joystick.releaseButton(previous_gear);
  }
  #ifdef IN_GEAR_LED
  digitalWrite(LED_BUILTIN, false);
  #endif
  previous_gear = NEUTRAL;
}

// the loop function runs over and over again forever
void loop() {
  

  // adjust to sensor calibration
  #if INVERT_X == true
  x_pos = map(analogRead(X_PIN), cal.x_min, cal.x_max, 1023, 0);
  #else
  x_pos = map(analogRead(X_PIN), cal.x_min, cal.x_max, 0, 1023);
  #endif

  #if INVERT_Y == true
  y_pos = map(analogRead(Y_PIN), cal.y_min, cal.y_max, 1023, 0);
  #else
  y_pos = map(analogRead(Y_PIN), cal.y_min, cal.y_max, 0, 1023);
  #endif

  // set joystick axises
  #ifdef DEBUG
  Joystick.setXAxis(x_pos);
  Joystick.setYAxis(y_pos);
  #endif

  byte gear = 0;

  // X axis
  for ( byte x = 1; x <= SHIFTER_X_COLS; x++) {
     Serial.print("X range :");
     Serial.println(x_ranges[x]);
    if (x_pos <= x_ranges[x]) {
      gear |= (x<< GEAR_X_OFFT);
      Serial.print("X Coord :");
      Serial.println(x);
      // break;
    }
  }
  
  // y axis
  for ( byte y = 1; y <= SHIFTER_Y_ROWS; y++) {
     Serial.print("Y range :");
     Serial.println(y_ranges[y]);
    if (y_pos <= y_ranges[y]) {
      gear |= y;
      Serial.print("Y is less than :");
      Serial.print(y_ranges[y]);
      Serial.print(", Y Coord :");
      Serial.println(y);
      // break;
    }
  }

  #ifdef DEBUG
  Serial.print("X: ");
  Serial.print(gear >> GEAR_X_OFFT);
  Serial.print(", X pos: ");
  Serial.print(x_pos);
  Serial.print(", Y: ");
  Serial.print(gear & 0xF);
  Serial.print(", Y pos: ");
  Serial.print(y_pos);
  Serial.print(", gear: ");
  Serial.println(gear);
  #endif


/*
 ( X,Y ) coordinates of a 8-gear shifter pattern

 R(1,3)   1(2,3)   3(3,3)   5(4,3)
 |        |        |        |
 |        |        |        |
 xxxxxxxxxxxxxxxxxxxxxxxxxxxx
 |(1,2)   |(2,2)   |(3,2)   |(4,2)
 |        |        |        |
 |        |        |        |
 x(1,1)   2(2,1)   4(3,1)   6(4,1)

 1,1 is the origin!
*/

  // remember, (1,1) is the origin!
  
  byte button = gear2button[gear >> GEAR_X_OFFT][gear & 0xF];

  if (previous_gear != button) {
    // gear changed
    if (button == NEUTRAL) {
      // we're shifting into neutral, release the previous button
      Joystick.releaseButton(previous_gear);
      Joystick.sendState();
    } else {
      // we're shifting directly from a gear into another gear (wtf? okay)
      Joystick.releaseButton(previous_gear);
      Joystick.pressButton(button);
      Joystick.sendState();
    }
    previous_gear = button;
  } else {
    // current button == previous button, do nothing.
  }

  /*
  switch (gear) {

    // X = ..., Y = 2
    case GEAR_1 : // 1,3
      if (previous_gear != 0) {
        neutral();
        previous_gear = 0;
        Joystick.pressButton(0);
  #ifdef IN_GEAR_LED
        digitalWrite(LED_BUILTIN, true);
  #endif
        Joystick.sendState();
      }
      break;

    // X = ..., Y = 0
    case GEAR_2 : // 1,1
      if (previous_gear != 1) {
        neutral();
        previous_gear = 1;
        Joystick.pressButton(1);
  #ifdef IN_GEAR_LED
        digitalWrite(LED_BUILTIN, true);
  #endif
        Joystick.sendState();
      }
      break;

    case GEAR_3 : // 2,3
      if (previous_gear != 2) {
        neutral();
        previous_gear = 2;
        Joystick.pressButton(2);
  #ifdef IN_GEAR_LED
        digitalWrite(LED_BUILTIN, true);
  #endif
        Joystick.sendState();
      }
      break;

    case GEAR_4 : // 2,1
      if (previous_gear != 3) {
        neutral();
        previous_gear = 3;
        Joystick.pressButton(3);
  #ifdef IN_GEAR_LED
        digitalWrite(LED_BUILTIN, true);
  #endif
        Joystick.sendState();
      }
      break;
    
    case GEAR_5: // 3,3
      if (previous_gear != 4) {
        neutral();
        previous_gear = 4;
        Joystick.pressButton(4);
  #ifdef IN_GEAR_LED
        digitalWrite(LED_BUILTIN, true);
  #endif
        Joystick.sendState();
      }
      break;

    case GEAR_6: // 3,1
      if (previous_gear != 5) {
        neutral();
        previous_gear = 5;
        Joystick.pressButton(5);
  #ifdef IN_GEAR_LED
        digitalWrite(LED_BUILTIN, true);
  #endif
        Joystick.sendState();
      }
      break;

    case GEAR_7: // 4,3
      if (previous_gear != 6) {
        neutral();
        previous_gear = 6;
        Joystick.pressButton(6);
  #ifdef IN_GEAR_LED
        digitalWrite(LED_BUILTIN, true);
  #endif
        Joystick.sendState();
      }
      break;

    case GEAR_8 : // 4,1
      if (previous_gear != 7) {
        neutral();
        previous_gear = 7;
        Joystick.pressButton(7);
  #ifdef IN_GEAR_LED
        digitalWrite(LED_BUILTIN, true);
  #endif
        Joystick.sendState();
      }
      break;

    default:
      if (previous_gear != NEUTRAL) {
        neutral();
        Joystick.sendState();
      }
      break;
  }
  */
  
  #ifdef DEBUG
  delay(50);
  #endif
}
