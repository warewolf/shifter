#include <Joystick.h>
#include <EEPROM.h>
#include <math.h>

struct Calibration {
  unsigned int x_min;
  unsigned int x_max;
  unsigned int y_min;
  unsigned int y_max;
};

// give it some bullshit values
struct Calibration cal = { 512,512,512, 512} ;

unsigned int x_pos = 0, y_pos = 0;

// #define DEBUG 
#define IN_GEAR_LED

#define X_PIN A0
#define Y_PIN A1

#define INVERT_X false
#define INVERT_Y false

#define EXIT_CALIBRATION 10*1000 // 10 seconds

#define SHIFTER_X_COLS 4 // side to side, almost always 3 to 4
#define SHIFTER_Y_ROWS 6 // up + down, almost always 4 

unsigned int x_ranges[10]; //  = { 0, 256, 512, 768, 1024};
unsigned int y_ranges[10]; //  = { 0, 342, 684, 1024};

unsigned int gear2button[SHIFTER_X_COLS+1][SHIFTER_Y_ROWS+1];

// overridden map function to prevent it from underflowing
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return ( (x < in_min ? in_min : x) - in_min)
    * (out_max - out_min)
    / (in_max - in_min)
    + out_min;
}

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
  delay(1000);
  Serial.begin(115200);
  while (!Serial) { delay(50); }
    delay(7000);
  #endif

  // fill in the ranges
  for (byte i = 0; i < SHIFTER_X_COLS; i++) {
    x_ranges[i] = i * ceil(1024/SHIFTER_X_COLS);
  }
  // integer math results in < 1024 for the last one, so just add it regardless
  x_ranges[SHIFTER_X_COLS] = 1024;

  for (byte i = 0; i < SHIFTER_Y_ROWS; i++) {
    y_ranges[i] = i * ceil(1024/SHIFTER_Y_ROWS);
  }
  // integer math results in < 1024 for the last one, so just add it regardless
  y_ranges[SHIFTER_Y_ROWS] = 1024;

  // create "gear" (packed X+Y coord) to "button" map
  byte button = 0;
  for (byte x = 1; x <= SHIFTER_X_COLS; x++) {
    // fill in "neutral" for all Y positions
    for (byte y = 1; y <= SHIFTER_Y_ROWS; y++) {
      gear2button[x][y] = NEUTRAL;
    }
    gear2button[x][SHIFTER_Y_ROWS] = button++; // up, odd numbered gears
    gear2button[x][1] = button++; // down, even numbered gears
  }

  analogReference(EXTERNAL); // 5v on 5v boards

  pinMode(X_PIN, INPUT);
  pinMode(Y_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, false);

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
    // delay(50);
    looptime = millis();
  }

  if (last_change == looptime) {
    #ifdef DEBUG
    Serial.println("Movement detected, clearing calibration.");
    #endif
  
    // user moved the joystick, blow away calibration
    cal.x_min = 1024; 
    cal.x_max = 0; 
    cal.y_min = 1024; 
    cal.y_max = 0; 
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

    if (((looptime >> 8) & 0b1) == blinky) {
      blinky = !blinky;
      digitalWrite(LED_BUILTIN, blinky);
    }

    #ifdef DEBUG
    Joystick.sendState();
    #endif
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
  x_pos = map(analogRead(X_PIN), cal.x_min, cal.x_max, 1023, 1);
  #else
  x_pos = map(analogRead(X_PIN), cal.x_min, cal.x_max, 1, 1023);
  #endif

  #if INVERT_Y == true
  y_pos = map(analogRead(Y_PIN), cal.y_min, cal.y_max, 1023, 1);
  #else
  y_pos = map(analogRead(Y_PIN), cal.y_min, cal.y_max, 1, 1023);
  #endif

  // cap within bounds, becase calibration can get fucky
  if (x_pos >= 1023) x_pos = 1023;
  if (x_pos < 1) x_pos = 1;
  if (y_pos >= 1023) y_pos = 1023;
  if (y_pos < 1) y_pos = 1;

  // set joystick axises
  #ifdef DEBUG
  Joystick.setXAxis(x_pos);
  Joystick.setYAxis(y_pos);
  #endif

  byte gear = 0;

  // X axis
  for ( byte i = 1; i <= SHIFTER_X_COLS; i++) {
    if (x_pos <= x_ranges[i]) {
      gear |= (i<< GEAR_X_OFFT);
      break;
    }
  }
  
  // y axis
  for ( byte i = 1; i <= SHIFTER_Y_ROWS; i++) {
    if (y_pos <= y_ranges[i]) {
      gear |= i;
      break;
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

  #ifdef DEBUG
  delay(150);
  #endif
}
