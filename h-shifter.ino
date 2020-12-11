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

/*

  Where are your analog sensors connected to?  Define them with X_PIN and Y_PIN.

*/

#define X_PIN A0
#define Y_PIN A1

/*

  Is up and left on the shifter not the lowest joystick button?  Try flipping INVERT_X and/or INVERT_Y to true

*/

#define INVERT_X false
#define INVERT_Y false

/*

  ENTER_CALIBRATION is a timeout in millis for if you accidentially bump the reset button. 
  If ENTER_CALIBRATION millis lapse and no movement is detected on the shifter, it bails out of calibration mode and leaves your calibration settings alone.
  If you wiggle the shifter before the LED goes out, it goes into calibraiton mode.

  TIMEOUT_CALIBRATION is a timeout to _exit_ calibration mode, when the shifter is held still.
*/

#define ENTER_CALIBRATION 5*1000 // 5 seconds
#define TIMEOUT_CALIBRATION 10*1000 // 10 seconds

#define SHIFTER_X_COLS 4 // side to side, almost always 3 to 4 1-based index
#define SHIFTER_Y_ROWS 3 // up, neutral, down, almost always 3  1-based index

#define GEAR_X_OFFT 4 // for packing X+Y into one byte for a "gear".  Don't change this.
#define NEUTRAL 254 // magic button/gear number representing neutral (no buttons pressed)

/*

Switch/button support: Useful for ATS/ETS gear splitters.  Don't want any switches?  Define BUTTONS to 0.

If you have 0 buttons, your first gear button will be the first joystick button.

If you define BUTTONS to 4, you'll have 4 buttons, and your first gear button will be the fifth button.

*/

// #define BUTTONS 0 // 0-based index, for buttons/switches.  These use internal pullups, so wire these to switch to ground.
#define BUTTONS 4 // 1-based index, for buttons/switches.  These use internal pullups, so wire these to switch to ground.

#if BUTTONS > 0
/*

  Set your digital pins here for your switches/buttons.  Remember to separate them by commas.

*/
byte button_pins[BUTTONS] = {
  12, 11, 10, 9
};

byte button_state[BUTTONS];
#endif

// what gear we were last in, so we know which button to release
unsigned char previous_gear = NEUTRAL;

unsigned int x_ranges[10];
unsigned int y_ranges[10];

unsigned int gear2button[SHIFTER_X_COLS+1][SHIFTER_Y_ROWS+1]; // these need to be +1 otherwise breakage


Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,
  BUTTONS+(SHIFTER_X_COLS*2), 0, // Button Count, Hat Switch Count
  false, false, false,           // No X, Y or Z asis
  false, false, false,           // No Rx, Ry, or Rz
  false, false,                  // No rudder or throttle
  false, false, false);          // No accelerator, brake, or steering

void setup() {
  int MCUSR_copy = MCUSR;
  // clear MCU status register
  MCUSR = 0;

  // Initialize the ranges for the analog sensors
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

  // switch support, init state
  #if BUTTONS > 0
  for (byte button = 0; button < BUTTONS-1; button++) {
    button_state[button] = false;
  }

  for(byte i = 0;  i < BUTTONS-1; i++) {
    pinMode(button_pins[i], INPUT_PULLUP);
  }
  #endif

  // create "gear" (packed X+Y coord) to "button" map
  byte button = BUTTONS; // not BUTTONS-1 here because we use postinc for assigning gear buttons
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

  // load calibration settings
  EEPROM.get(0, cal);

  // sane defaults for when eeprom is blank
  if (cal.x_min > 1023 ) {
    cal.x_min = 0; 
    cal.x_max = 1023; 
    cal.y_min = 0; 
    cal.y_max = 1023; 
  }

  Joystick.sendState();
  #if BUTTONS > 0
  // without this delay the first run through loop() doesn't catch the initial state of buttons
  delay(1500);
  #endif
}

void calibrate() {

  unsigned int x_last = 512, y_last = 512;
  unsigned int x_val = 0, y_val = 0;

  unsigned long last_change;
  unsigned long looptime = millis();
  bool blinky = true;

  // turn on LED to signal the start of the calibration period:
  digitalWrite(LED_BUILTIN, true);

  // timeout after 10 seconds of "no change"
  last_change = looptime;

  // intialize our "last" values to physical reality
  x_last = analogRead(X_PIN);
  y_last = analogRead(Y_PIN);

  // initial "don't do calibration if someone accidentially hits the reset button" phase
  while (looptime - last_change < ENTER_CALIBRATION) {
    x_val = analogRead(X_PIN);
    y_val = analogRead(Y_PIN);

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
    looptime = millis();
  }

  if (last_change == looptime) {
  
    // user moved the joystick, blow away calibration
    cal.x_min = 1024; 
    cal.x_max = 0; 
    cal.y_min = 1024; 
    cal.y_max = 0; 
  } else {
    // user didn't move the joystick, bail.
    digitalWrite(LED_BUILTIN, false);
    return;
  }
  
  while (looptime - last_change < TIMEOUT_CALIBRATION) {
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

    // blink the LED to note that we're in calibration mode
    if (((looptime >> 7) & 0b1) == blinky) {
      blinky = !blinky;
      digitalWrite(LED_BUILTIN, blinky);
    }

  }

  EEPROM.put(0, cal);

  // signal the end of the calibration period turning off the LED
  digitalWrite(LED_BUILTIN, false);

}

// the loop function runs over and over again forever
void loop() {
  
  unsigned int x_real = analogRead(X_PIN);
  unsigned int y_real = analogRead(Y_PIN);
  bool change_detected = false;

  /*

  Hey! the real world is wobbly.  Sometimes someone can push a sensor further than they did before.  The reason we have..

     x_real < cal.x_min ? cal.x_min : x_real

  .. is because we need to cap the min and max read from the analog sensor to what our current calibration min and max ar, otherwise the Arduino map() function will roll over and cause weird results.

  */

  // adjust to sensor calibration
  #if INVERT_X == true
  x_pos = map(x_real < cal.x_min ? cal.x_min : x_real, cal.x_min, cal.x_max, 1023, 1);
  #else
  x_pos = map(x_real < cal.x_min ? cal.x_min : x_real, cal.x_min, cal.x_max, 1, 1023);
  #endif

  #if INVERT_Y == true
  y_pos = map(y_real < cal.y_min ? cal.y_min : y_real, cal.y_min, cal.y_max, 1023, 1);
  #else
  y_pos = map(y_real < cal.y_min ? cal.y_min : y_real, cal.y_min, cal.y_max, 1, 1023);
  #endif


  /*
    for position -> coordinate lookups, we skip the 0th index of the ranges array
    because the 0th index is 0, and 0 is less than _everything_.

    This bit here joins together the X and Y coordinate into _one_ value,
    so we can use a lookup table.

  */
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

  byte button = gear2button[gear >> GEAR_X_OFFT][gear & 0xF];

  if (previous_gear != button) {
    // gear changed
    if (button == NEUTRAL) {
      // we're shifting into neutral, release the previous button
      Joystick.releaseButton(previous_gear);
      change_detected = true;
    } else {
      // we're shifting directly from a gear into another gear w/o passing through neutral (wtf? okay)
      Joystick.releaseButton(previous_gear);
      Joystick.pressButton(button);
      change_detected = true;
    }
    previous_gear = button;
  } else {
    // current button == previous button, do nothing.
  }

  /*

  Toggle switch / button support

  */
  #if BUTTONS > 0

  for (byte button = 0; button <= BUTTONS; button++) {
    // switches are internal pull up, so a low is "on"
    bool state = digitalRead(button_pins[button]) == LOW;

    if (button_state[button] != state) {
      if (state == true) {
        Joystick.pressButton(button);
      } else {
        Joystick.releaseButton(button);
      }

      change_detected = true;
      button_state[button] = state;
    } else {
      // no change
    }
  }

  #endif

  if (change_detected == true) {
    Joystick.sendState();
  }
}
