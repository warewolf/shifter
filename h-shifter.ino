// vim: foldmethod=marker commentstring=\ //\ %s
#include <Joystick.h>
#include <EEPROM.h>
#include <math.h>


// struct for save/restore of calibration to/from EEPROM.
struct Calibration { // {{{
  unsigned int point1_min;
  unsigned int point1_max;
  unsigned int point2_min;
  unsigned int point2_max;
  unsigned int point3_min;
  unsigned int point3_max;
}; // }}}

// give it some bullshit values
struct Calibration cal = { // {{{
  512, 512,
  512, 512,
  512, 512
}; // }}}

/*
  Where are your analog sensors connected to?  Define them in sensor_pins
*/
byte sensor_pins[3] = { // {{{
  A0,
  A1,
  A2
}; // }}}

// what slot in our RRD we're reading
volatile bool sensor_slot = 0;
#define NUM_SLOTS 3

// read ADC values when conversion is complete get stuffed in these by the interrupt handler
// sensor_value[sensor][slot] -- two slots, so we can detect a change.
volatile unsigned int sensor_value[3][NUM_SLOTS];

// last sensor we read -- ADC interrupt handler will check and increment this, modulus 3 for the 3 sensors.
volatile byte last_sensor = 0;

// setup sensors
struct Sensor { // {{{
  unsigned int X;
  unsigned int Y;
}; // }}}

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

// #define BUTTONS 4 // 1-based index, for buttons/switches.  These use internal pullups, so wire these to switch to ground.
#define BUTTONS 0

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

// max 10 x 10 grid of gears, normally won't really be that big.
unsigned int x_ranges[10];
unsigned int y_ranges[10];

unsigned int gear2button[SHIFTER_X_COLS+1][SHIFTER_Y_ROWS+1]; // these need to be +1 otherwise breakage

Joystick_ Joystick( // {{{
  JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,
  BUTTONS+(SHIFTER_X_COLS*2), 0, // Button Count, Hat Switch Count
  false, false, false,           // No X, Y or Z asis
  false, false, false,           // No Rx, Ry, or Rz
  false, false,                  // No rudder or throttle
  false, false, false            // No accelerator, brake, or steering
); // }}}

struct Sensor sensor[3];

void trilateration(Sensor result) {
              //(x1,y1,r1,
              // x2,y2,r2,
              // x3,y3,r3) {
  // static
  unsigned long A = (sensor[1].X - sensor[0].X)<<1;
  unsigned long B = (sensor[1].Y - sensor[0].Y)<<1;
  unsigned long D = (sensor[2].X - sensor[1].X)<<1;
  unsigned long E = (sensor[2].Y - sensor[1].Y)<<1;

  // dynamic
  // 0 = r1, 1 = r2, 2 = r3
  unsigned long C = sq(sensor_value[0][sensor_slot]) - sq(sensor_value[1][sensor_slot]) - sq(sensor[0].X) + sq(sensor[1].X) - sq(sensor[0].X) + sq(sensor[1].Y);
  unsigned long F = sq(sensor_value[1][sensor_slot]) - sq(sensor_value[2][sensor_slot]) - sq(sensor[1].X) + sq(sensor[2].X) - sq(sensor[1].Y) + sq(sensor[2].Y);
  result.X = (C*E - F*B) / (E*A - B*D);
  result.Y = (C*D - A*F) / (B*D - A*E);


}

void setup() {
  int MCUSR_copy = MCUSR;
  // clear MCU status register
  MCUSR = 0;

  // disable interrupts
  cli();

  // turn off LED
  digitalWrite(LED_BUILTIN, false);

  // convert to actual PORTx bit mask - this should be portable to other AVR MCUs since it uses digitalPinToBitMask
  for ( byte i = 0; i<3; i++) {
    sensor_pins[i] = digitalPinToBitMask(sensor_pins[i]);
  };

  // initialize our isosceles triangle for our sensor position coordinates, clockwise, starting bottom left.
  //  1
  // 0 2

  sensor[0].X = 0;
  sensor[0].Y = 0;
  sensor[1].X = 511;
  sensor[1].Y = 886;
  sensor[2].X = 1023;
  sensor[2].Y = 0;

  // Initialize the ranges for the analog sensors
  for (byte i = 0; i < SHIFTER_X_COLS; i++) {
    x_ranges[i] = i * ceil(1024/SHIFTER_X_COLS);
  }
  // integer math results in < 1024 for the last one, so just set it to 1024 regardless
  x_ranges[SHIFTER_X_COLS] = 1024;

  for (byte i = 0; i < SHIFTER_Y_ROWS; i++) {
    y_ranges[i] = i * ceil(1024/SHIFTER_Y_ROWS);
  }
  // integer math results in < 1024 for the last one, so just set it to 1024 regardless
  y_ranges[SHIFTER_Y_ROWS] = 1024;

  // switch support, init state
  #if BUTTONS > 0
  for (byte i = 0; i < BUTTONS-1; i++) {
    button_state[i] = false;
  }

  // set buttons to be internal pullups, switches should _ground_ the pin.
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
    gear2button[x][1]              = button++; // down, even numbered gears
  }

  // initialize sensor pins to be input
  for (byte i=0; i==3; i++) {
    pinMode(sensor_pins[i], INPUT);
  }

  // Initialize Joystick Library, but don't start it
  Joystick.begin(false);

  // setup ADC
  freerun_adc();
  sei();

  // jump into calibration mode if our reset pin was mashed
  if(MCUSR_copy & 1<<EXTRF) {
    calibrate();
  }

  // load calibration settings
  EEPROM.get(0, cal);

  // set sane defaults for when eeprom is blank
  if (cal.point1_min > 1023 ) {
    cal.point1_min = 0; 
    cal.point1_max = 1023; 
    cal.point2_min = 0; 
    cal.point3_max = 1023; 
    cal.point3_min = 0; 
    cal.point3_max = 1023; 
  }

  Joystick.sendState();
  #if BUTTONS > 0
  // without this delay the first run through loop() doesn't catch the initial state of buttons
  delay(1500);
  #endif
}

// set ADC to free-running
void freerun_adc() { // {{{

  //ADCSRA |= bit (ADPS0);                               //   2    5 bit, 9us
  //ADCSRA |= bit (ADPS1);                               //   4    6 bit, 9us
  //ADCSRA |= bit (ADPS0) | bit (ADPS1);                 //   8    9 bit, 9.19us
  //ADCSRA |= bit (ADPS2);                               //  16   10 bit, 13us
  //ADCSRA |= bit (ADPS0) | bit (ADPS2);                 //  32
  //ADCSRA |= bit (ADPS1) | bit (ADPS2);                 //  64
  //ADCSRA |= bit (ADPS0) | bit (ADPS1) | bit (ADPS2);   // 128

  // Set analog reference to AVCC, select analog pin, enable left adjust result.
  ADMUX = 0 | bit(REFS0) | sensor_pins[last_sensor] | bit(ADLAR);

  ADCSRB = 0; // ADTS[0-2]=0 - Free Running mode
  // finally enable ADC
  ADCSRA = 0
         | bit(ADEN) // enable ADC
         | bit(ADSC) // start conversion
         | bit(ADATE) | bit(ADIE)   // enable automatic conversion, interrupt on finish.
         | bit(ADPS0) | bit(ADPS1); // set prescaler to 8, 9 bit conversion
} // }}}

ISR(ADC_vect) { // {{{

  // save value, and prepare for next sensor
  sensor_value[last_sensor++][sensor_slot] = ADCH;

  if (last_sensor == 2 ) {
    // flip slots
    sensor_slot++;
    sensor_slot %= NUM_SLOTS;
  }

  // mod last_sensor to switch to the next one
  last_sensor %= 3;

  // Set analog reference to AVCC, select analog pin, enable left adjust result.
  ADMUX = 0 | bit(REFS0) | sensor_pins[last_sensor] | bit(ADLAR);

  // toggle automatic conversion
  // if (0) ADCSRA ^= bit(ADATE);

} // }}}

void calibrate() {

  unsigned long last_change;
  unsigned long looptime = millis();
  bool blinky = true;

  unsigned short point1_val, point2_val, point3_val;
  unsigned short point1_last, point2_last, point3_last;


  // initialize with the actual previous value
  cli();
  point1_last = sensor_value[0][(sensor_slot-1)%2];
  point2_last = sensor_value[1][(sensor_slot-1)%2];
  point3_last = sensor_value[2][(sensor_slot-1)%2];
  sei();


  // turn on LED to signal the start of the calibration period:
  digitalWrite(LED_BUILTIN, true);

  // timeout after 10 seconds of "no change"
  last_change = looptime;

  // initial "don't do calibration if someone accidentially hits the reset button" phase
  while (looptime - last_change < ENTER_CALIBRATION) {
    cli();
    point1_val = sensor_value[0][sensor_slot];
    point2_val = sensor_value[1][sensor_slot];
    point3_val = sensor_value[2][sensor_slot];
    sei();

    // we only care about one axis at a time for last_change timeout
    if (point1_val >> 3 != point1_last >> 3) {
      last_change = looptime;
      point1_last = point1_val;
      break; // exit while loop
    } else if (point2_val >> 3 != point2_last >> 3) {
      last_change = looptime;
      point2_last = point2_val;
      break; // exit while loop
    } else if (point3_val >> 3 != point3_last >> 3) {
      last_change = looptime;
      point3_last = point3_val;
      break; // exit while loop
    }
    looptime = millis();
  }

  if (last_change == looptime) {
  
    // user moved the joystick, blow away calibration
    cal.point1_min = 1024; 
    cal.point1_max = 0; 
    cal.point2_min = 1024; 
    cal.point2_max = 0; 
    cal.point3_min = 1024; 
    cal.point3_max = 0; 
  } else {
    // user didn't move the joystick, bail.
    digitalWrite(LED_BUILTIN, false);
    return;
  }
  
  while (looptime - last_change < TIMEOUT_CALIBRATION) {
    // fetch time and save it for this time around the loop, so we're not repeatedly fetching it
    looptime = millis();
    
    cli();
    point1_val = sensor_value[0][sensor_slot];
    point2_val = sensor_value[1][sensor_slot];
    point3_val = sensor_value[2][sensor_slot];
    sei();

    // we only care about one axis at a time for last_change timeout
    if (point1_val >> 3 != point1_last >> 3) {
      last_change = looptime;
      point1_last = point1_val;
    } else if (point2_val >> 3 != point2_last >> 3) {
      last_change = looptime;
      point2_last = point2_val;
    } else if (point3_val >> 3 != point3_last >> 3) {
      last_change = looptime;
      point3_last = point3_val;
    } 

    if (point1_val > cal.point1_max) {
      cal.point1_max = point1_val;
    } else if (point1_val < cal.point1_min) {
      cal.point1_min = point1_val;
    }

    if (point2_val > cal.point2_max) {
      cal.point2_max = point2_val;
    } else if (point2_val < cal.point2_min) {
      cal.point2_min = point2_val;
    }

    if (point3_val > cal.point3_max) {
      cal.point3_max = point3_val;
    } else if (point3_val < cal.point3_min) {
      cal.point3_min = point3_val;
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
  
  unsigned short point1_real, point2_real, point3_real;
  unsigned short x_pos, y_pos;
  bool change_detected = false;

  // results of trilateration
  struct Sensor trilateration;

  cli();
  point1_real = sensor_value[0][sensor_slot];
  point2_real = sensor_value[1][sensor_slot];
  point3_real = sensor_value[2][sensor_slot];
  sei();

  /*

  Hey! the real world is wobbly.  Sometimes someone can push a sensor further than they did before.  The reason we have..

     point1_real < cal.point1_min ? cal.point1_min : point1_real

  .. is because we need to cap the min and max read from the analog sensor to what our current calibration min and max are, otherwise the Arduino map() function will roll over and cause weird results.

  */


  // cap to calibration bounds if outside bounds
  point1_real = point1_real < cal.point1_min ? cal.point1_min : point1_real;
  point1_real = point1_real > cal.point1_max ? cal.point1_max : point1_real;

  point2_real = point2_real < cal.point2_min ? cal.point2_min : point2_real;
  point2_real = point2_real > cal.point2_max ? cal.point2_max : point2_real;

  point3_real = point3_real < cal.point3_min ? cal.point3_min : point3_real;
  point3_real = point3_real > cal.point3_max ? cal.point3_max : point3_real;

  point1_real = map(point1_real, cal.point1_min, cal.point1_max, 1, 1023);
  point2_real = map(point2_real, cal.point2_min, cal.point2_max, 1, 1023);
  point3_real = map(point3_real, cal.point3_min, cal.point3_max, 1, 1023);

  // do trilateration
  // trilateration(*trilateration,);

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
