// vim: foldmethod=marker commentstring=\ //\ %s
#include <Joystick.h>
#include <EEPROM.h>
#include <math.h>


// struct for save/restore of calibration to/from EEPROM.
struct Calibration { // {{{
  byte point1_min;
  byte point1_max;
  byte point2_min;
  byte point2_max;
  byte point3_min;
  byte point3_max;
}; // }}}

// give it some bullshit values
struct Calibration cal = { // {{{
  0, 255,
  0, 255,
  0, 255
}; // }}}

/*
  Where are your analog sensors connected to?  Define them in sensor_pins
*/
byte sensor_pins[] = { // {{{
  // A0, A1, A2
  7, 6, 5
}; // }}}

// what slot in our RRD we're reading
volatile byte sensor_slot = 0;
#define NUM_SLOTS 4

// read ADC values when conversion is complete get stuffed in these by the interrupt handler
// sensor_value[sensor][slot] -- two slots, so we can detect a change.
volatile byte sensor_value[3][NUM_SLOTS];

// last sensor we read -- ADC interrupt handler will check and increment this, modulus 3 for the 3 sensors.
volatile byte next_sensor = 1;


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

#define SHIFTER_X_COLS 3 // side to side, almost always 2 to 3 0-based index
#define SHIFTER_Y_ROWS 2 // up, neutral, down, almost always 2 0-based index

#define GEAR_X_OFFT 4 // for packing X+Y into one byte for a "gear".  Don't change this.
#define NEUTRAL 254 // magic button/gear number representing neutral (no buttons pressed)

/*
Switch/button support: Useful for ATS/ETS gear splitters.  Don't want any switches?  Define BUTTONS to 0.

If you have 0 buttons, your first gear button will be the first joystick button.

If you define BUTTONS to 4, you'll have 4 buttons, and your first gear button will be the fifth button.
*/

#define BUTTONS 4

volatile byte button_state;

// if a change was detected, we need to Joystick.sendState();
volatile bool change_detected = false;
volatile bool measurement_is_garbage = true;

// what gear we were last in, so we know which button to release
unsigned char previous_gear = NEUTRAL;

// max 10 x 10 grid of gears, normally won't really be that big.
unsigned int x_ranges[10];
unsigned int y_ranges[10];

unsigned int gear2button[SHIFTER_X_COLS][SHIFTER_Y_ROWS];

// this gets updated by ISRs, it has to be global and volatile.
volatile Joystick_ Joystick( // {{{
  JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,
  BUTTONS+(SHIFTER_X_COLS*2), 0, // Button Count, Hat Switch Count
  true, true, false,           // No X, Y or Z asis
  false, false, false,           // No Rx, Ry, or Rz
  false, false,                  // No rudder or throttle
  false, false, false            // No accelerator, brake, or steering
); // }}}

struct Sensor sensor[3];

void trilateration(Sensor* result) { // {{{
  // static
  // unsigned long A = (sensor[1].X - sensor[0].X)<<1;
  // unsigned long B = (sensor[1].Y - sensor[0].Y)<<1;
  // unsigned long D = (sensor[2].X - sensor[1].X)<<1;
  // unsigned long E = (sensor[2].Y - sensor[1].Y)<<1;
  signed long A = (511 - 0)<<1;
  signed long B = (886 - 0)<<1;
  signed long D = (1023 - 511)<<1;
  signed long E = (0 - 886)<<1;

  // dynamic
  // 0 = r1, 1 = r2, 2 = r3
  unsigned long C = sq(sensor_value[0][(sensor_slot-2)%NUM_SLOTS]) - sq(sensor_value[1][(sensor_slot-2)%NUM_SLOTS]) - sq(sensor[0].X) + sq(sensor[1].X) - sq(sensor[0].X) + sq(sensor[1].Y);
  unsigned long F = sq(sensor_value[1][(sensor_slot-2)%NUM_SLOTS]) - sq(sensor_value[2][(sensor_slot-2)%NUM_SLOTS]) - sq(sensor[1].X) + sq(sensor[2].X) - sq(sensor[1].Y) + sq(sensor[2].Y);
  result->X = (C*E - F*B) / (E*A - B*D);
  result->Y = (C*D - A*F) / (B*D - A*E);
} // }}}

void setup() { // {{{
  Serial.begin(9600);

  DIDR0  = ((1<<ADC0D)|   // Turn Off Digital Input Buffer.
            (1<<ADC1D)|
            (1<<ADC4D)|
            (1<<ADC5D)|
            (1<<ADC6D)|
            (1<<ADC7D));  

  int MCUSR_copy = MCUSR;
  // clear MCU status register
  MCUSR = 0;

  // disable interrupts
  cli();

  // initialize sensor values to zero for consistency
  // memset(sensor_value,0, sizeof sensor_value);

  // turn off LED
  digitalWrite(LED_BUILTIN, false);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

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

  // PORTB setup
  DDRB   = 0x00; // set PORTB to be input
  PORTB  = 0xFF; // turn on PORTB pull ups for all 8 pins
  //PCMSK0 = 0xFF; // set all 8 pins of PORTB to trigger an interript
  //PCICR |= 1 << PCIE0; // turn on pin change upterrupts

  // create "gear" (packed X+Y coord) to "button" map -- offset by # of toggle buttons
  byte button = BUTTONS; // not BUTTONS-1 here because we use postinc for assigning gear buttons
  for (byte x = 0; x <= SHIFTER_X_COLS; x++) {

    // fill in "neutral" for all Y positions
    for (byte y = 0; y <= SHIFTER_Y_ROWS; y++) {
      gear2button[x][y] = NEUTRAL;
    }

    // fill in up and down gears
    gear2button[x][SHIFTER_Y_ROWS] = button++; // up, odd numbered gears
    gear2button[x][0]              = button++; // down, even numbered gears
  }

  // Initialize Joystick Library, but don't start it
  Joystick.begin(false);

  // setup ADC
  freerun_adc();

  // enable interrupts
  sei();

  // jump into calibration mode if our reset pin was mashed
  if(MCUSR_copy & 1<<EXTRF) {
    // calibrate();
  }

  // load calibration settings
  // EEPROM.get(0, cal);

  // set sane defaults for when eeprom is blank
  if (cal.point1_min > 1023 ) { // {{{
    cal.point1_min = 0; 
    cal.point1_max = 255; 
    cal.point2_min = 0; 
    cal.point3_max = 255; 
    cal.point3_min = 0; 
    cal.point3_max = 255; 
  } // }}}

  Joystick.sendState();

  // initialize button state to physical reality
  button_state = PINB;
} // }}}

// set ADC to free-running
void freerun_adc() { // {{{

  // Set analog reference to AVCC, select analog pin, enable left adjust result.
  // sensor_pins[0] here is intentional, because we init next_sensor to 1.
  ADMUX = 0 | (1<<REFS0) | sensor_pins[0] | (1<<ADLAR);

  ADCSRB = 0; // ADTS[0-2]=0 - Free Running mode
  // finally enable ADC
  ADCSRA = 0
         | (1<<ADEN) // enable ADC
         | (1<<ADATE) | (1<<ADIE)   // enable automatic conversion, interrupt on finish.
         | (1<<ADPS0) | (1<<ADPS1); // set prescaler to 8, 9 bit conversion
  ADCSRA |= (1<<ADSC); // start conversion
} // }}}


/*
  The ADC in free-running mode will perpetually be one measurement off from what we expect in the ISR,
  because by the time we're in the ADC Conversion Complete ISR, it will have already started a measurement.
*/
ISR(ADC_vect) {

  byte current_sensor;

  //ADCSRA |= ~(1<<ADATE);
  if (measurement_is_garbage == false) {
    current_sensor = (next_sensor+3)%3;
    sensor_value[current_sensor][sensor_slot] = ADCH;

    // have we looped back around to the beginning?
    if (next_sensor == 0 ) {
      // next slot please
      sensor_slot++;
      sensor_slot %= NUM_SLOTS;
    }

    // next sensor please
    next_sensor++;
    next_sensor %= 3;

    // next one is garbage
    measurement_is_garbage = true;
  } else {
    current_sensor = ADCH;
    // next one is good
    measurement_is_garbage = false;
  }
  // Set analog reference to AVCC, select analog pin, enable left adjust result.
  ADMUX = 0 | (1<<REFS0) | sensor_pins[next_sensor] | (1<<ADLAR);
  //ADCSRA |= ~(1<<ADATE);
} 

void calibrate() { // {{{

  unsigned long last_change;
  unsigned long looptime = millis();
  bool blinky = true;

  unsigned short point1_val, point2_val, point3_val;
  unsigned short point1_last, point2_last, point3_last;


  // initialize with the actual previous value
  cli();
  point1_last = sensor_value[0][(sensor_slot-2)%NUM_SLOTS];
  point2_last = sensor_value[1][(sensor_slot-2)%NUM_SLOTS];
  point3_last = sensor_value[2][(sensor_slot-2)%NUM_SLOTS];
  sei();


  // turn on LED to signal the start of the calibration period:
  digitalWrite(LED_BUILTIN, true);

  // timeout after 10 seconds of "no change"
  last_change = looptime;

  // initial "don't do calibration if someone accidentially hits the reset button" phase
  while (looptime - last_change < ENTER_CALIBRATION) {
    cli();
    point1_val = sensor_value[0][(sensor_slot-1)%NUM_SLOTS];
    point2_val = sensor_value[1][(sensor_slot-1)%NUM_SLOTS];
    point3_val = sensor_value[2][(sensor_slot-1)%NUM_SLOTS];
    sei();

    // we only care about one axis at a time for last_change timeout
    if (point1_val >> 2 != point1_last >> 2) {
      last_change = looptime;
      point1_last = point1_val;
      break; // exit while loop
    } else if (point2_val >> 2 != point2_last >> 2) {
      last_change = looptime;
      point2_last = point2_val;
      break; // exit while loop
    } else if (point3_val >> 2 != point3_last >> 2) {
      last_change = looptime;
      point3_last = point3_val;
      break; // exit while loop
    }
    looptime = millis();
  }

  if (last_change == looptime) {
  
    // user moved the joystick, blow away calibration
    cal.point1_min = 255; 
    cal.point1_max = 0; 
    cal.point2_min = 255; 
    cal.point2_max = 0; 
    cal.point3_min = 255; 
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
    point1_val = sensor_value[0][(sensor_slot-1)%NUM_SLOTS];
    point2_val = sensor_value[1][(sensor_slot-1)%NUM_SLOTS];
    point3_val = sensor_value[2][(sensor_slot-1)%NUM_SLOTS];
    sei();

    // we only care about one axis at a time for last_change timeout
    if (point1_val >> 2 != point1_last >> 2) {
      last_change = looptime;
      point1_last = point1_val;
    } else if (point2_val >> 2 != point2_last >> 2) {
      last_change = looptime;
      point2_last = point2_val;
    } else if (point3_val >> 2 != point3_last >> 2) {
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

} // }}}

// the loop function runs over and over again forever
void loop() {
  
  unsigned short point1_real, point2_real, point3_real;
  char tx_data[8];
  byte val1, val2, val3;
  byte prev_slot;

  //Serial.println("loop says hi");
  // results of trilateration
  cli();
  prev_slot = sensor_slot;
  prev_slot -= 1;
  prev_slot %= NUM_SLOTS;
  val1=sensor_value[0][prev_slot];
  val2=sensor_value[1][prev_slot];
  val3=sensor_value[2][prev_slot];
  sei();
  sprintf(tx_data,"%03x%02x%02x",val1,val2,val3);
  Serial.print("Sensors: ");
  Serial.println(tx_data);
  struct Sensor result;

#if 0
  // do trilateration
  trilateration(&result);

  byte gear = 0;
  Joystick.setXAxis(result.X);
  Joystick.setYAxis(result.Y);

  // X axis
  for ( byte i = 0; i <= SHIFTER_X_COLS; i++) { // {{{
    if (result.X <= x_ranges[i]) {
      gear |= (i<< GEAR_X_OFFT);
      break;
    }
  } // }}}
  
  // y axis
  for ( byte i = 0; i <= SHIFTER_Y_ROWS; i++) { // {{{
    if (result.Y <= y_ranges[i]) {
      gear |= i;
      break;
    }
  } // }}}

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

  if (change_detected == true) {
    Joystick.sendState();
    change_detected == false;
  }
#endif
}

/*
ISR(PCINT0_vect) { // {{{

  cli();
  // read current button state
  byte new_state = PINB;
  byte difference = 0;

  // old saved button state is button_state
  // new read button state is new_state
  // high = released, "off"
  // low  = pressed,  "on"

  // CHANGE THIS to be a PORTB interrupt handler
  if (button_state != new_state) {
    // something changed
    change_detected = true;

    // figure out what changed, xor old state with new state, only high bits will be the actual changes
    difference = button_state ^ new_state;

    for (byte button = 0; button <= BUTTONS; button++) {
      if (difference & ( 1 << button) == ( 1 << button) ) {
        // *this* button changed
        // what did it change to?
        if (button_state & (1<<button) == 1<<button) {
          // went from low to high
          Joystick.releaseButton(button);
        } else {
          //  went from high to low
          Joystick.pressButton(button);
        }
      }
    }

    // save state
    button_state = new_state;
  }
  sei();
} // }}}
*/
