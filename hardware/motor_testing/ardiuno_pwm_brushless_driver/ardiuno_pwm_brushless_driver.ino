// Monty Pythons pwm brushless motor driver
//
// This is intended to drive a Rev Robotics SparkMAX-family motor controller as well as other motors and motor controllers
// capable of using the PWM inputs for direction and speed control.  This includes the SparkMAX controller as well as the
// Falcon 500 and Kraken X60/X44 motors with their integrated Talon FX controllers as well as standalone Talon FXS controllers.
//
// In this mode, the motor is controlled just like a conventional servo turning the motor CW or CCW based
// on the servo angle set:
//
//      * 0-90 degrees correspond to full reverse to stopped respectively
//      * 90 degrees corresond to stopped
//      * 90-180 degrees correspond to stopped to full forward repectively
//
// (see the motor controller docs for more details)
//
// The PWM is controlled using a potentiometer connected to an analog input.  The pot only sets speed and the direction
// is controlled by a DPDT center-off rocker switch.  Even though the PWM input to the motor controller sets both the
// speed and direction, by breaking these into seperate control inputs on the front panel, we get the full 270 degree
// range of the pot for speed control of the motor regardless of the direction selected.
//
// Each of the two positions (forward and reverse) is connected to a digital input and indicates the direction that
// motor should run with the speed proportional to the pot setting.  The two direction inputs for each motor are pulled high
// so that when no direction is selected (the direction switch is in the center position), both of the direction inputs are
// pulled high using the configured pull up resistors.  When a direction is selected, the corresponding input is pulled low
// indicating the direction the motor should move.
//
// Each motor direction is controlled by the following truth table:
//
//       FWD_* input   REV_* input   Motor Direction
//
//       HJGH          HIGH          No direction selected - motor is stopped
//
//       LOW           HIGH          Forward
//
//       HIGH          LOW           Reverse
//
//       LOW           LOW          This should be an impossible state since the switch won't let both inputs be selected at
//                                  the same time - we do check for this condition in the code and stop the motor if it
//                                  were to ever occur
//
// Two motors can be controlled - the associated variables and constants are identified as "RIGHT" or "LEFT" as appropriate

#include <Servo.h>


// To simpify the wiring, we use a combination of pullups on the digital inputs together with digital outputs set to
// either 0 or 1 to provide 5V and ground levels where required.  We have to be careful to not place too large of
// a load on a digital I/O pins to avoid damaging the arduino.  The values of the pots was chosen to prevent this; the pots
// act as simple voltage dividers between output pins set to both high and low logic levels (5V and 0V respectively)
// and will work fine with relatively high pot values which keep the load on the respective digital I/O pins.
//
// This allows us to keep the pots, the direction switches, and the pwm outputs on different connector groups on the uno
// to make the wiring easier and more obvious to connect.  We're using full width dupont headers for each group so each
// connection block is a different size to help differentiate them and include markings on the connectors and the arduino
// PCB to show the proper orientation.
//
// For the analog inputs, we need to supply a voltage across the pots so we do that using two of the analog inputs configured
// as digital outputs and set them to either 0 or 1 to provide ground and 5V voltages.
//
// The other two inputs we are using are set to analog inputs and connect to the center wipers of the pots for our variable
// inputs and can be read just like any other analog input.

#define POT_RIGHT       A2    // these are the logical pin names
#define POT_LEFT        A3
#define POT_GND         A5
#define POT_5V          A0

// The PWM outputs from the overall rig are wired so that they can be used to drive actual servos.  When using in this manner,
// the direction switches control which 90 degree quadrant the servo moves in with the speed pot controlling the actual servo angle.
// The PWM outputs are spread across two connectors; one which provides an actual GND connection as well as regulated 5V output
// from the arduino with a pair of PWM-capable outputs driving the actual servo or motor controller inputs.
 
#define PWM_OUT_RIGHT   11    // these are all I/O pin numbers
#define PWM_OUT_LEFT    10

// the direction inputs are all distinct - this allows us to detect a switch setting that indicates either forward or
// backward as well as differentiating the "stopped" condition where no direction is selected (see the truth table above)
//
// The FWD and REV direction inputs are set as digital inputs with a pullup enabled; this means when no direction is selected
// the inputs will both return a 1 value.  The switch will only select one direction at a time so depending on the direction selected,
// the corresponding direction input will be pulled down.  If no direction is selected - the center-off position on the
// direction switch - both direction inputs will be high since neither is being pulled down using the direction switch.
//
// A pull down voltage is provided to the direction switches using a digital output set to logical 0 (the DIR_GND pin).

#define REV_RIGHT       3      // these are all I/O pin numbers
#define FWD_RIGHT       4
#define REV_LEFT        5
#define FWD_LEFT        6
#define DIR_GND         7


// The motors respond to a servo-style pwm signal based on the angle.  An "angle" of 90 degrees (midpoint) is a stop indication
// to the motor.  An angle of 0 degrees is full reverese and an angle of 180 degrees is be full forward.
//
// Depending on the reading from the pot, the appropriate angle corresponding to the speed will be set.  For our controller, the
// speed and direction are control are distinct - rocker switches specify the direction and the pots provide the speed so the full range
// of motion for the pots can be used for speed control and the pot values are mapped to "degrees" of 0-90 or 90-180 depending
// on the direction specified.
//
// Even though the motors are not technically servos, we use the servo class to generate the pwm signals
//
// Depending on the specific arduino, some tuning might be in order depending on the crystal quality mostly (I'm assuming)
//
// The values below are tuned to hit 1.5ms (1,500uSec) at the midpoint - our measured adjustment factor is 2.2% to the fast side
// and since the degree are specified as an integer, round to the nearest whole number.  The easiest way to confirm the
// values is to use an oscilloscope and examine the waveforms and measure their duration.  The most important timing
// to hit is the "stopped" or midpoint which should be a pulse of 1.5ms - this will ensure the motor is stopped when
// the tester is powered up and no direction is selected.   Checking the maximum forward and reverse pulse widths is
// also reccomended to ensure the outputs stay in the acceptable range; pulse width for full reverse should not be
// shorter than 1ms and the full forward pulse width should be no longer than 2ms).  Values outside of these ranges
// may cause the motor controller to stop the motor due to out-of-tolerance control inputs.
//
// The values also put a 6 degree floor on the reverse direction - some motors, notably the Kraken X60, seemed to not like
// the "full" reverse value would actually stop the motor.  Others like the SparkMAX seemed to tolerate that better.  All
// motors tested seem to be happy with these values

#define SERVO_FULL_REV    5   // corresponds to 0 degrees
#define SERVO_STOP        92  // mid point - perfect is 90
#define SERVO_FULL_FWD    180 // max servo PWM value - perfect is 180

#define PWM_LEFT          10  // left motor PWM output pin
#define PWM_RIGHT         11  // right motor PWM output pin

#define A2D_MIN           0   //  minimum value we would read from an analog input
#define A2D_MAX           1023 // maximum value we woudl read from an analog input (10 bits or 2^10)

#define A2D_STOP_BAND     5   // a reading below this should be treated as stopped - just in case the pot doesn't fully hit 0


// This code was drived from and simplified from a 4 motor driver variant.  It was just too many controls and wasn't as useful as
// was hoped so the overall design was simplified into the two motor version here.
//
// The table below would allow easy expansion for driving more motors and even in the case of only two motors, it leverages the fact
// that all the operations between the two motors are *exactly* the same and just use different pins for input and output.

typedef struct {
  int   motorSpeedPin;      // analog speed input pin
  int   motorPwmPin;        // PWM output pin
  int   motorDirFwd;        // forward direction pin
  int   motorDirRev;        // reverse direction pin
  Servo servo;              // servo object to be able to set various PWM "angles" for direction and speed
} IO_CONNECTIONS;


IO_CONNECTIONS  IoConns[] = {
  { POT_RIGHT, PWM_RIGHT, FWD_RIGHT, REV_RIGHT },
  { POT_LEFT,  PWM_LEFT,  FWD_LEFT,  REV_LEFT },
};


#define SERIAL_BAUD_RATE    115200 // mostly for debugging output

//#define DEBUG


#ifdef DEBUG
#define PER_LOOP_PAUSE      1000  // slow down if we're debugging to be able to read the output easier
#else
#define PER_LOOP_PAUSE      0     // if needed in case the PWM needs time to stabilize
#endif



// Runs once to get things configured

void setup() {

#ifdef DEBUG        // test output only if DEBUG is set
  // initialize serial output and put out the initial message

  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println("Starting...");
#endif

  // set the fixed I/O pins we need to make this work

  // shared 0 logic level for all direction pins
  pinMode(DIR_GND, OUTPUT);       // set the pin as a digital output
  digitalWrite(DIR_GND, LOW);     // set its logic level to 0 (low)

  pinMode(POT_5V, OUTPUT);        // we need two output pins for the pots - one high and the other low
  digitalWrite(POT_5V, HIGH);     // 5 volts for the speed pots (logic level 1 - aka high)
  pinMode(POT_GND, OUTPUT);       // ground for speed pots
  digitalWrite(POT_GND, LOW);

  // initialize the pins needed for each set of inputs and corresponding outputs for the "right" and "left" motors
  int i;
  
  for (i = 0 ; i < sizeof(IoConns)/sizeof(IoConns[0]) ; i++) {   // loop through all the motor controls we have defined
    pinMode(IoConns[i].motorDirFwd, INPUT_PULLUP);               // set direction pins as inputs with pullups enabled
    pinMode(IoConns[i].motorDirRev, INPUT_PULLUP);
    pinMode(IoConns[i].motorPwmPin, OUTPUT);                     // set the pwm pin as output
    IoConns[i].servo.attach(IoConns[i].motorPwmPin);             // associate a servo controller with the pwm pin
  }
}


// This runs forever - when this function exits, it is run again (arduino convention)

// For each motor defined, we read the direction pins as configured in the table entry and
// determine the direction the motor should move.  If no direction is specified, we stop the motor and go on
// to process the next motor.
//
// If there is a direction specified, we read the speed pot for the current motor and use the arduino map()
// routine to create the appropriate servo "angle" corresponding to the speed and direction we want the motor
// to run at
//
// Once we've processed all the defined motor controllers, the function exits and the arduino environment
// re-runs this function starting everythnig over.

void loop() {
  
  int i, speedRead, fwdRead, revRead, servoVal;

  for (i = 0 ; i < (sizeof(IoConns)/sizeof(IoConns[0])) ; i++) {

    fwdRead = digitalRead(IoConns[i].motorDirFwd);
    revRead = digitalRead(IoConns[i].motorDirRev);

    if ((fwdRead == HIGH && revRead == HIGH) ||     // no direction selected
        (fwdRead == LOW  && revRead == LOW)) {      // invalid state - STOP!    

      servoVal = SERVO_STOP;

    } else {

      speedRead = analogRead(IoConns[i].motorSpeedPin);   // get the speed, 0-1023 range

      if (speedRead < A2D_STOP_BAND) {  // anything below this is considered as "stopped"
        speedRead = A2D_MIN;            // makes sure we can hit stopped even if the pot is a little off
      }

      if (fwdRead == LOW) {                         // map forward
        servoVal = map(speedRead, A2D_MIN, A2D_MAX, SERVO_STOP, SERVO_FULL_FWD);
      } else {                                      // else backward
        servoVal = map(speedRead, A2D_MIN, A2D_MAX, SERVO_STOP, SERVO_FULL_REV);
      }
    }

#ifdef DEBUG        // test output only if DEBUG is set - show before we set
    switch (i) {
      
      case 0:
              Serial.print("R: ");
              break;
              
      case 1:
              Serial.print("L: ");
              break;
    }
    
    Serial.print("Speed value for ");
    Serial.print(IoConns[i].motorSpeedPin);
    Serial.print(" is ");
    Serial.print(speedRead);
    Serial.print(" (R ");
    Serial.print(revRead);
    Serial.print(" / F ");
    Serial.print(fwdRead);
    Serial.println(")");
    
    Serial.print("Servo value for ");
    Serial.print(IoConns[i].motorPwmPin);
    Serial.print(" is ");
    Serial.println(servoVal);   
#endif

    IoConns[i].servo.write(servoVal);   // set the actual speed and direction for the motor
  }

  delay(PER_LOOP_PAUSE);                // give things a chance to settle...
}
