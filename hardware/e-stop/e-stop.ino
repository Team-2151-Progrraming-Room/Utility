// e-stop
//
// provides an external e-stop for the drivers station
//
// the devicer instantiates as a HID keyboard and when the e-stop switch is closed, it sends a ' ' (space) character
// every second until the e-stop opens (character sent is a defined constant)
//
// it also drives a pair of LEDs:
//
// green for NOT e-stopped
//
// red for e-stopped

#include <Keyboard.h>         // Ardiuno "Keyboard" library (originally built with 1.0.6)


// #define DEBUG


#define RED_LED           15      // Red LED pin (high active)
#define GREEN_LED         14      // Green LED pin (high active)

#define ESTOP_SWITCH      5       // E-stop input pin (low active)

#define ESTOP_CHAR        ' '     // sent every interval

#define ESTOP_CHAR_DELAY  1000    // milliseconds between re-sending ESTOP character

#define DEBOUNCE_DELAY    50      // don't re-read the switch faster than this


void setup() {

  pinMode(ESTOP_SWITCH, INPUT_PULLUP);

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  Keyboard.begin();

#ifdef DEBUG
  Serial.begin(115200);

  Serial.println("Starting...");
#endif  // DEBUG
}



long now = 0;         // used to prevent high rate e-stop char sending

void loop() {
  
  if (digitalRead(ESTOP_SWITCH) == LOW) {     // triggered if true

      // don't send e-stop character too fast

#ifdef DEBUG
  Serial.println(now);
#endif  // DEBUG

    if (now == 0) {
      now = millis();

#ifdef DEBUG
      Serial.print("E-stop detected at ");
      Serial.print(now);
      Serial.println("millis");
#endif  // DEBUG

      Keyboard.write(ESTOP_CHAR);     // send the initial e-stop char

    } else {

#ifdef DEBUG
      Serial.print("E-stop active at ");
      Serial.print(millis());
      Serial.println("millis");
      Serial.print("millis() - now = ");
      Serial.println(millis() - now);
#endif  // DEBUG
      if ((millis() - now) > ESTOP_CHAR_DELAY) {

#ifdef DEBUG
      Serial.print("E-stop resending at ");
      Serial.print(millis());
      Serial.println("millis");
#endif  // DEBUG        

        Keyboard.write(ESTOP_CHAR);   // time to send it again - otherwise skip for now

        now = millis();               // reset the timer (ignore the 50 day overflow)
      }
    }

    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, LOW);

  } else {                            // e-stop is not active

#ifdef DEBUG
    // Serial.print("E-stop clear at ");
    // Serial.print(millis());
    // Serial.println("millis");
#endif  // DEBUG
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);

    now = 0;                          // clear the timer
  }

  delay(DEBOUNCE_DELAY);            // let the switch settle down

}
