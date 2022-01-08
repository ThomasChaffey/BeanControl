
// At the moment, this is a rapid prototype written without any object orientedness.
// Final version will become OO.

#include <arduino-timer.h>
#include <RotaryEncoder.h>

// terminal definitions
#define SW 6
#define DT 5
#define CLK 4

// globals for setpoint
float r = 92.0;
RotaryEncoder encoder(CLK, DT, RotaryEncoder::LatchMode::TWO03);


// globals for the stopwatch
unsigned long lastButtonPress = 0;
bool stopwatchOn = false;
auto timer = timer_create_default(); // create a timer with default settings
Timer<>::Task stopwatch;
float t = 0.0; // stopwatch time

// stopwatch function
bool increment_time(void *) {
  t += 0.5;
  Serial.println(t);
  return true;
}

void setup() {
  Serial.begin(9600);
  pinMode(SW, INPUT_PULLUP);

}

void loop() {

  // setpoint section
  encoder.tick();

  static int pos = 0;
  int newPos = encoder.getPosition();
  if (pos != newPos) {
    r += 0.5 * (float)encoder.getDirection();
    Serial.println(r);
    pos = newPos;
  }

  

  // stopwatch section
  timer.tick(); // tick the timer

   // Read the button state
  int btnState = digitalRead(SW);

  //If we detect LOW signal, button is pressed
  if (btnState == LOW) {
    //if 50ms have passed since last LOW pulse, it means that the
    //button has been pressed, released and pressed again
    if (millis() - lastButtonPress > 50) {
      if (stopwatchOn)
      {
        timer.cancel(stopwatch);
        stopwatchOn = false;
        t = 0.0;
      } else
      {
        // start the stopwatch, increment every half second
        stopwatch = timer.every(500, increment_time);
        stopwatchOn = true;
      }
    }

    // Remember last button press event
    lastButtonPress = millis();
  }

  

}
