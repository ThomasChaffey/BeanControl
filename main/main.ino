

#include <arduino-timer.h>
# define SW 6

unsigned long lastButtonPress = 0;
bool stopwatchOn = false;

auto timer = timer_create_default(); // create a timer with default settings
Timer<>::Task stopwatch;

float t;

bool increment_time(void *) {
  t += 0.5;
  Serial.println(t);
  return true;
}

void setup() {
  t = 0.0; // start stopwatch from 0
  Serial.begin(9600);
  pinMode(SW, INPUT_PULLUP);

}

void loop() {
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
