
// At the moment, this is a rapid prototype written without any object orientedness.
// Final version will become OO.

#include <arduino-timer.h>
#include <RotaryEncoder.h>
#include <U8g2lib.h>
#include <Wire.h>

// terminal definitions
#define SW 6
#define DT 5
#define CLK 4
#define PWM 9

// screen init
//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NO_ACK);  // Display which does not send ACK

// globals for setpoint
#define REFERENCE 100
#define PWM_PERIOD 500
float r = 92.0;
RotaryEncoder encoder(CLK, DT, RotaryEncoder::LatchMode::TWO03);

// globals for the stopwatch
unsigned long lastButtonPress = 0;
bool stopwatchOn = false;
auto timer = timer_create_default(); // create a timer with default settings
Timer<>::Task stopwatch;
float t = 0.0; // stopwatch time

// globals for PWM
unsigned long lastPWMCycle = 0;

// stopwatch function
bool increment_time(void *) {
  t += 0.5;
  Serial.println(t);
  return true;
}

// timer function for turning heat off
bool turn_heat_off(void *) {
  digitalWrite(PWM, LOW);
  return true;
}

// status printing function
bool print_status(void *) {
  u8g2.clearBuffer();
  u8g2.setCursor(0, 16);
  u8g2.print(r);
  u8g2.sendBuffer();
  return true;
}
  

void setup() {
  Serial.begin(9600);
  pinMode(SW, INPUT_PULLUP);
  u8g2.begin();
  u8g2.setFont(u8g2_font_logisoso16_tf);
  Wire.setClock(400000); 
}

void loop() {

  // setpoint section
  encoder.tick();

  static int pos = 0;
  int newPos = encoder.getPosition();
  if (pos != newPos) {
    r += 0.5 * (float)encoder.getDirection();
    if(r < 0) r = 0;
    if(r > REFERENCE) r = REFERENCE;
    Serial.println(r);
    pos = newPos;
  }

  // stopwatch section
  timer.tick(); // tick the timer

  // heat control
  unsigned long thisPWMCycle = millis();
  if (thisPWMCycle - lastPWMCycle >= PWM_PERIOD) {
    float error = REFERENCE - r;
    int upTime = error > 0 ? floor((REFERENCE - r)*PWM_PERIOD/100) : 0; //time in ms
    digitalWrite(PWM, HIGH);
    if (upTime < PWM_PERIOD) timer.in(upTime, turn_heat_off);
    lastPWMCycle = thisPWMCycle;
  }

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
      } else
      {
        // start the stopwatch, increment every half second
        t = 0.0;
        stopwatch = timer.every(500, increment_time);
        stopwatchOn = true;
      }
    }

    // Remember last button press event
    lastButtonPress = millis();
  }

    // update screen every 0.2s
    timer.every(1000, print_status);

}
