
// At the moment, this is a rapid prototype written without any object orientedness.
// Final version will become OO.

#include <arduino-timer.h>
#include <RotaryEncoder.h>
#include <ss_oled.h>


// terminal definitions
#define SW 6
#define DT 5
#define CLK 4
#define PWM 9

// screen init
static uint8_t ucBackBuffer[1024];
#define SDA_PIN 2
#define SCL_PIN 3
// Set this to -1 to disable or the GPIO pin number connected to the reset
// line of your display if it requires an external reset
#define RESET_PIN -1
// let ss_oled figure out the display address
#define OLED_ADDR -1
// don't rotate the display
#define FLIP180 0
// don't invert the display
#define INVERT 0
// Bit-Bang the I2C bus
#define USE_HW_I2C 0

#define MY_OLED OLED_128x64
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

unsigned long lastPrint = 0;
#define PRINT_PERIOD 100

SSOLED ssoled;

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

// globals for temperature
float temp = 21.0;

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
bool printStatus(void *) {
  char tempString[10];
  dtostrf(temp, 4, 2, tempString);
  char tempLine[12];
  snprintf(tempLine, 12, "Temp: %5s", tempString);
  oledWriteString(&ssoled, 0,16,0,tempLine, FONT_NORMAL, 0, 1);
  
  char rString[10];
  dtostrf(r, 4, 2, rString);
  char referenceLine[12];
  snprintf(referenceLine, 12, "Ref:  %5s", rString);
  oledWriteString(&ssoled, 0,16,2,referenceLine, FONT_NORMAL, 0, 1);

    char tString[10];
  dtostrf(t, 4, 2, tString);
  char tLine[12];
  snprintf(tLine, 12, "Time:  %5s", tString);
  oledWriteString(&ssoled, 0,16,4, tLine, FONT_NORMAL, 0, 1);
  
  return true;
}
  

void setup() {
  Serial.begin(9600);
  pinMode(SW, INPUT_PULLUP);
  int rc = oledInit(&ssoled, MY_OLED, OLED_ADDR, FLIP180, INVERT, USE_HW_I2C, SDA_PIN, SCL_PIN, RESET_PIN, 400000L); // use standard I2C bus at 400Khz
  oledFill(&ssoled, 0, 1);
}

void loop() {

  timer.tick(); // tick the timer
  
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

  // heat control
  unsigned long thisPWMCycle = millis();
  if (thisPWMCycle - lastPWMCycle >= PWM_PERIOD) {
    float error = REFERENCE - r;
    int upTime = error > 0 ? floor((REFERENCE - r)*PWM_PERIOD/100) : 0; //time in ms
    digitalWrite(PWM, HIGH);
    if (upTime < PWM_PERIOD) timer.in(upTime, turn_heat_off);
    lastPWMCycle = thisPWMCycle;
  }

  //stopwatch section
   // Read the button state
  int btnState = digitalRead(SW);

  //If we detect LOW signal, button is pressed
  if (btnState == LOW) {
    //if 100 ms have passed since last LOW pulse, it means that the
    //button has been pressed, released and pressed again
    if (millis() - lastButtonPress > 100) {
      if (stopwatchOn)
      {
        stopwatchOn = false;
        timer.cancel(stopwatch);
      } else
      {
        // start the stopwatch, increment every half second
        t = 0.0;
        Serial.println("Button pressed");
        stopwatchOn = true;
        stopwatch = timer.every(500, increment_time);
      }
    }

    // Remember last button press event
    lastButtonPress = millis();
  }

  unsigned long thisPrint = millis();
  if (thisPrint - lastPrint > PRINT_PERIOD){
    printStatus(NULL);
    lastPrint = thisPrint;
  }

}
