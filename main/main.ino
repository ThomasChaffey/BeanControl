
// At the moment, this is a rapid prototype written without any object orientedness.
// Final version will become OO.

#include <arduino-timer.h>
#include <RotaryEncoder.h>
#include <ss_oled.h>
#include <Adafruit_MAX31865.h>
#include <EEPROM.h>


// terminal definitions
// rotary encoder
#define SW 6
#define DT 5
#define CLK 4

// pwm output
#define PWM 9

//steam switch
#define STEAM 7

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
#define FLIP180 1
// don't invert the display
#define INVERT 0
// Bit-Bang the I2C bus
#define USE_HW_I2C 0

#define MY_OLED OLED_128x64
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

unsigned long lastPrint = 0;
#define PRINT_PERIOD 100

//EEPROM addresses for r and temp
int rAddress = 0;
int steamAddress = sizeof(float);

SSOLED ssoled;

// control gains
float ki = 0.00003;
float kpBrew = 0.035;
float kpSteam = 0.05; // aggressive control for steaming
float kd = 0.00015;
float* kp = &kpBrew;

// global integrator states
float brewInt = 0.0;
float steamInt = 0.0;
// for derivative calculation
float previousTemp = 20.0;

// globals for setpoint
#define REFMAX 150
#define PWM_PERIOD 500
float r;
float steam;
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
float temp = 0.0;

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  99.87579345

// controller
// output is saturated at 0 and 1 - multiply by PWMPERIOD to give ms of HIGH output
float control(float ref, float temp, float* integrator) {

// if the temp is really low, just saturate the heaters and be done
  if (temp < ref/2) {
    return 1.0;
  }
  
  float e = ref - temp;
  *integrator += e;
  float output = *kp*e + ki*(*integrator) - kd*(temp - previousTemp);
  previousTemp = temp;
  if (output < 0.0) {
    output = 0.0;
  } else if (output > 1.0) {
    output = 1.0;
    *integrator -= e;
  }
  return output;
}
// stopwatch function
bool increment_time(void *) {
  t += 0.5;
//  Serial.println(t);
  return true;
}

// timer function for turning heat off
bool turn_heat_off(void *) {
//  Serial.println("Heat off");
  digitalWrite(PWM, LOW);
  return true;
}

// timer function for updating reference
void updateR() {
  EEPROM.put(rAddress, r);
  EEPROM.put(steamAddress, steam);
}

// status printing function
bool printStatus(float* ref) {
  char tempString[10];
  dtostrf(temp, 4, 2, tempString);
  char tempLine[12];
  snprintf(tempLine, 12, "Temp: %5s", tempString);
  oledWriteString(&ssoled, 0,16,0,tempLine, FONT_NORMAL, 0, 1);
  
  char rString[10];
  dtostrf(*ref, 4, 2, rString);
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
  pinMode(STEAM, INPUT_PULLUP);
  pinMode(PWM, OUTPUT);
  int rc = oledInit(&ssoled, MY_OLED, OLED_ADDR, FLIP180, INVERT, USE_HW_I2C, SDA_PIN, SCL_PIN, RESET_PIN, 400000L); // use standard I2C bus at 400Khz
  oledFill(&ssoled, 0, 1);
  thermo.begin(MAX31865_2WIRE);

  EEPROM.get(rAddress, r);
  // check r is sensible otherwise set default
  if (r > 200.0 || r < 0.0) r = 98.0;
  EEPROM.get(steamAddress, steam);
    // check steam is sensible otherwise set default
  if (steam > 200.0 || steam < 0.0) steam = 98.0;
}

void loop() {

  timer.tick(); // tick the timer

  // pointer to reference temp for this cycle
  float* ref;
  // pointer to switch integrators
  float* integrator;
  // check if steam switch is on
  int steamState = digitalRead(STEAM);
  if (steamState == HIGH) {
    ref = &r;
    integrator = &brewInt;
    kp = &kpBrew;
  } else {
    ref = &steam;
    integrator = &steamInt;
    kp = &kpSteam;
  }
  
  
  // setpoint section
  encoder.tick();  
  static int pos = 0;
  int newPos = encoder.getPosition();
  if (pos != newPos) {
    *ref += 0.5 * (float)encoder.getDirection();
    if(*ref < 0) *ref = 0;
    if(*ref > REFMAX) *ref = REFMAX;
    pos = newPos;
  }

  // heat control
  unsigned long thisPWMCycle = millis();
  if (thisPWMCycle - lastPWMCycle >= PWM_PERIOD) {
      // read the current temperature
    temp = thermo.temperature(RNOMINAL, RREF);
    Serial.println(temp);
    int upTime = floor(control(*ref, temp, integrator)*PWM_PERIOD); //time in ms
    if (upTime !=0 ) {
      digitalWrite(PWM, HIGH);
//      Serial.println("Heat on");
      if (upTime < PWM_PERIOD) timer.in(upTime, turn_heat_off);
    }
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

  // update EEPROM
  updateR();

  unsigned long thisPrint = millis();
  if (thisPrint - lastPrint > PRINT_PERIOD){
    printStatus(ref);
    lastPrint = thisPrint;
  }

}
