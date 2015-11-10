//
// Title: BoxFishOven Controller
// Author: Orange Cat
// Date: 2015-11-11
// 
// The BoxFishOven controller supports multiple reflow, annealing, curing and drying profiles
// selectable from the LCD display and a design that makes it easy to add new profiles
// (see BoxFishProfiles to add new profiles.)
//
// The target oven has two SSRs controlling the top and bottom elements (although we
// only turn them on/off together), a relay to control the oven's convection fan, and
// a PWM controlled blower that forces room air into the oven to cool it down rapidly.
//
// Built to run on an Arduino Uno with the Adafruit LCD shield and a MAX31855 thermocouple board, plus
// transistor/mosfet drivers for all the relays.
// 
// Required Libraries:
//   Adafruit RGB LCD Shield Library ( https://github.com/adafruit/Adafruit-RGB-LCD-Shield-Library )
//   Arduino PID Library ( https://github.com/br3ttb/Arduino-PID-Library )
//   MAX31855 Library for reading the thermocouple temperature ( https://github.com/rocketscream/MAX31855 )
//   MenuBackend 1.6 ( https://github.com/Orange-Cat/MenuBackend )
//
// Included libraries:
//   BoxFishUI (a simple menu driven interface to a 2 line LCD display that uses MenuBackend)
//   PIDSeq (a simple PID operations sequencer that uses the Arduino PID Library)
//
// PID Tuning:
//   http://www.cds.caltech.edu/~murray/books/AM08/pdf/am06-pid_16Sep06.pdf
//
// License:
//   This firmware is released under the Creative Commons Attribution-ShareAlike 4.0
//   International license.
//     http://creativecommons.org/licenses/by-sa/4.0/
//

#include <Wire.h>
#include <MAX31855.h>
#include <PID_v1.h>
#include <MenuBackend.h>
#include "PIDSeq.h"
#include "BoxFishUI.h"
#ifdef BOXFISH_USE_ADAFRUIT_LCD  // BOXFISH_USE_ADAFRUIT_LCD is defined (or not) in BoxFishUI.h
  #include <Adafruit_RGBLCDShield.h>
#else
  #include <LiquidCrystal.h>
#endif

// define to simulate temperature rises and falls in the oven, instead of reading the temperature sensor
#undef BOXFISH_OVEN_SIMULATE

const char kBoxFishOvenVersion[] = "1.7";
const char kBoxFishOvenProgramName[] =  "BoxFishOven";

// Pin assignments
#ifdef BOXFISH_USE_ADAFRUIT_LCD
const int relayTopPin = 3;
const int relayBotPin = 4;
const int relayFanPin = 5;
const int blowerPin = 9;
#else
const int relayTopPin = 13;
const int relayBotPin = 13;
const int relayFanPin = 13;
const int blowerPin = 13;
#endif
const int thermocoupleDOPin = 10;
const int thermocoupleCSPin = 11;
const int thermocoupleCLKPin = 12;

// Constants
const int kMaxErrors = 10;               // after this many consecutive errors we quit
const double kTemperatureFanMin = 40.0;  // keep internal fan on until oven cools to this temperture after cycle is complete
const double kTemperatureMax = 280.0;    // abort if temp rises above this
const int kBlowerPWMMax = 127;           // maximum blower PWM (out of 0-255)
const double kBoxFishTemperatureError = -300.0;    // we use this temperature to indicate a temperature read error

// The window size is the number of milliseconds over which we time slice the elements being on/off,
// so if we need half power heating, we will turn the elements on for half of the window size of off for the other half.
// The control variable returned from the PID during heating will be between 0 (off) and kWindowSize (always on).
const unsigned long kWindowSize = 2000;

const unsigned long kUpdateTime = 1000;        // frequency for updating lcd display (in milliseconds)
const unsigned long kSensorSampleTime = 1000;  // frequency of sampling the temperature sensor (in milliseconds)
const unsigned long kPIDSampleTime = 1000;     // frequency of PID calculations (in milliseconds)
unsigned long jobLogSeconds = 1;   // seconds between serial logging events

// PID variables
double setpoint;             // current internal setpoint of PID
double temperature = 20.0;   // current process variable (temperature)
double control;              // current control variable
unsigned long windowStartTime;

double mmThickness = 10.0;   // thickness of plastic in mm (for annealing)

bool isRunning = false;
bool isCooling = false;
bool isError = false;
bool isFanFinished = true;

unsigned long jobLogNumber;     // the number of jobLogSeconds since start of job (normally seconds or minutes since job start)

PIDSeq ovenSeq;   // the PID sequencer
BoxFishUI ui;  // user interface


void setup()
{
  // relay pin initialisation
  digitalWrite(relayTopPin, LOW);
  digitalWrite(relayBotPin, LOW);
  digitalWrite(relayFanPin, LOW);
  pinMode(relayTopPin, OUTPUT);
  pinMode(relayFanPin, OUTPUT);
  pinMode(relayBotPin, OUTPUT);

  // blower pin initialisation
  digitalWrite(blowerPin, LOW);
  pinMode(blowerPin, OUTPUT);
  setBlowerSpeed(0);

  // serial communication at 115200 bps (for logging)
  Serial.begin(115200);

  // configure the user interface:
  // begin() is passed our callback funtion which will be called when user selects some operation in the menus
  ui.begin(kBoxFishOvenProgramName, kBoxFishOvenVersion, menuItemWasSelected);

  // build menus and show root menu:
  buildMenus();
  ui.menuGotoRoot();
}

void loop()
{
  static unsigned long nextUpdate = millis();
  static unsigned long nextRead = millis();

  // time to read thermocouple?
  if (millis() >= nextRead) {
    readThermocouple();
    nextRead += kSensorSampleTime;
  }

  ui.menuNavigate();        // calls our menuItemWasSelected function if user wants to do something
  pidControl();             // where we actually do all the control of the elements, blower and fan

  // time to update status?
  if (millis() >= nextUpdate) {
    updateStatus();
    nextUpdate += kUpdateTime;
  }

  // we abort if the user presses the select button (now emergency stop)
  if (ui.lastButton() == kBoxFishButtonSelect) {
    ovenSeq.abort();
    ui.menuGotoRoot();
  }
}

void pidControl()
{
  // PID sequencing and relay/SSR/blower control
  if (isRunning) {
    // the actual control system call -- we send the process variable (temperature) and it returns the control variable (control)
    control = ovenSeq.control(temperature);

    setpoint = ovenSeq.curSetpoint();
    isRunning = !ovenSeq.isComplete();
    if (!isRunning) {
      // we've just completed the job
      ui.menuGotoRoot();
      ui.beep();
    }
    isCooling = ovenSeq.curOpIsReverse();

    // if we are running fan should be on
    fanEnable(true);
    isFanFinished = false;
          
    if (isCooling) {
      // cooling -- we control blower speed directly
      setBlowerSpeed(control);

      // turn off elements
      elementsEnable(false);
    }
    else {
      // heating: we turn on the element for for control milliseconds within kWindowSize milliseconds
      unsigned long now = millis();

      if ((now - windowStartTime) > kWindowSize) {
        // Time to shift the Relay Window
        windowStartTime += kWindowSize;
      }
      if (control > (now - windowStartTime)) {
        elementsEnable(true);
      }
      else {
        elementsEnable(false);
      }

      // ensure blower is off while in heating cycle
      setBlowerSpeed(0);
    }
  }
  else {
    // when we are not running ensure elements and blower are off
    elementsEnable(false);
    setBlowerSpeed(0);

    // internal fan remains on after profile is complete until the temperature drops below kTempFanMin (and there is no error)
    if (temperature < kTemperatureFanMin) {
      isFanFinished = true;
    }
    fanEnable(!isError && !isFanFinished);
  }
}

void displayTemperature(double temp)
{
  if (temp <= kBoxFishTemperatureError) {
    ui.writeStatus(F("ERROR"));
  }
  else {
    String tempString = String(temp, 0) + kBoxFishDegreeChar + F("C");
    ui.writeStatus(tempString);
  }
}

void updateStatus()
{
  // should be called exaclty once a second to update the LCD and update the serial log as needed
  static unsigned long nextSerialLog = millis();

  // display the control system status
  String status_line = "";
  if (isRunning) {
    if (ovenSeq.curOpName() != NULL) {
      status_line = ovenSeq.curOpName();
    }
    else {
      status_line = ovenSeq.curOpFlashName();
    }
  }
  else {
    if (ovenSeq.isComplete() && ovenSeq.wasStarted()) {
      status_line = F("Complete");
    }
  }
  ui.writeInfo(status_line);
  
  if (isRunning) {
    unsigned long now = millis();
    if (now >= nextSerialLog) {
      // oven is in use and it's time to write to the serial port
      serialLog(status_line.c_str());
      nextSerialLog = now + jobLogSeconds * 1000.0;
    }
  }

  if (isError) {
    // problem reading thermocouple
    displayTemperature(kBoxFishTemperatureError);
  }
  else {
    displayTemperature(temperature);
  }
}

void serialLog(const char current_status[])
{
  // called normally every second or every minute only when job is running to log data to serial port

  // we write a header to the serial port if it's the first time here
  if (jobLogNumber == 0) {
    Serial.println(F("Time,Status,Setpoint,Temperature,Control Percent"));
  }

  jobLogNumber++;

  // determine the percentage that the heater elements or blower is running at (100% == full power)
  double percent_on;
  if (isCooling) {
    percent_on = control / kBlowerPWMMax * 100.0;
  }
  else {
    percent_on = control / kWindowSize * 100.0;
  }

  // now send timestamp, operation name, setpoint, temperature, and percent on to the serial port
  Serial.print(jobLogNumber);
  Serial.print(",");
  Serial.print(current_status);
  Serial.print(",");
  Serial.print(setpoint);
  Serial.print(",");
  Serial.print(temperature);
  Serial.print(",");
  Serial.print(percent_on, 1);
  Serial.println("");
}

#ifdef BOXFISH_OVEN_SIMULATE
void simulateOven()
{
  // simulate oven's temperature based on the control variable (time elements are on, strength of blower)
  
  // average the control variable for heating over time to simulate thermal mass
  const int kNumAverage = 11;
  static double thermal_mass[kNumAverage] ;

  double heating_control;
  if (isCooling) {
    heating_control = 0.0;
  }
  else {
    heating_control = control;
  }

  double sum = 0.0;
  int i;
  for (i=0; i<kNumAverage - 1; i++) {
    sum += thermal_mass[i];
    thermal_mass[i] = thermal_mass[i+1];
  }
  thermal_mass[i] = heating_control;
  sum += thermal_mass[i];
  
  double average_heating_control = sum / kNumAverage;

  // we determine the heat transfer from the elements assuming the surface temp is 1000C.
  // this compensates both for the increase in resistance that occurs as temperature rises
  // and for the loss of efficiency as the air temp gets closer to the element temp.
  double element_heat_transfer = 1.0 - (temperature / 1000.0);

  // we estimate the heating from the elements (note that when we start cooling the elements will still be hot even though off)
  temperature += 2.1 * element_heat_transfer * average_heating_control / kWindowSize;

  // subtract the estimated heat loss at this temperature
  double heat_loss = ((temperature - 20.0) / 330.0);
  temperature -= heat_loss;

  // if we are cooling, we increase heat loss depending on strength of blower:
  if (isCooling) {
    temperature -= (1.85 * control / 255.0) * heat_loss;
  }
}
#endif

void readThermocouple()
{
#ifndef BOXFISH_OVEN_SIMULATE
  static MAX31855 thermocouple(thermocoupleDOPin, thermocoupleCSPin, thermocoupleCLKPin);
  static int errCount = 0;

  // read current temperature
  float temp_temp = thermocouple.readThermocouple(CELSIUS);

  // If thermocouple problem detected
  if ((temp_temp == FAULT_OPEN) || (temp_temp == FAULT_SHORT_GND) || (temp_temp == FAULT_SHORT_VCC)) {
    errCount++;
  }
  else if (temp_temp <  0.001 || temp_temp > kTemperatureMax) {
    // if the i2c amp itself isn't connected, the temperature returned can be 0.0
    // and if the temperture exceeds the maximum it's also treated as an error
    errCount++;
  }
  else {
    // no error, reset error count, we only count consecutive errors
    errCount = 0;
    temperature = temp_temp;
  }

  // too many consecutive errors and we abort
  if (errCount > kMaxErrors) {
    isError = true;
    ovenSeq.abort();
    errCount = 0;
  }
#else
  simulateOven();
#endif
}

void menuDisplayThickness()
{
  String thickness = String(mmThickness, 0) + F("mm");
  ui.overwriteMenu(thickness);
}

void menuSetThickness()
{
  // we allow the user to set the thickness in 5mm increments.
  const double kThicknessIncrement = 5.0;
  const double kMaxThickness = 300.0;
  BoxFishButton button;
  bool selectingThickness = true;

  // just beep if the oven is in operation. we can't set thickness during operation because it's blocking
  // and because it has no effect after the sequence is setup.
  if (isRunning) {
    ui.beep();
    return;
  }

  menuDisplayThickness();
  while (selectingThickness) {
    button = ui.readButton();
    switch (button) {
      case kBoxFishButtonRight:
      case kBoxFishButtonLeft:
      case kBoxFishButtonSelect:
        selectingThickness = false;
        break;
        
      case kBoxFishButtonUp:
        if (mmThickness < kMaxThickness-kThicknessIncrement) {
          mmThickness += kThicknessIncrement;
        }
        break;
        
      case kBoxFishButtonDown:
        if (mmThickness > kThicknessIncrement) {
          mmThickness -= kThicknessIncrement;
        }

      default:
        // do nothing
        break;
    }
    menuDisplayThickness();
  }
  ui.redisplayCurrentMenu();
}

void setSerialLogSeconds(unsigned long seconds)
{
  // set seconds between serial logging events
  jobLogSeconds = seconds;
}

void ovenBegin()
{
  // display a message and reset the job timer
  ui.writeInfo(F("Start"));
  delay(1000);
  jobLogNumber  = 0;
  setSerialLogSeconds(1);  // default 1 second, profile can change this after calling ovenBegin()

  // we setup the oven PID sequencing and our sample time
  ovenSeq.begin();
  ovenSeq.setSampleTime(kPIDSampleTime);
}

void ovenRun()
{
  isRunning = true;
  windowStartTime = millis();  
  isError = false;
  ovenSeq.start(temperature);
}

void reset()
{
  // turn off everthing then reset the contoller
  elementsEnable(false);
  fanEnable(false);
  setBlowerSpeed(0);
  
  ui.softReset();
}

void elementsEnable(bool on)
{
  digitalWrite(relayTopPin, (on)?HIGH: LOW);
  digitalWrite(relayBotPin, (on)?HIGH: LOW);
}

void fanEnable(bool on)
{
  digitalWrite(relayFanPin, (on)?HIGH: LOW);
}

void setBlowerSpeed(int pwm)
{
  if (pwm > kBlowerPWMMax) {
    pwm = kBlowerPWMMax;
  } 
  else if (pwm < 0) {
    pwm = 0;
  }   
  analogWrite(blowerPin, pwm);
}


