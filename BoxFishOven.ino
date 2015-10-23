//
// Title: BoxFishOven Controller
// Author: Orange Cat
// Date: 23-10-2015
// 
// The BoxFishOven controller supports multiple reflow, annealing and curing profiles
// selectable from the LCD display and a design that makes it easy to add new profiles.
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


const char kBoxFishOvenVersion[] = "1.3";
const char kBoxFishOvenProgramName[] =  "BoxFishOven";

enum BoxFishMenuItem {
  // start numbering at 1, because 0 indicates no menu item
  kBoxFishMenuItemLeaded = 1,
  kBoxFishMenuItemLeadFree,
  
  kBoxFishMenuItemSetThickness,
        
  kBoxFishMenuItemAcrylic,
  kBoxFishMenuItemPolycarbonate,
  kBoxFishMenuItemAcetal,

  kBoxFishMenuItemCureEpoxy80,
  
  kBoxFishMenuItemRapidCool,
  kBoxFishMenuItemReset
};

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

const unsigned long kLogTime = 1000;           // frequency for logging data to serial port (in milliseconds)
const unsigned long kSensorSampleTime = 1000;  // frequency of sampling the temperature sensor (in milliseconds)
const unsigned long kPIDSampleTime = 1000;     // frequency of PID calculations (in milliseconds)

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

unsigned long jobSeconds;    // seconds timer since start of job

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
  static unsigned long nextLog = millis();
  static unsigned long nextRead = millis();

  // time to read thermocouple?
  if (millis() > nextRead) {
    readThermocouple();
    nextRead += kSensorSampleTime;
  }

  // time to update status?
  if (millis() > nextLog) {
    updateStatus();
    nextLog += kLogTime;
  }

  ui.menuNavigate();        // calls our menuItemWasSelected function if user wants to do something
  pidControl();             // where we actually do all the control of the elements, blower and fan

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
    ui.displayStatus(F("ERROR"));
  }
  else {
    String tempString = String(temp, 0) + kBoxFishDegreeChar + F("C");
    ui.displayStatus(tempString);
  }
}

void updateStatus()
{
  // must be called exaclty once a second to update the LCD and the log writen to the serial port

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
  ui.displayInfo(status_line);
  
  // if oven is in use
  if (isRunning) {

    // determine the percentage that the heater elements or blower is running at (100% == full power)
    double percent_on;
    if (isCooling) {
      percent_on = control / kBlowerPWMMax * 100.0;
    }
    else {
      percent_on = control / kWindowSize * 100.0;
    }

    // we write a header to the serial port if it's the first time here
    if (jobSeconds == 0) {
      Serial.println(F("Time,Status,Setpoint,Temperature,Control Percent"));
    }

    // while the job is running we increase the seconds count for the job
    jobSeconds++;

    // now send timestamp, operation name, setpoint, temperature and control variable to the serial port
    Serial.print(jobSeconds);
    Serial.print(",");
    Serial.print(status_line);
    Serial.print(",");
    Serial.print(setpoint);
    Serial.print(",");
    Serial.print(temperature);
    Serial.print(",");
    Serial.print(percent_on, 1);
    Serial.println("");
  }

  if (isError) {
    // problem reading thermocouple
    displayTemperature(kBoxFishTemperatureError);
  }
  else {
    displayTemperature(temperature);
  }
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

void buildMenus()
{
  MenuItemRef root = ui.getRootMenu();

  // create our menu system
  static MenuItem mi_reflow = MenuItem(F("[Reflow]"));
  static MenuItem mi_reflow_lead = MenuItem(F("[Leaded]"));
  static MenuItem mi_reflow_lead_use = MenuItem(F("Run Leaded"), kBoxFishMenuItemLeaded);
  static MenuItem mi_reflow_lead_free = MenuItem(F("[Lead Free]"));
  static MenuItem mi_reflow_lead_free_use = MenuItem(F("Run Lead Free"), kBoxFishMenuItemLeadFree);

  static MenuItem mi_anneal = MenuItem(F("[Anneal]"));
  static MenuItem mi_anneal_thickness = MenuItem(F("[Set Thickness]"), kBoxFishMenuItemSetThickness);
  static MenuItem mi_anneal_acrylic = MenuItem(F("[Acrylic]"));
  static MenuItem mi_anneal_acrylic_use = MenuItem(F("Run Acrylic"), kBoxFishMenuItemAcrylic);
  static MenuItem mi_anneal_polycarbonate = MenuItem(F("[Polycarbonate]"));
  static MenuItem mi_anneal_polycarbonate_use = MenuItem(F("Run Polycarbonate"), kBoxFishMenuItemPolycarbonate);
  static MenuItem mi_anneal_acetal = MenuItem(F("[Acetal]"));
  static MenuItem mi_anneal_acetal_use = MenuItem(F("Run Acetal"), kBoxFishMenuItemAcetal);

  static MenuItem mi_cure = MenuItem(F("[Cure]"));
  static MenuItem mi_cure_epoxy80 = MenuItem(F("[Epoxy Staged 80]"));
  static MenuItem mi_cure_epoxy80_use = MenuItem(F("Run Epoxy Staged 80"), kBoxFishMenuItemCureEpoxy80);
  
  static MenuItem mi_rapid = MenuItem(F("[Rapid Cool]"));
  static MenuItem mi_rapid_use = MenuItem(F("Run Rapid Cool"), kBoxFishMenuItemRapidCool);

  static MenuItem mi_system = MenuItem(F("[System]"));
  static MenuItem mi_system_version = MenuItem(F("[Version]"));
  static MenuItem mi_system_version_show = MenuItem(kBoxFishOvenVersion);
  static MenuItem mi_system_reset = MenuItem(F("[Reset]"));
  static MenuItem mi_system_reset_use = MenuItem(F("Reset Controller"), kBoxFishMenuItemReset);

  // setup the menu hierarchy
  root.add(mi_reflow).add(mi_anneal).add(mi_cure).add(mi_rapid).add(mi_system);

  // configure the remaining menus
  mi_reflow.addRight(mi_reflow_lead).add(mi_reflow_lead_free);
  mi_reflow_lead.addRight(mi_reflow_lead_use);
  mi_reflow_lead_free.addRight(mi_reflow_lead_free_use);

  mi_anneal.addRight(mi_anneal_thickness).add(mi_anneal_acrylic).add(mi_anneal_polycarbonate).add(mi_anneal_acetal);
  mi_anneal_acrylic.addRight(mi_anneal_acrylic_use);
  mi_anneal_polycarbonate.addRight(mi_anneal_polycarbonate_use);
  mi_anneal_acetal.addRight(mi_anneal_acetal_use);

  mi_cure.addRight(mi_cure_epoxy80);
  mi_cure_epoxy80.addRight(mi_cure_epoxy80_use);
  
  mi_rapid.addRight(mi_rapid_use);

  mi_system.addRight(mi_system_version).add(mi_system_reset);
  mi_system_reset.addRight(mi_system_reset_use);
  mi_system_version.addRight(mi_system_version_show);
}

void menuItemWasSelected(int item)
{
  // this function is called when the user selects one of the menu items where a second
  // argument was passed to MenuItem during creation. The name of this function is
  // passed to ui.begin() in setup().

  unsigned long hold_time_min = (mmThickness/6.35) * 30.0 + 0.5;
  
  switch (item) {
    case kBoxFishMenuItemLeadFree:
      // standard lead free profile, peak 245C
      startReflow(245.0);
      break;

    case kBoxFishMenuItemLeaded:
      // standard leaded profile, peak 225C
      startReflow(225.0);
      break;

    case kBoxFishMenuItemSetThickness:
      // allows user to set the hold_time for annealing by selecting the material thickness
      menuSetThickness();
      break;

    case kBoxFishMenuItemAcrylic:
      // ramp up to 82C over 2 hrs, hold for hold_time_min minutes, and ramp down to 30C at 27.8C/hr
      startAnneal(82.0, 2*60, hold_time_min, 27.8);
      break;

    case kBoxFishMenuItemPolycarbonate:
      // ramp up to 135C over 4 hrs, hold for hold_time_min minutes, and ramp down to 30C at 27.8C/hr
      startAnneal(135.0, 4*60, hold_time_min, 27.8);
      break;
      
    case kBoxFishMenuItemAcetal:
      // ramp up to 160C over 4 hrs, hold for hold_time_min minutes, and ramp down to 30C at 27.8C/hr
      startAnneal(160.0, 4*60, hold_time_min, 27.8);
      break;

    case kBoxFishMenuItemCureEpoxy80:
      // ramp up slowly to 40C, hold for 60 minutes, ramp up slowly to 80C and hold for 180 minutes, then slow cool
      startEpoxy(40.0, 60.0, 80.0, 180.0);
      break;

    case kBoxFishMenuItemRapidCool:
      startRapidCool();
      break;

    case kBoxFishMenuItemReset:
      reset();
      break;

    default:
      // do nothing
      break;
  }
}

void menuDisplayThickness()
{
  String thickness = String(mmThickness, 0) + F("mm");
  ui.displayOverwriteMenu(thickness);
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


// reusable PID operation objects to save memory
PIDOp preheat;
PIDOp soak;
PIDOp reflow;
PIDOp cool;

void ovenBegin()
{
  // display a message and reset the job timer
  ui.displayInfo(F("Start"));
  delay(1000);
  jobSeconds  = 0;

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

void startReflow(double reflow_temperature)
{
  // For profile see:
  //  http://www.kaschke.de/fileadmin/user_upload/documents/datenblaetter/Induktivitaeten/Reflowprofile.pdf
  //  http://www.compuphase.com/electronics/reflowsolderprofiles.htm

  // ready for a new job
  ovenBegin();

  // preheat: raise temperature quickly to 150C
  preheat.begin(150.0, 112.0, 0.02, 252.0);
  preheat.setEpsilon(4.0);
  preheat.setControlLimits(0.0, kWindowSize);
  preheat.setName(F("Preheat"));
  ovenSeq.addOp(preheat);
  
  // soak: now raise temperature to 200C over 100 seconds
  soak.begin(200.0, 622.0, 0.22, 1122.0);
  soak.setRampTime(100);
  soak.setEpsilon(4.0);
  soak.setControlLimits(0.0, kWindowSize);
  soak.setName(F("Soak"));
  ovenSeq.addOp(soak);
  
  // reflow: raise temperature quickly to the reflow_temperature
  reflow.begin(reflow_temperature+4.0, 242.0, 0.0, 25.0);
  reflow.setHoldTime(10); // hold for 10 seconds to allow pins on metal connectors to reflow
  reflow.setEpsilon(5.0);
  reflow.setControlLimits(0.0, kWindowSize);
  reflow.setName(F("Reflow"));
  ovenSeq.addOp(reflow);
  
  // cool: cool quickly using the blower
  cool.begin(50.0, 40.0, 0.01, 30.0);
  cool.setReverse(true);
  cool.setEpsilon(3.0);
  cool.setControlLimits(0.0, kBlowerPWMMax);
  cool.setName(F("Cool"));
  ovenSeq.addOp(cool);

  // start the sequence
  ovenRun();
}

void startAnneal(double hold_temp, unsigned long ramp_up_minutes, unsigned long hold_minutes, double ramp_down_deg_hour)
{
  // All temps in C. For profiles (must convert F to C) see:
  //  http://www.boedeker.com/anneal.htm
  
  // ready for a new job
  ovenBegin();
  
  double epsilon = 3.0;

  // slowly increase heat from room temperature to hold_temp (C) over ramp_up_minutes minnutes
  preheat.begin(hold_temp, 300.0, 0.03, 200.0);
  preheat.setRampTime(ramp_up_minutes * 60uL);
  preheat.setEpsilon(epsilon);
  preheat.setControlLimits(0.0, kWindowSize);
  preheat.setName(F("Slow Heat"));
  ovenSeq.addOp(preheat);

  // hold at hold_temp for hold_minutes minutes
  soak.begin(hold_temp, 300.0, 0.02, 200.0);
  soak.setHoldTime(hold_minutes * 60uL);
  soak.setEpsilon(epsilon);
  soak.setControlLimits(0.0, kWindowSize);
  soak.setName(F("Hold"));
  ovenSeq.addOp(soak);
  
  // cool to 30C at ramp_down_deg_hour degrees C/hr (note this is actually a heating cycle with decreasing setpoint)
  unsigned long ramp_down_minutes = ((hold_temp - 30.0) * 60.0) / ramp_down_deg_hour + 0.5;
  cool.begin(30.0, 300.0, 0.03, 200.0);
  cool.setRampTime(ramp_down_minutes * 60uL);
  cool.setEpsilon(epsilon);
  cool.setControlLimits(0.0, kWindowSize);
  cool.setName(F("Slow Cool"));
  ovenSeq.addOp(cool);

  // start the sequence
  ovenRun();
}

void startEpoxy(double hold_temp1, unsigned long hold_minutes1, unsigned long hold_temp2, unsigned long hold_minutes2)
{
  // ready for a new job
  ovenBegin();
  
  double epsilon = 3.0;

  // slowly increase heat from room temperature to hold_temp1 (C) over 20 minutes then hold for hold_minutes1
  preheat.begin(hold_temp1, 300.0, 0.03, 200.0);
  preheat.setRampTime(20uL * 60uL);
  preheat.setHoldTime(hold_minutes1 * 60uL);
  preheat.setEpsilon(epsilon);
  preheat.setControlLimits(0.0, kWindowSize);
  preheat.setName(F("Cure Temp 1"));
  ovenSeq.addOp(preheat);

  // slowly increase heat from hold_temp1 to hold_temp2 (C) over 20 minutes then hold for hold_minutes2
  soak.begin(hold_temp2, 300.0, 0.03, 200.0);
  soak.setRampTime(20uL * 60uL);
  soak.setHoldTime(hold_minutes2 * 60uL);
  soak.setEpsilon(epsilon);
  soak.setControlLimits(0.0, kWindowSize);
  soak.setName(F("Cure Temp 2"));
  ovenSeq.addOp(soak);
  
  // cool to 30C at 50C/hr (note this is actually a heating cycle with decreasing setpoint)
  double ramp_down_deg_hour = 50.0;
  unsigned long ramp_down_minutes = ((hold_temp2 - 30.0) * 60.0) / ramp_down_deg_hour + 0.5;
  cool.begin(30.0, 300.0, 0.03, 200.0);
  cool.setRampTime(ramp_down_minutes * 60uL);
  cool.setEpsilon(epsilon);
  cool.setControlLimits(0.0, kWindowSize);
  cool.setName(F("Cool Down"));
  ovenSeq.addOp(cool);

  // start the sequence
  ovenRun();
}

void startRapidCool()
{
  // ready for a new job
  ovenBegin();

  // aggressively cool to 40.0C
  cool.begin(40.0 - 5.0, 38, 0.008, 32);
  cool.setEpsilon(5.0);
  cool.setReverse(true);
  cool.setControlLimits(0.0, kBlowerPWMMax);
  cool.setName(F("Rapid Cool"));
  ovenSeq.addOp(cool);
  
  // start the sequence
  ovenRun();
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


