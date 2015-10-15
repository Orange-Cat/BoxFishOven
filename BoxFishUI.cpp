//
// Title: BoxFish UI
// Author: Orange Cat
// Date: 10-11-2015
//
// License:
//   This firmware is released under the Creative Commons Attribution-ShareAlike 4.0
//   International license.
//     http://creativecommons.org/licenses/by-sa/4.0/
//
#include <Arduino.h>
#include <String.h>
#include "BoxFishUI.h"

// define to use the Adafruit LCD.
#define BOXFISH_USE_ADAFRUIT_LCD


#ifdef BOXFISH_USE_ADAFRUIT_LCD
  //#include <Adafruit_MCP23017.h>
  #include <Adafruit_RGBLCDShield.h>
#else
  #include <LiquidCrystal.h>
#endif

// LCD display PIN assignments
#ifndef BOXFISH_USE_ADAFRUIT_LCD
static const int buttonPin = A0;
static const int backlightPin = 30;
static const int buzzerPin = -1;      // set buzzerPin to -1 where there is no buzzer available
#else
static const int buzzerPin = 6;
#endif

// Specify LCD interface
#ifdef BOXFISH_USE_ADAFRUIT_LCD
static Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
#else
static LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
#endif

static BoxFishMenuCallback callback_func = NULL;

static void menuChangeEventCallback(MenuChangeEvent changed);
static void menuUseEventCallback(MenuUseEvent used);

static const char* prog_name = "";
static const char* prog_version = "";


BoxFishUI::BoxFishUI()
  :menu_(menuUseEventCallback, menuChangeEventCallback),
  button_state_(kBoxFishButtonNone)
{
  return;
}

MenuItemRef BoxFishUI::getRootMenu()
{
  return menu_.getRoot();
}

static void menuUseEventCallback(MenuUseEvent used)
{
  // callled when a menu item is selected

  int selection = (int) used.item.getShortkey();
  if (selection != '\0' && callback_func != NULL) {
    callback_func(selection);
  }
}

void BoxFishUI::displaySplash()
{
  // display splash screen
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(prog_name);
  lcd.setCursor(0, 1);
  lcd.print("v");
  lcd.print(prog_version);

  beep();
  delay(2000);

  lcd.clear();
}

void BoxFishUI::displayStatus(String stat)
{
  // lower line, last 5 chars
  lcd.setCursor(11, 1);
  
  int i;
  for (i=0; i < 5 && stat[i] != '\0'; i++) {
    lcd.write(stat[i]);
  }
  while (i++ < 5) {
    lcd.write(' ');
  }
}

void BoxFishUI::displayInfo(String info)
{
  // lower line, first 10 chars
  lcd.setCursor(0, 1);

  int i;
  for (i=0; i < 10 && info[i] != '\0'; i++) {
    lcd.write(info[i]);
  }
  while (i++ < 10) {
    lcd.write(' ');
  }
}

void BoxFishUI::beep()
{
  if (buzzerPin != -1) {
    tone(buzzerPin, 1000, 200);
  }
}

void BoxFishUI::lcdSetup()
{
  lcd.begin(16, 2);

#ifndef BOXFISH_USE_ADAFRUIT_LCD
  pinMode( buttonPin, INPUT );         //ensure A0 is an input
  digitalWrite( buttonPin, LOW );      //ensure pullup is off on A0

  digitalWrite( backlightPin, HIGH );  //backlight control pin D3 is high (on)
  pinMode( backlightPin, OUTPUT );     //D3 is an output
#endif

  // create degree character for LCD as char kBoxFishDegreeChar
  static uint8_t degree[8]  = {
    140, 146, 146, 140, 128, 128, 128, 128
  };
  lcd.createChar(kBoxFishDegreeChar, degree);
}

void BoxFishUI::begin(const char program_name[], const char program_version[], BoxFishMenuCallback callback)
{
  prog_name = program_name;
  prog_version = program_version;
  callback_func = callback;
  
  button_state_ = kBoxFishButtonNone;

  lcdSetup();
  displaySplash();
  if (buzzerPin != -1) {
    noTone(buzzerPin);
  }
}

static void menuChangeEventCallback(MenuChangeEvent changed)
{
  // called when the menu system is traversed
  MenuItem new_menu = changed.to;

  // clear line and reset cursor
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 0);

  String name = String(new_menu.getName());
  if (name == "MenuRoot") {
    lcd.print("[");
    lcd.print(prog_name);
    lcd.print("]");
  }
  else {
    lcd.print(name);
  }
}

void BoxFishUI::menuNavigate()
{
  // should be called everytime through loop(), handles button pressing and menu displaying
  
  BoxFishButton button_state = readButton();
  MenuItem cur_menu = menu_.getCurrent();

  switch (button_state) {
    case kBoxFishButtonDown:
      menu_.moveDown();
      break;

    case kBoxFishButtonUp:
      menu_.moveUp();
      break;

    case kBoxFishButtonRight:
      if (!(cur_menu.moveRight())) {
        menu_.use();       // invoke callback if terminal item
      }
      else {
        menu_.moveRight();
      }
      break;      

    case kBoxFishButtonLeft:
      menu_.moveLeft();
      break;

    case kBoxFishButtonNone:
    default:
      break;
  }
}

void BoxFishUI::menuGotoRoot()
{
  menu_.toRoot();
}

BoxFishButton BoxFishUI::debounce(BoxFishButton button_reading)
{
  // simple debouce method, if button is stable for kDebouceDelay milliseconds
  // and it'state has changed, then the button is returned, otherwise just
  // returns no button.
  static const long kDebouceDelay = 20;   // in milliseconds
  static BoxFishButton last_button_reading = kBoxFishButtonNone;
  static BoxFishButton debounced_button = kBoxFishButtonNone;
  static long debounce_time = millis();

  if (button_reading != last_button_reading) {
    // button status has changed, we reset the debounce time
    // and return no button as at the moment the button is not stable
    last_button_reading = button_reading;
    debounce_time = millis();
    return kBoxFishButtonNone;
  }

  if ((millis() - debounce_time) > kDebouceDelay) {
    // button is stable
    if (button_reading != debounced_button) {
       // it's new so return this state
       last_button_reading = button_reading;
       debounced_button = button_reading;
       return debounced_button;
    }
  }
  return kBoxFishButtonNone;
}

BoxFishButton BoxFishUI::readButton()
{
  // hardware dependent button reading. Note that we do not OR buttons together
  // because detecting multiple simultaneous button presses cannot be done on
  // all hardware.
  BoxFishButton button_reading;
  
#ifdef BOXFISH_USE_ADAFRUIT_LCD
  uint8_t buttons = lcd.readButtons();

  if (buttons & BUTTON_SELECT) {
    button_reading = kBoxFishButtonSelect;
  }
  else if (buttons & BUTTON_UP) {
    button_reading = kBoxFishButtonUp;
  }
  else if (buttons & BUTTON_DOWN) {
    button_reading = kBoxFishButtonDown;
  }
  else if (buttons & BUTTON_LEFT) {
    button_reading = kBoxFishButtonLeft;
  }
  else if (buttons & BUTTON_RIGHT) {
    button_reading = kBoxFishButtonRight;
  }
  else {
    button_reading = kBoxFishButtonNone;
  }
#else
  // ADC readings expected for the 5 buttons on the ADC input
  static const unsigned int kButtonRes10BitRight = 0;
  static const unsigned int kButtonRes10BitUp = 145;
  static const unsigned int kButtonRes10BitDown = 329;
  static const unsigned int kButtonRes10BitLeft = 505;
  static const unsigned int kButtonRes10BitSelect = 741;
  static const unsigned int kButtonResTolerance = 10;   // we accept +/- this many ADC counts from the expected reading

  unsigned int button_adc;

  //read the button ADC pin voltage
  button_adc = analogRead(buttonPin);

  //sense if the voltage falls within valid voltage windows
  if (button_adc < kButtonRes10BitRight + kButtonResTolerance) {
    button_reading = kBoxFishButtonRight;
  }
  else if (button_adc >= kButtonRes10BitUp - kButtonResTolerance && button_adc <= kButtonRes10BitUp + kButtonResTolerance) {
    button_reading = kBoxFishButtonUp;
  }
  else if (button_adc >= kButtonRes10BitDown - kButtonResTolerance && button_adc <= kButtonRes10BitDown + kButtonResTolerance) {
    button_reading = kBoxFishButtonDown;
  }
  else if (button_adc >= kButtonRes10BitLeft - kButtonResTolerance && button_adc <= kButtonRes10BitLeft + kButtonResTolerance) {
    button_reading = kBoxFishButtonLeft;
  }
  else if (button_adc >= kButtonRes10BitSelect - kButtonResTolerance && button_adc <= kButtonRes10BitSelect + kButtonResTolerance) {
    button_reading = kBoxFishButtonSelect;
  }
  else {
    button_reading = kBoxFishButtonNone;
  }
#endif
  button_state_ = debounce(button_reading);  
  return button_state_;
}


BoxFishButton BoxFishUI::lastButton()
{
  // returns the last button state.
  return button_state_;
}

