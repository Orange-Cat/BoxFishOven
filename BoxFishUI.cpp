//
// Title: BoxFish UI
// Author: Orange Cat
// Date: 23-10-2015
//
// License:
//   This firmware is released under the Creative Commons Attribution-ShareAlike 4.0
//   International license.
//     http://creativecommons.org/licenses/by-sa/4.0/
//
#include <Arduino.h>
#include <String.h>
#include <avr/wdt.h>
#include "BoxFishUI.h"

// LCD display PIN assignments
#ifndef BOXFISH_USE_ADAFRUIT_LCD
static const int buttonPin = A0;
static const int backlightPin = 30;
static const int buzzerPin = -1;      // set buzzerPin to -1 where there is no buzzer available
#else
static const int buzzerPin = 6;
#endif


static BoxFishUI* lcd;


BoxFishUI::BoxFishUI()
#ifdef BOXFISH_USE_ADAFRUIT_LCD
  :Adafruit_RGBLCDShield(),
#else
  :LiquidCrystal(8, 9, 4, 5, 6, 7),   // Freetronics 16x2 LCD sheild v2.0
#endif
  menu_(menuUseEventCallback, menuChangeEventCallback),
  button_state_(kBoxFishButtonNone),
  last_button_reading_(kBoxFishButtonNone),
  debounced_button_(kBoxFishButtonNone),
  debounce_time_(0),
  callback_func_(NULL),
  prog_name_(""),
  prog_version_("")
{
  return;
}

MenuItemRef BoxFishUI::getRootMenu()
{
  return menu_.getRoot();
}

MenuItemRef BoxFishUI::getCurrentMenu()
{
  return menu_.getCurrent();
}

void BoxFishUI::redisplayCurrentMenu()
{
  menuDisplayMenu(menu_.getCurrent());
}

void BoxFishUI::menuGotoRoot()
{
  menu_.toRoot();
}

void BoxFishUI::menuUseEventCallback(MenuUseEvent used)
{
  // callled when a menu item is selected

  int selection = (int) used.item.getShortkey();
  if (selection != '\0' && lcd->callback_func_ != NULL) {
    lcd->callback_func_(selection);
  }
}

void BoxFishUI::displaySplash()
{
  // display splash screen
  clear();
  setCursor(0, 0);
  print(prog_name_);
  setCursor(0, 1);
  print("v");
  print(prog_version_);

  beep();
  delay(2000);

  clear();
}

void BoxFishUI::writeExactlyAt(unsigned int x, unsigned int y, const String& str, unsigned int width)
{
  // writes exactly width chars, truncating the string to width and padding on the right if necessary to fill width
  setCursor(x, y);
  
  unsigned int i;
  for (i=0; i < str.length() && i < width; i++) {
    write(str[i]);
  }
  while (i < width) {
    write(' ');
    i++;
  }
}

void BoxFishUI::writeStatus(const String& stat)
{
  // lower line, last 5 chars 
  writeExactlyAt(11, 1, stat, 5);
}

void BoxFishUI::writeInfo(const String& info)
{
  // lower line, first 10 chars
  writeExactlyAt(0, 1, info, 10);
}

void BoxFishUI::overwriteMenu(const String& menu)
{
  // upper line, all 16 chars
  writeExactlyAt(0, 0, menu, 16);
}

void BoxFishUI::beep()
{
  if (buzzerPin != -1) {
    tone(buzzerPin, 1000, 200);
  }
}

void BoxFishUI::lcdSetup()
{
#ifndef BOXFISH_USE_ADAFRUIT_LCD
  // all buttons connected to one analog input:
  pinMode(buttonPin, INPUT);
  digitalWrite(buttonPin, LOW);
  
  // turn on backlight
  digitalWrite(backlightPin, HIGH);
  pinMode(backlightPin, OUTPUT);
#endif

  // create degree character for LCD as char kBoxFishDegreeChar
  static uint8_t degree[8]  = {
    140, 146, 146, 140, 128, 128, 128, 128
  };
  createChar(kBoxFishDegreeChar, degree);
}

void BoxFishUI::begin(const char program_name[], const char program_version[], BoxFishMenuCallback callback)
{
  lcd = this;
  LCD_CLASS::begin(16, 2);
  
  prog_name_ = program_name;
  prog_version_ = program_version;
  callback_func_ = callback;
  
  button_state_ = kBoxFishButtonNone;

  lcdSetup();
  displaySplash();
  if (buzzerPin != -1) {
    noTone(buzzerPin);
  }
}

void BoxFishUI::menuChangeEventCallback(MenuChangeEvent changed)
{
  // called when the menu system is traversed
  lcd->menuDisplayMenu(changed.to);
}

void BoxFishUI::menuNavigate()
{
  // should be called everytime through loop(), handles button pressing and menu displaying
  lcd = this;
  
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

void BoxFishUI::menuDisplayMenu(const MenuItem& menu)
{
  // clear line and reset cursor
  setCursor(0, 0);
  print("                ");
  setCursor(0, 0);

  String name;
  if (menu.getName() != NULL) {
    name = menu.getName();
  }
  else {
    name = menu.getFlashName();
  }
  if (name == "MenuRoot") {
    print("[");
    print(lcd->prog_name_);
    print("]");
  }
  else {
    print(name);
  }
}

BoxFishButton BoxFishUI::debounce(BoxFishButton button_reading)
{
  // simple debouce method, if button is stable for kDebouceDelay milliseconds
  // and it'state has changed, then the button is returned, otherwise just
  // returns no button.

  if (button_reading != last_button_reading_) {
    // button status has changed, we reset the debounce time
    // and return no button as at the moment the button is not stable
    last_button_reading_ = button_reading;
    debounce_time_ = millis();
    return kBoxFishButtonNone;
  }

  if ((millis() - debounce_time_) > kDebouceDelay) {
    // button is stable
    if (button_reading != debounced_button_) {
       // it's new so return this state
       last_button_reading_ = button_reading;
       debounced_button_ = button_reading;
       return debounced_button_;
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
  uint8_t buttons = readButtons();

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
  const unsigned short kButtonRes10BitRight = 0;
  const unsigned short kButtonRes10BitUp = 145;
  const unsigned short kButtonRes10BitDown = 329;
  const unsigned short kButtonRes10BitLeft = 505;
  const unsigned short kButtonRes10BitSelect = 741;
  const unsigned short kButtonResTolerance = 10;   // we accept +/- this many ADC counts from the expected reading

  unsigned short button_adc;

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

void BoxFishUI::softReset()
{
  // enable watchdog timer
  wdt_enable(WDTO_15MS);

  // loop forever until watchdog timer resets the microcontroller
  for (;;) {
    continue;
  }
}

