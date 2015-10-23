//
// Title: BoxFish UI
// Author: Orange Cat
// Date: 15-10-2015
//
// BoxFishUI is a class to implement a consistent user interface on 2x16 line displays
// it divides the display up into 3 parts. The top line is reserved for the menu system. The bottom line is split into left (info) and right (status).
// the status area is 5 characters, the info area is 10.
//
// It performs the button reading and deboucing and the driving a MenuBackend menu system. So all the caller has to do is setup the menu hierarchy,
// create a callback and wait for menu options to be selected.
//
// License:
//   This firmware is released under the Creative Commons Attribution-ShareAlike 4.0
//   International license.
//     http://creativecommons.org/licenses/by-sa/4.0/
//

#ifndef BOXFISH_UI_H
#define BOXFISH_UI_H

#include <MenuBackend.h>
#include <String.h>

// define to use the Adafruit LCD.
#define BOXFISH_USE_ADAFRUIT_LCD


enum BoxFishButton {
  kBoxFishButtonNone,
  kBoxFishButtonUp = 0x01,
  kBoxFishButtonDown = 0x02,
  kBoxFishButtonLeft = 0x04,
  kBoxFishButtonRight = 0x08,
  kBoxFishButtonSelect = 0x10
};

typedef void (*BoxFishMenuCallback)(int);

static const char kBoxFishDegreeChar = '\1';     // character which prints a degree symbol on the LCD display
static const unsigned long kDebouceDelay = 20;   // key debouce delay in milliseconds
  
class BoxFishUI {
  public:
    void begin(const char program_name[], const char program_version[], BoxFishMenuCallback callback);
    MenuItemRef getRootMenu();
    MenuItemRef getCurrentMenu();
    
    void menuNavigate();        // call each time through loop()
    void menuGotoRoot();

    void displayStatus(const String& stat);
    void displayInfo(const String& info);
    void displayOverwriteMenu(const String& menu);
    void redisplayCurrentMenu();
    void writeExactlyAt(unsigned int x, unsigned int y, const String& str, unsigned int width);
    
    void beep();
    BoxFishButton readButton();
    BoxFishButton lastButton();

    void softReset();   // software reset of microcontroller

  public:
    BoxFishUI();

  private:
    MenuBackend menu_;
    BoxFishButton button_state_;
    BoxFishButton last_button_reading_;
    BoxFishButton debounced_button_;
    unsigned long debounce_time_;
    
  private:
    static void menuChangeEventCallback(MenuChangeEvent changed);
    static void menuUseEventCallback(MenuUseEvent used);
    static void menuDisplayMenu(const MenuItem& menu);
    void lcdSetup();
    void displaySplash();
    BoxFishButton debounce(BoxFishButton button);
};

#endif // BOXFISH_UI_H

