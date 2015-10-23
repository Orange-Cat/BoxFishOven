The BoxFishOven controller supports multiple reflow, annealing and curing profiles selectable from the LCD display and a design that makes it easy to add new profiles.

The target oven has two SSRs controlling the top and bottom elements (although we only turn them on/off together), a relay to control the oven's convection fan, and a PWM controlled blower that forces room air into the oven to cool it down rapidly.

Built to run on an Arduino Uno or similar with the Adafruit LCD shield and a MAX31855 thermocouple board, plus transistor/mosfet drivers for all the relays.

A built-in simulator allows the program to be tested and evaluted without any special hardware (thermocouple or relays or an oven).

**Required Libraries:**
* Adafruit RGB LCD Shield Library ( https://github.com/adafruit/Adafruit-RGB-LCD-Shield-Library )
* Arduino PID Library ( https://github.com/br3ttb/Arduino-PID-Library )
* MAX31855 Library for reading the thermocouple temperature ( https://github.com/rocketscream/MAX31855 )
* MenuBackend 1.6 ( https://github.com/Orange-Cat/MenuBackend )

**Included libraries:**
* BoxFishUI (a simple menu driven interface to a 2 line LCD display that uses MenuBackend)
* PIDSeq (a simple PID operations sequencer that uses the Arduino PID Library)

**PID Tuning:**
* http://www.cds.caltech.edu/~murray/books/AM08/pdf/am06-pid_16Sep06.pdf

**License:**
* This firmware is released under the Creative Commons Attribution-ShareAlike 4.0
* International license.
* *  http://creativecommons.org/licenses/by-sa/4.0/
