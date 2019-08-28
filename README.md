# odroid-carpc-due
An Arduino Due project for controlling an Odroid N2 via BMW Keyfob and iDrive.

## Hardware used
- BMW 1-Series 2009
- Arduino Due
- Canbus module with 16mhz crystal
- Sparkfun Bluesmirf Silver
- iDrive 7-Button Controller 

## Purpose
This project is intended to support an Odroid N2 in its function as a car-pc. This also involves controlling the display brightness of the Vu7+ display by applying PWM signals to its backlight chip by reacting on the cars' light sensor on the windscreen.
As a bonus it's designed to support an iDrive controller as an input device for the Odroid to have comfortable access to the car-pc functions. To make it function properly an app called "Button Mapper" is used to react on F-Keys.
See: https://play.google.com/store/apps/details?id=flar2.homebutton

## 3rd Party Libraries used
I'm using a modified version of the BPLib that now supports sending and holding multiple keys. That's probably important for future tweaks. The original library only supports sending one single key and modifiers.
See: https://github.com/Neuroquila-n8fall/BPLib

Also: Seeedstudios Canbus Shield library
See: https://github.com/Seeed-Studio/CAN_BUS_Shield

## Usage
### Wiring
W.I.P.
### Parameters
W.I.P.

## Known Issues
W.I.P.

## Questions I got asked...
- Why an Arduino Due?
Well...it's 84mhz CPU is capable of handling all the canbus chatter if you desire so. At it's current state the overall load could be handled by a 16mhz CPU aswell... I found it much more responding than its little cousins
- Why???
Because I always felt the need to include a full blown car pc into my car but the original iDrive system is just too limited. I want a brwosable music library with cloud sync, internet, the precious Torque app, ...all the good stuff!
The icing on the cake was the decision to go the extra mile and swap the center console and install an iDrive 7-Button control unit. It looks like OEM but it does much, much more.

