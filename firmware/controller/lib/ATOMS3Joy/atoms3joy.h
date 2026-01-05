/*
* MIT License
* 
* Copyright (c) 2024 Kouhei Ito
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

#ifndef ATOMS3JOY_H
#define ATOMS3JOY_H
#include <stdint.h>
#include <M5AtomS3.h>

#define I2C_ADDRESS (0x59)
#define LEFT_STICK_X_ADDRESS (0x00)
#define LEFT_STICK_Y_ADDRESS (0x02)
#define RIGHT_STICK_X_ADDRESS (0x20)
#define RIGHT_STICK_Y_ADDRESS (0x22)
#define LEFT_STICK_BUTTON_ADDRESS (0x70)
#define RIGHT_STICK_BUTTON_ADDRESS (0x71)
#define LEFT_BUTTON_ADDRESS (0x72)
#define RIGHT_BUTTON_ADDRESS (0x73)

#define LEFTX (0)
#define LEFTY (1)
#define RIGHTX (2)
#define RIGHTY (3)

#define NEW_ATOM_JOY
#ifdef NEW_ATOM_JOY
#define BATTERY_VOLTAGE1 (0x60)
#define BATTERY_VOLTAGE2 (0x62)
#define LEFT_STICK_BUTTON (2)
#define RIGHT_STICK_BUTTON (3)
#define LEFT_BUTTON (0)
#define RIGHT_BUTTON (1)
#else
#define BATTERY_VOLTAGE1 (0x60)
#define LEFT_STICK_BUTTON (0)
#define RIGHT_STICK_BUTTON (1)
#define LEFT_BUTTON (2)
#define RIGHT_BUTTON (3)
#endif


//Control Mapping
//#define THROTTLE RIGHTY
//#define AILERON LEFTX
//#define ELEVATOR LEFTY
//#define RUDDER RIGHTX
//#define ARM_BUTTON RIGHT_STICK_BUTTON
//#define MODE_BUTTON RIGHT_BUTTON
//#define FLIP_BUTTON LEFT_STICK_BUTTON
//#define OPTION_BUTTON LEFT_BUTTON
extern uint8_t THROTTLE;
extern uint8_t AILERON;
extern uint8_t ELEVATOR;
extern uint8_t RUDDER;
extern uint8_t ARM_BUTTON;
extern uint8_t MODE_BUTTON;
extern uint8_t FLIP_BUTTON;
extern uint8_t OPTION_BUTTON;




extern float Battery_voltage[2];

void joy_update(void);
uint16_t getThrottle(void);
uint16_t getAileron(void);
uint16_t getElevator(void);
uint16_t getRudder(void);
uint8_t getArmButton(void);
uint8_t getModeButton(void);
uint8_t getFlipButton(void);
uint8_t getOptionButton(void);

#endif