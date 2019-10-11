//This file contains stuff that is specific to the hardware in use - sleep, the pins that buttons use, and so on. 
#include <avr/sleep.h>
#include <avr/power.h>

//This code is for the counterclockwise pinout of the attiny841. 
#ifndef OLD_PINOUT
#error "Wrong pinout selected!"
#endif

#define BUTTON_1 10 //PA0
#define BUTTON_2 7  //PA3
#define BUTTON_3 6  //PA4
#define BUTTON_4 5  //PA5
#define BUTTON_5 4  //PA6
#define BUTTON_6 3  //PA7

#define PCMSK0_SLEEP 0xF9 //0b11111001
//#define PCMSK1_SLEEP //use this if other ports are used.

void setupButtons(){
  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);
  pinMode(BUTTON_3, INPUT_PULLUP);
  pinMode(BUTTON_4, INPUT_PULLUP);
  pinMode(BUTTON_5, INPUT_PULLUP);
  pinMode(BUTTON_6, INPUT_PULLUP);
}

void setupInterrupts(){
  GIMSK |= 1 << PCIE0; //enable PCINT on port A
  PCMSK0 = 0;
}

void enterSleep() {
  PCMSK0 = PCMSK0_SLEEP;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();
  sleep_disable();
}

byte getButtonState() {
  byte retval=0;
  retval+=digitalRead(BUTTON_1); //pin1
  retval+=digitalRead(BUTTON_2)<<1; //pin2
  retval+=digitalRead(BUTTON_3)<<2; //pin3
  retval+=digitalRead(BUTTON_4)<<3; //pin4
  retval+=digitalRead(BUTTON_5)<<4; //pin5
  retval+=digitalRead(BUTTON_6)<<5; //pin6
  return (~retval) & 0x3F;
}
