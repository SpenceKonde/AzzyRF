#include <avr/sleep.h>
#include <avr/power.h>
#include <util/crc16.h>
#include "AzzyRFTX.h"
#include "Buttons.h"


#define PCMSK0_SLEEP 0xF9 //0b11111001


byte sleeping = 0;
byte btnst = 0;


void setup() {
  pinMode(txpin,OUTPUT);
  setupAzzyRFTX();
  setupButtons();
  GIMSK |= 1 << PCIE0; //enable PCINT on port A
  PCMSK0 = 0;
  ADCSRA &= 127; //turn off ADC, we don't need it and it's just gonna waste power.

}

void loop() {
  byte btnst = getButtonState();
  static byte lastst=0;
  if (btnst) {
    byte vers=1;
    if (btnst == 1 ) {
      sendPacket(0,1);
    } else if (btnst == 2) {
      sendPacket(4,1);
    } else if (btnst == 4) {
      sendPacket(1,1);
    } else if (btnst == 8) {
      sendPacket(5,1);
    } else if (btnst ==16){
      sendPacket(3,1);
    } else if (btnst ==32){
      sendPacket(7,1);
    }
  }

  if (!btnst) { //make sure all the buttons are not pressed, otherwise skip sleep and send the signal again
    PCMSK0 = PCMSK0_SLEEP;
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleeping = 1;
    sleep_enable();
    //Serial.println("sleep enabled");
    //delay(200);
    sleep_mode();
    //now we're sleeping....
  
    sleep_disable(); //execution will continue from here.
    delay(60);
  }
}

ISR (PCINT0_vect) // handle pin change interrupt for D0 to D7 here
{
  PCMSK0 = 0; //disable the interrupts by masking it off.
  sleeping = 0;
}


#ifndef OLD_PINOUT
#error "Wrong pinout selected!"
#endif
