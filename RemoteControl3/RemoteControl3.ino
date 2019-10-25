#include <avr/sleep.h>
#include <avr/power.h>
#include <util/crc16.h>
#include "AzzyRFTX.h"
#include "Commands.h"
#include "Hardware.h"


#define PCMSK0_SLEEP 0xF9 //0b11111001


byte sleeping = 0;


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
  static unsigned long lastBtnTime=0;
  
  if (btnst != lastst) { //if the button state changes quickly, then that means people are pushing more buttons
    lastBtnTime=millis();
    lastst=btnst;
  }
  
  if (btnst && millis()-lastBtnTime>500) {
    //it's been long enough
    doButtonAction(btnst);
  }

  if (!btnst) { //make sure all the buttons are not pressed, otherwise skip sleep and send the signal again
    if (lastBtnState) {
      doButtonAction(btnst);
    }
    btnst=0;
    lastst=0;
    lastBtnTime=0;
    
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
doButtonAction(byte btnst) {
  if (btnst) {
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
}

ISR (PCINT0_vect) // handle pin change interrupt for D0 to D7 here
{
  PCMSK0 = 0; //disable the interrupts by masking it off.
  sleeping = 0;
}

