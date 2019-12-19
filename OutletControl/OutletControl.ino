#include <util/crc16.h>
#include "AzzyRFRX.h"

byte MyAddress = 0;

const byte commands[][4] = {
  {0x1E,0x40,0xFF,0x00}, //Pin 0, PA6
  {0x1E,0x40,0xFF,0x10}, //pin 1, PA7
  {0x1E,0x40,0xFF,0x20}, //pin 2, PA1
  {0x1E,0x41,0xFF,0x10}, //Same as tower light, but on distinct signal
  {0x1E,0x40,0x00,0x00},
  {0x1E,0x40,0x00,0x10},
  {0x1E,0x40,0x00,0x20},
  {0x1F,0xE1,0x02,0x00} //legacy tower light signal
};

const byte portbv[3]={0x40,0x80,0x02};

#define BRIGHTNESS_THRESHOLD 400 //out of 1024;
#define LIGHT_DURATION 60000UL //duration for stair light to stay on, max
#define RIG_DURATION 1000UL*60*60 //duration for rig to stay on, max

unsigned long stairsOnAt=0;
unsigned long rigOnAt=0;

void setup() {
  // put your setup code here, to run once:
  PORTA.DIR=0xC2; //set PA6, PA7, PA1 OUTPUT
  setupTimer();
}

void loop() {
  // put your main code here, to run repeatedly:
  byte rlen = handleReceive();
  if (rlen==4) {
    byte cmd=255;
    for (byte i=0;i<9;i++) {
      if (memcmp(commands[i],recvMessage,4)==0) {
        cmd=i;
        break;
      }
    }
    if (cmd<8){
      if (cmd == 7) {
        analogRead(3); //dummy readings
        analogRead(3); //dummy readings
        if (analogRead(3) < BRIGHTNESS_THRESHOLD) { //if brightness is below threshold
          PORTA.OUTSET=portbv[1];
          stairsOnAt=millis();
        }
      } else if (cmd == 2) {
        PORTA.OUTSET=portbv[2];
        rigOnAt=millis();
      } else if (cmd == 3) {
        PORTA.OUTSET=portbv[1];
        stairsOnAt=millis();
      } else { //cmd is 0~2 or 4~6
        if (cmd>3) { //cmd is 4,5,6 OFF 
          PORTA.OUTCLR=portbv[(cmd&0x03)];
        } else {//cmd is 0,1,2 ON
          PORTA.OUTSET=portbv[(cmd&0x03)];
        }
      }
    }
  } else {
    if (stairsOnAt && ((millis()-stairsOnAt)>(LIGHT_DURATION))) {
      stairsOnAt=0;
      PORTA.OUTCLR=portbv[1];
    }
  }
}
