#include <avr/sleep.h>
#include <avr/power.h>
#include <util/crc16.h>

#define txpin 2
#define txPIN PINB
#define txPORT PORTB
#define txBV 4
#define RX_MAX_LEN 256
#include <EEPROM.h>
//#define LED5 4
//#define LED_ON 1
//#define LED_OFF 0
#define BUTTON_1 10
#define BUTTON_2 7
#define BUTTON_3 6
#define BUTTON_4 5 
#define BUTTON_5 4 
#define BUTTON_6 3 

#define PCMSK0_SLEEP 0xF9 //0b11111001

unsigned char txrxbuffer[RX_MAX_LEN >> 3];
byte btnst = 0;
byte myid;
byte mytarget=31;
byte sleeping = 0;
byte TXLength = 0;


unsigned int txOneLength  = 525;
unsigned int txZeroLength  = 300;
unsigned int txLowTime  = 300;
unsigned int txSyncTime  = 2000;
unsigned int txTrainLen  = 200;
byte txTrainRep  = 20;

const byte commands[][4] = {
  {0x1E,0x40,0xFF,0x00}, //Pin 0, PA6
  {0x1E,0x40,0xFF,0x01}, //pin 1, PA7
  {0x1E,0x40,0xFF,0x02}, //pin 2, PA1
  {0x1E,0x40,0xFF,0x03}, //pin 3, PA2
  {0x1E,0x40,0x00,0x00},
  {0x1E,0x40,0x00,0x01},
  {0x1E,0x40,0x00,0x02},
  {0x1E,0x40,0x00,0x03},
  {0x1F,0x1E,0x02,0x00} //legacy tower light signal
};
/*
unsigned int rxSyncMin  = 1750;
unsigned int rxSyncMax  = 2250;
unsigned int rxZeroMin  = 100;
unsigned int rxZeroMax  = 490;
unsigned int rxOneMin  = 510;
unsigned int rxOneMax  = 900;
unsigned int rxLowMax  = 900;
unsigned int txOneLength  = 700;
unsigned int txZeroLength  = 300;
unsigned int txLowTime  = 500;
unsigned int txSyncTime  = 2000;
unsigned int txTrainLen  = 200;
byte txTrainRep  = 30;
*/
unsigned int txRepDelay = 2000; //delay between consecutive transmissions
byte txRepCount = 5; //number of times to repeat each transmission

void setup() {
  if (EEPROM.read(3) < 255) {
#ifdef OSCCAL
    OSCCAL = EEPROM.read(3);
#else
    OSCCAL0 = EEPROM.read(3);
#endif
    delay(50); //let's be cautious;
  }
  byte tval = EEPROM.read(8);
  mytarget = (tval == 255) ? mytarget : tval;
  tval = EEPROM.read(9);
  myid = (tval == 255) ? myid : tval;
  pinMode(txpin,OUTPUT);
  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);
  pinMode(BUTTON_3, INPUT_PULLUP);
  pinMode(BUTTON_4, INPUT_PULLUP);
  pinMode(BUTTON_5, INPUT_PULLUP);
  pinMode(BUTTON_6, INPUT_PULLUP);
  GIMSK |= 1 << PCIE0; //enable PCINT on port A
  PCMSK0 = 0;
  ADCSRA &= 127; //turn off ADC, we don't need it and it's just gonna waste power.
  Serial.begin(9600);
  delay(500);
  Serial.println("ROAR");

}

ISR (PCINT0_vect) // handle pin change interrupt for D0 to D7 here
{
  PCMSK0 = 0; //disable the interrupts by masking it off.
  sleeping = 0;
}

byte getBtnst() {
  byte retval=0;
  retval+=digitalRead(BUTTON_1); //pin1
  retval+=digitalRead(BUTTON_2)<<1; //pin2
  retval+=digitalRead(BUTTON_3)<<2; //pin3
  retval+=digitalRead(BUTTON_4)<<3; //pin4
  retval+=digitalRead(BUTTON_5)<<4; //pin5
  retval+=digitalRead(BUTTON_6)<<5; //pin6
  return (~retval) & 0x3F;
}



void loop() {
  //btnst=(~PINB)&0x0F;
  byte btnst = getBtnst();
  if (btnst) {
    byte vers=1;
    Serial.println(btnst);
    if (btnst == 1 ) {
      preparePayload(0);
    } else if (btnst == 2) {
      preparePayload(4);
    } else if (btnst == 4) {
      preparePayload(1);
    } else if (btnst == 8) {
      preparePayload(5);
    } else if (btnst ==16){
      preparePayload(8);
    } else if (btnst ==32){
      preparePayload(0);
    }
    doTransmit(10,TXLength,vers);
  }
  delay(50);
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

void preparePayload(byte cmd) {
  txrxbuffer[0] = commands[cmd][0];
  txrxbuffer[1] = commands[cmd][1];
  txrxbuffer[2] = commands[cmd][2];
  txrxbuffer[3] = commands[cmd][3]<<4;
  TXLength = 4;
}



void doTransmit(int rep, byte len, byte vers) { //rep is the number of repetitions
  Serial.println("Starting transmit");
  
  byte txchecksum = 0;
    byte txchecksum2 = 0;
  for (byte i = 0; i < len - 1; i++) {
    txchecksum = txchecksum ^ txrxbuffer[i];
      txchecksum2 = _crc8_ccitt_update(txchecksum2,txrxbuffer[i]);
  }
  if (len == 4) {
    txchecksum = (txchecksum & 0x0F) ^ (txchecksum >> 4) ^ ((txrxbuffer[3] & 0xF0) >> 4);
    
      txchecksum2 = (txchecksum2 & 0x0F) ^ (txchecksum2 >> 4) ^ ((txrxbuffer[3] & 0xF0) >> 4);
      if (txchecksum == txchecksum2)txchecksum2++;
      txrxbuffer[3] = (txrxbuffer[3] & 0xF0) | 0x0F & (vers == 1 ? txchecksum : txchecksum2);
  } else {
      if (txchecksum == txchecksum2)txchecksum2++;
      txrxbuffer[len - 1] = (vers == 1 ? txchecksum : txchecksum2);
  }
  for (byte r = 0; r < rep; r++) {
    
    for (byte j = 0; j < 2 * txTrainRep; j++) {
      delayMicroseconds(txTrainLen);
      digitalWrite(txpin, j & 1);
      txPIN = txBV;
    }
    txPORT|=txBV;
    delayMicroseconds(txSyncTime);
    txPIN = txBV;
    delayMicroseconds(txSyncTime);
    for (byte k = 0; k < TXLength; k++) {
      //send a byte
            for (int m = 7; m >= 0; m--) {
        txPIN = txBV;
        if ((txrxbuffer[k] >> m) & 1) {
          delayMicroseconds(txOneLength);
        } else {
          delayMicroseconds(txZeroLength);
        }
        txPIN = txBV;
        /*
        if ((txrxbuffer[k] >> m) & 1) {
          delayMicroseconds(txOneLength);
        } else {
          delayMicroseconds(txZeroLength);
        }
       */ 
        delayMicroseconds(txLowTime);
            }
      //done with that byte
    }
    //done with sending this packet;
    //digitalWrite(txpin, 1); //make sure it's off;
    //interrupts();
    delayMicroseconds(rep>5?2500:5000); //wait 2.5ms before doing the next round.
    digitalWrite(txpin,0);
  }
  TXLength = 0;
}

#ifndef OLD_PINOUT
#error "Wrong pinout selected!"
#endif
