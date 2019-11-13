#include "AzzyRFRXPins.h"
#include <util/crc16.h>


volatile byte receiving = 0;
volatile byte bitnum = 0; //current bit received

volatile byte gotMessage = 0;
volatile byte dataIn = 0;
volatile byte pktLength = 31;
unsigned long lastPacketTime = 0;
unsigned long lastPacketSig = 0;
unsigned long lastSendSig = 0;
unsigned long lastSer = 0;
char * pEnd; //dummy pointer for strtol


// ##############
// TIMING VALUES
// ##############

// Version 2.2/2.3
#if defined(TCB1) || defined(TCB0) //means it's a megaavr
#if(F_CPU==8000000)
#define TIME_MULT * 4
#elif(F_CPU==10000000)
#define TIME_MULT * 5
#elif(F_CPU==16000000)
#define TIME_MULT * 8
#elif(F_CPU==20000000)
#define TIME_MULT * 10
#else 
#error "Unsupported clock speed"
#endif
#else
#if(F_CPU==8000000)
#define TIME_MULT * 1
#elif(F_CPU==16000000)
#define TIME_MULT * 2
#elif(F_CPU==12000000)
#define TIME_MULT * 3/2
#else 
#error "Unsupported clock speed"
#endif
#endif


const unsigned int rxSyncMin  = (1750 TIME_MULT);
const unsigned int rxSyncMax  = (2250 TIME_MULT);
const unsigned int rxZeroMin  = (100 TIME_MULT);
const unsigned int rxZeroMax  = (390 TIME_MULT);
const unsigned int rxOneMin  = (410 TIME_MULT);
const unsigned int rxOneMax  = (700 TIME_MULT);
const unsigned int rxLowMax  = (600 TIME_MULT);
const int commandForgetTime = 5000;



//Buffers
volatile byte rxBuffer[32];
byte recvMessage[32];

byte handleReceive() {
  if (gotMessage) {
    byte vers = checkCSC(); //checkCSC() gives 0 on failed CSC, 1 on v1 structure (ACD...), 2 on v2 structure (DSCD...)
    if (!vers) { //if vers=0, unknown format ot bad CSC
      resetReceive();
      return 0;
    }
    if (rxBuffer[0]==0 &&  rxBuffer[1]==0 && rxBuffer[2]==0 && (rxBuffer[3]&0xF0)==0) {
      resetReceive();
      return 0;
    }
    if (!isForMe()) { //matches on MyAddress==0, destination address==0, destination address==MyAddress.
      resetReceive();
      return 0;
    }
    if (lastPacketSig == getPacketSig() && lastPacketTime) {
      
    lastPacketTime = millis();
      resetReceive();
      return 0;
    }
    lastPacketSig = getPacketSig();
    lastPacketTime = millis();
    byte rlen = ((pktLength >> 3) + 1) | ((vers - 1) << 6);
    memcpy(recvMessage, rxBuffer, 32);
    if (rlen==4){
      recvMessage[3]=recvMessage[3]&0xF0;
    } else {
      recvMessage[rlen-1]=0;
    }
    resetReceive();
    return rlen;
  } else {
    unsigned long t=(millis()-lastPacketTime);
    if (lastPacketTime && (t > commandForgetTime)) {
      lastPacketTime = 0;
      lastPacketSig = 0;
    }
    return 0;
  }
}

void resetReceive() {

  bitnum = 0;
  memset(rxBuffer, 0, 32);
  gotMessage = 0;
  #ifdef TCB1
  TCB1.INTCTRL=0x01;
  #elif defined(TCB0)
  TCB0.INTCTRL=0x01;
  #else
  TIMSK1 = 1 << ICIE1;
  #endif
  return;
}

byte checkCSC() {
  byte rxchecksum = 0;
  byte rxchecksum2 = 0;
  for (byte i = 0; i < pktLength >> 3; i++) {
    rxchecksum = rxchecksum ^ rxBuffer[i];
    rxchecksum2 = _crc8_ccitt_update(rxchecksum2,rxBuffer[i]);
  }
  if (pktLength >> 3 == 3) {
    rxchecksum = (rxchecksum & 0x0F) ^ (rxchecksum >> 4) ^ ((rxBuffer[3] & 0xF0) >> 4);
    rxchecksum2 = (rxchecksum2 & 0x0F) ^ (rxchecksum2 >> 4) ^ ((rxBuffer[3] & 0xF0) >> 4);
    if (rxchecksum == rxchecksum2)rxchecksum2++;
    return (rxBuffer[3] & 0x0F) == rxchecksum ? 1 : ((rxBuffer[3] & 0x0F) == rxchecksum2 ) ? 2 : 0;
  } else {
    if (rxchecksum == rxchecksum2)rxchecksum2++;
    return ((rxBuffer[pktLength >> 3] == rxchecksum) ? 1 : ((rxBuffer[pktLength >> 3] == rxchecksum2 ) ? 2 : 0));
  }
}

byte isForMe() {
  if ((rxBuffer[0] & 0x3F) == MyAddress || MyAddress == 0 || (rxBuffer[0] & 0x3F) == 0) {
    return 1;
  }
  return 0;
}

unsigned long getPacketSig() {
  byte len = pktLength >> 3;
  unsigned long lastpacketsig=0;
  for (byte i = (len == 3 ? 0 : 1); i < (len == 3 ? 3 : 4); i++) {
    lastpacketsig += rxBuffer[i];
    lastpacketsig = lastpacketsig << 8;
  }
  lastpacketsig += rxBuffer[len];
  return lastpacketsig;
}

void setupTimer() {
  #if defined(TCCR1A) && defined(TIMSK1) //In this case, it's a classic AVR with a normal timer1
  TCCR1A = 0;
  TCCR1B = 0;
  TIFR1 = bit (ICF1) | bit (TOV1);  // clear flags so we don't get a bogus interrupt
  TCNT1 = 0;          // Counter to zero
  TIMSK1 = 1 << ICIE1; // interrupt on Timer 1 input capture
  // start Timer 1, prescalar of 8, edge select on falling edge
  TCCR1B =  ((F_CPU == 1000000L) ? (1 << CS10) : (1 << CS11)) | 1 << ICNC1; //prescalar 8 except at 1mhz, where we use prescalar of 1, noise cancler active
  //ready to rock and roll
  #elif defined(TCB1) // it's a megaavr
  TCB1.CTRLA=0x02; //disable, CKPER/2 clock source.
  TCB1.CTRLB=0x03; //Input Capture Frequency Measurement mode
  TCB1.INTFLAGS=1; //clear flag
  TCB1.CNT=0; //count to 0
  TCB1.INTCTRL=0x01;
  EVSYS.ASYNCCH0=RX_ASYNC0 //PA1 Set event channel for PA1 pin
  EVSYS.ASYNCUSER11=0x03;
  TCB1.EVCTRL=0x51; //filter, falling edge, ICIE=1
  TCB1.CTRLA=0x03; //enable
  #elif defined(TCB0) // it's a megaavr
  TCB0.CTRLA=0x02; //disable, CKPER/2 clock source.
  TCB0.CTRLB=0x03; //Input Capture Frequency Measurement mode
  TCB0.INTFLAGS=1; //clear flag
  TCB0.CNT=0; //count to 0
  TCB0.INTCTRL=0x01;
  EVSYS.ASYNCCH0=RX_ASYNC0; //PA1 Set event channel for PA1 pin
  EVSYS.ASYNCUSER0=0x03;
  TCB0.EVCTRL=0x51; //filter, falling edge, ICIE=1
  TCB0.CTRLA=0x03; //enable
  #else
  #error "architecture not supported"
  #endif
}

#ifdef TCB1
ISR(TCB1_INT_vect)
#elif defined(TCB0)
ISR(TCB0_INT_vect)
#else
ISR (TIMER1_CAPT_vect)
#endif
{
  #if defined(TCB1)
  static unsigned long lasttime = 0;
  unsigned int newTime = TCB1.CCMP; //immediately get the ICR value
  #elif defined(TCB0)
  static unsigned long lasttime = 0;
  unsigned int newTime = TCB0.CCMP; //immediately get the ICR value
  #else
  unsigned int newTime = ICR1; //immediately get the ICR value
  #endif
  byte state = (RX_PIN_STATE);
  #ifdef TCB1
  TCB1.EVCTRL=state?0x51:0x41; //trigger on falling edge if pin is high, otherwise rising edge
  unsigned int duration = newTime;
  #elif defined(TCB0)
  TCB0.EVCTRL=state?0x51:0x41; //trigger on falling edge if pin is high, otherwise rising edge
  unsigned int duration = newTime;
  #else
  TCCR1B = state ? (1 << CS11 | 1 << ICNC1) : (1 << CS11 | 1 << ICNC1 | 1 << ICES1); //and set edge
  unsigned int duration = newTime - lasttime;
  #endif
  lasttime = newTime;
  if (state) {
    if (receiving) {
      if (duration > rxLowMax) {
        
        receiving = 0;
        bitnum = 0; // reset to bit zero
        memset(rxBuffer, 0, 32); //clear buffer
      }
    } else {
      if (duration > rxSyncMin && duration < rxSyncMax) {
        receiving = 1;
      }
    }
  } else {
    if (receiving) {
      if (duration > rxZeroMin && duration < rxZeroMax) {
        dataIn = dataIn << 1;
      } else if (duration > rxOneMin && duration < rxOneMax) {
        dataIn = (dataIn << 1) + 1;
      } else {
        receiving = 0;
        bitnum = 0; // reset to bit zero
        memset(rxBuffer, 0, 32); //clear buffer
        return;
      }
      if ((bitnum & 7) == 7) {
        rxBuffer[bitnum >> 3] = dataIn;
        if (bitnum == 7) {
          byte t = dataIn >> 6;
          pktLength = t ? (t == 1 ? 63 : (t == 2 ? 127 : 255)) : 31;
        }
        dataIn = 0;
      }
      if (bitnum >= pktLength) {
        bitnum = 0;
        receiving = 0;
        gotMessage = 1;
        #ifdef TCB1
        TCB1.INTCTRL=0x00;
        #elif defined(TCB0)
        TCB0.INTCTRL=0x00;
        #else
        TIMSK1 = 0; //turn off input capture;
        #endif
      } else {
        bitnum++;
      }
    }
  }
}
