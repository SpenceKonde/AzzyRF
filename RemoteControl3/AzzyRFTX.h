#include <util/crc16.h>
#include "Commands.h"
#include "AzzyRFPins.h"
#define RX_MAX_LEN 256

unsigned char txrxbuffer[RX_MAX_LEN >> 3];

byte TXLength = 0;
void doTransmit(int, byte, byte);
void sendPacket(byte,byte);

unsigned int txOneLength  = 525;
unsigned int txZeroLength  = 300;
unsigned int txLowTime  = 300;
unsigned int txSyncTime  = 2000;
unsigned int txTrainLen  = 200;
byte txTrainRep  = 20;


unsigned int txRepDelay = 2000; //delay between consecutive transmissions
byte txRepCount = 5; //number of times to repeat each transmission

void setupAzzyRFTX() {
  pinMode(txpin,OUTPUT);
}

void sendPacket(byte cmd, byte vers) {
  #ifndef MEGATINYCORE
  txrxbuffer[0]=pgm_read_byte_near(&commands[cmd][0]);
  #else
  txrxbuffer[0]=commands[cmd][0];
  #endif
  byte plen = txrxbuffer[0] >> 6;
  plen = 4 << plen;
  for (byte i=1;i<plen;i++){
    #ifndef MEGATINYCORE
    txrxbuffer[i] = pgm_read_byte_near(&commands[cmd][i]);
    #else
    txrxbuffer[i]=commands[cmd][i];
    #endif
  }
  TXLength = plen;
  doTransmit(10,TXLength,vers);
}


void doTransmit(int rep, byte len, byte vers) { //rep is the number of repetitions
  
  
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
