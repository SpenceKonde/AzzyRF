#include "AzzyRFTXPins.h"
#include <util/crc16.h>

const unsigned int txOneLength  = 500;
const unsigned int txZeroLength  = 300;
const unsigned int txSyncTime  = 2000;
const unsigned int txTrainLen  = 200;
const unsigned int txRepCount = 12;
const unsigned int txRepDelay = 2000;
const byte txTrainRep  = 30;


byte txBuffer[32];

void setupAzzyRFTX() {
  pinMode(TX_PIN,OUTPUT);
}

byte doTransmit(byte len, byte vers) {
  if (!(receiving || lastPacketTime)) {
#ifdef TCB1
    TCB1.INTCTRL = 0x00;
#elif defined(TCB0)
    TCB0.INTCTRL = 0x00;
#else
    TIMSK1 = 0;
#endif
#ifdef LED_TX
    digitalWrite(LED_TX, TX_LED_ON);
#endif
    digitalWrite(TX_PIN, LOW); // known state
    byte txchecksum = 0;
    byte txchecksum2 = 0;
    for (byte i = 0; i < len - 1; i++) {
      txchecksum = txchecksum ^ txBuffer[i];
      txchecksum2 = _crc8_ccitt_update(txchecksum2, txBuffer[i]);
    }
    if (len == 4) {
      txchecksum = (txchecksum & 0x0F) ^ (txchecksum >> 4) ^ ((txBuffer[3] & 0xF0) >> 4);
      txchecksum2 = (txchecksum2 & 0x0F) ^ (txchecksum2 >> 4) ^ ((txBuffer[3] & 0xF0) >> 4);
      if (txchecksum == txchecksum2)txchecksum2++;
      txBuffer[3] = (txBuffer[3] & 0xF0) | 0x0F & (vers == 1 ? txchecksum : txchecksum2);
    } else {
      if (txchecksum == txchecksum2)txchecksum2++;
      txBuffer[len - 1] = (vers == 1 ? txchecksum : txchecksum2);
    }

    for (byte r = 0; r < txRepCount; r++) {
      for (byte j = 0; j <= 2 * txTrainRep; j++) {
        delayMicroseconds(txTrainLen);
        //digitalWrite(txpin, j & 1);
        txPIN = txBV;
      }
      digitalWrite(TX_PIN, HIGH);
      delayMicroseconds(txSyncTime);
      txPIN = txBV;
      delayMicroseconds(txSyncTime);
      for (byte k = 0; k < len; k++) {
        //send a byte
        for (int m = 7; m >= 0; m--) {
          txPIN = txBV;
          if ((txBuffer[k] >> m) & 1) {
            delayMicroseconds(txOneLength);
            txPIN = txBV;
            delayMicroseconds(txOneLength);
          } else {
            delayMicroseconds(txZeroLength);
            txPIN = txBV;
            delayMicroseconds(txZeroLength);
          }
        }
        //done with that byte
      }
      //done with sending this packet;
      digitalWrite(TX_PIN, LOW); //make sure it's off;
      delayMicroseconds(txRepDelay); //wait before doing the next round.
    }
#ifdef LED_TX
    digitalWrite(LED_TX, TX_LED_OFF);
#endif
#ifdef TCB1
    TCB1.INTCTRL = 0x01;
#elif defined(TCB0)
    TCB0.INTCTRL = 0x01;
#else
    TIMSK1 = 1 << ICIE1;
#endif
    return 1;
  } else {
    return 0;
  }
}
