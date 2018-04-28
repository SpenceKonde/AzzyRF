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


//Microcontroller-specific
//328p

#define RX_PIN_STATE (PINB&1) //RX on pin 8 for input capture. 
#define TX_PIN 7
#define txPIN PIND
#define txBV 128
#define SERIAL_CMD Serial
//#define SERIAL_DBG Serial

//Configuration

byte MyAddress = 0;


//#define SERIAL_DBG Serial

//Buffers
volatile byte rxBuffer[32];
byte txBuffer[32];
byte recvMessage[32];
#define MAX_SER_LEN 10
char serBuffer[MAX_SER_LEN];

// ##############
// TIMING VALUES
// ##############

// Version 2.2/2.3
#if(F_CPU==8000000)
#define TIME_MULT * 1
#elif(F_CPU==16000000)
#define TIME_MULT * 2
#endif

const unsigned int rxSyncMin  = 1750 TIME_MULT;
const unsigned int rxSyncMax  = 2250 TIME_MULT;
const unsigned int rxZeroMin  = 100 TIME_MULT;
const unsigned int rxZeroMax  = 390 TIME_MULT;
const unsigned int rxOneMin  = 410 TIME_MULT;
const unsigned int rxOneMax  = 700 TIME_MULT;
const unsigned int rxLowMax  = 600 TIME_MULT;
const unsigned int txOneLength  = 500;
const unsigned int txZeroLength  = 300;
const unsigned int txSyncTime  = 2000;
const unsigned int txTrainLen  = 200;
const unsigned int txRepCount = 12;
const unsigned int txRepDelay = 2000;
const byte txTrainRep  = 30;
const int commandForgetTime = 5000;
const char endMarker = '\r';
const char endMarker2 = '\n';
byte SendVersion=1;

// Commands:

const char ATSEND[] PROGMEM = {"AT+SEND"};
const char ATSENDM[] PROGMEM = {"AT+SENDM"};
const char ATSENDL[] PROGMEM = {"AT+SENDL"};
const char ATSENDE[] PROGMEM = {"AT+SENDE"};
const char ATSENDV1[] PROGMEM = {"AT+SENDV1"};
const char ATSENDV2[] PROGMEM = {"AT+SENDV2"};
const char ATSENDVQ[] PROGMEM = {"AT+SENDV?"};
const char ATADRQ[] PROGMEM = {"AT+ADR?"};
const char ATADR[] PROGMEM = {"AT+ADR"};
const char ATVERS[] PROGMEM = {"AT+VERS"};



void setup() {
  // put your setup code here, to run once:
  pinMode(TX_PIN, OUTPUT);
  TCCR1A = 0;
  TCCR1B = 0;
  TIFR1 = bit (ICF1) | bit (TOV1);  // clear flags so we don't get a bogus interrupt
  TCNT1 = 0;          // Counter to zero
  TIMSK1 = 1 << ICIE1; // interrupt on Timer 1 input capture
  // start Timer 1, prescalar of 8, edge select on falling edge
  TCCR1B =  ((F_CPU == 1000000L) ? (1 << CS10) : (1 << CS11)) | 1 << ICNC1; //prescalar 8 except at 1mhz, where we use prescalar of 1, noise cancler active
  //ready to rock and roll
  SERIAL_CMD.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  byte rlen = handleReceive();
  if (rlen) {
    outputPacket(rlen);
  }
  processSerial();
}

void processSerial() {
  static char minb[3] = {0, 0, 0};
  static byte ndx = 0;
  static byte SerRXidx;
  static byte SerRXmax;
  while (SERIAL_CMD.available() > 0) {
    char c=SERIAL_CMD.read();
    if (SerRXmax) {
      if (ndx == 1) {
        minb[1] = c;
        ndx = 0;
        byte t = strtol(minb, &pEnd, 16);
        if (SerRXmax == 1) {
          if (t<32){
            MyAddress = t;
            SERIAL_CMD.println(F("OK"));
          } else {
            SERIAL_CMD.println(F("ERROR"));
          }
          ndx = 0;
          SerRXidx = 0;
          SerRXmax = 0;
          resetSer();
        } else {
          SerRXidx++;
          txBuffer[SerRXidx] = t;
        }
      } else {
        minb[0] = c;
        ndx = 1;
      }
      if (SerRXidx >= SerRXmax && SerRXmax) {
        ndx = 0;
        
        byte len = preparePayloadFromSerial(SerRXmax);
        while (!doTransmit(len, SendVersion));
        SERIAL_CMD.println(F("OK"));
        SerRXidx=0;
        SerRXmax=0;
        resetSer();
      }
    } else {
      
      if (c != endMarker && c != endMarker2) {
        serBuffer[SerRXidx] = c;
        SerRXidx++;
        if (SerRXidx >= MAX_SER_LEN) {
          SerRXidx = 0;
    SerRXmax = 0;
          SERIAL_CMD.println(F("ERROR"));
          resetSer();
        }
      } else {
        if (SerRXidx) { //index 0? means it's a \r\n pattern.
          serBuffer[SerRXidx] = '\0'; // terminate the string
          SerRXidx = 0;
          SerRXmax = checkCommand();
          if (SerRXmax==255 ) {//error condition
            SerRXmax=0;
            SerRXidx=0;
            SERIAL_CMD.println(F("ERROR"));
            resetSer();
          }
        }
      }
    }
    if (c != endMarker && c != endMarker2){
      lastSer = millis();
    }
  }
  if (millis() - lastSer > 10000 && lastSer) {
    SERIAL_CMD.println(F("ERROR"));
    SerRXidx = 0;
    SerRXmax = 0;
    resetSer();
  }
}

byte preparePayloadFromSerial(byte rcvlen) {
  txBuffer[0] = (txBuffer[0] & 0x3F);
  if (rcvlen > 4) {
    txBuffer[0] = txBuffer[0] | (rcvlen == 7 ? 0x40 : (rcvlen == 15 ? 0x80 : 0xC0));
    return rcvlen + 1;
  } else {
    txBuffer[3] = txBuffer[3] << 4;
    return 4;
  }
}

byte checkCommand() {
  byte paramlen = 0;
  if (strcmp_P (serBuffer, ATSEND) == 0) {
    paramlen = 4;
  } else if (strcmp_P (serBuffer, ATSENDM) == 0) {
    paramlen = 7;
  } else if (strcmp_P (serBuffer, ATSENDL) == 0) {
    paramlen = 15;
  } else if (strcmp_P (serBuffer, ATSENDE) == 0) {
    paramlen = 31;
  } else if (strcmp_P (serBuffer, ATADRQ) == 0) { //AT+ADR?
    SERIAL_CMD.println(MyAddress);
  } else if (strcmp_P (serBuffer, ATSENDVQ) == 0) {
    SERIAL_CMD.println(SendVersion);
  } else if (strcmp_P (serBuffer, ATSENDV1) == 0) {
    SendVersion=1;
  } else if (strcmp_P (serBuffer, ATSENDV2) == 0) {
    SendVersion=2;
  } else if (strcmp_P (serBuffer, ATADR) == 0) {
    paramlen = 1;
  } else if (strcmp_P (serBuffer, ATVERS) == 0) {
    SERIAL_CMD.println(F("AzzyRF v2.3"));
  } else {
    return 255;
  }
  if (paramlen) {
    SERIAL_CMD.print('>');
  }
  return paramlen;
}
void resetSer() {
  if (lastSer) { //if we've gotten any characters since last reset, print newline to signify completion.
    lastSer = 0;
    SERIAL_CMD.println();
  }
  memset(serBuffer, 0, MAX_SER_LEN);
}

void outputPacket(byte rlen) { //format of rlen: what is passed back from handleReceive(), first 2 bits are version-1, last 6 are length in bytes
  
  byte vers = (rlen & 196) >> 6;
  rlen &= 0x3F;
  if (vers == 0) { //version 1
    SERIAL_CMD.print('+');
  } else {
    SERIAL_CMD.print('=');
  }
  for (byte i = 0; i < (rlen-1); i++) {
    showHex(recvMessage[i], 1);
  }
  if (rlen == 4) {
      showHex(recvMessage[3] >> 4, 1);
  }
  SERIAL_CMD.println();
}


void showHex (const byte b, const byte c) {
  // try to avoid using sprintf
  char buf [3] = { ((b >> 4) & 0x0F) | '0', (b & 0x0F) | '0', 0};
  if (buf [0] > '9')
    buf [0] += 7;
  if (buf [1] > '9')
    buf [1] += 7;

  if (c) {
    SERIAL_CMD.print(buf);
  } else {
#ifdef SERIAL_DBG
    SERIAL_DBG.print(buf);
#endif
  }
}

byte handleReceive() {
  if (gotMessage) {
    byte vers = checkCSC(); //checkCSC() gives 0 on failed CSC, 1 on v1 structure (ACD...), 2 on v2 structure (DSCD...)
    if (!vers) { //if vers=0, unknown format ot bad CSC
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
  TIMSK1 = 1 << ICIE1;
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

ISR (TIMER1_CAPT_vect)
{
  static unsigned long lasttime = 0;
  unsigned int newTime = ICR1; //immediately get the ICR value
  byte state = (RX_PIN_STATE);
  TCCR1B = state ? (1 << CS11 | 1 << ICNC1) : (1 << CS11 | 1 << ICNC1 | 1 << ICES1); //and set edge
  unsigned int duration = newTime - lasttime;
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
        TIMSK1 = 0; //turn off input capture;

      } else {
        bitnum++;
      }
    }
  }
}



byte doTransmit(byte len, byte vers) {
  if (!(receiving || lastPacketTime)) {
    TIMSK1 = 0;
#ifdef LED_TX
    digitalWrite(LED_TX, TX_LED_ON);
#endif
    digitalWrite(TX_PIN, 0); // known state
    byte txchecksum = 0;
    byte txchecksum2 = 0;
    for (byte i = 0; i < len - 1; i++) {
      txchecksum = txchecksum ^ txBuffer[i];
      txchecksum2 = _crc8_ccitt_update(txchecksum2,txBuffer[i]);
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
      digitalWrite(TX_PIN, 1);
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
      digitalWrite(TX_PIN, 0); //make sure it's off;
      delayMicroseconds(txRepDelay); //wait before doing the next round.
    }
#ifdef LED_TX
    digitalWrite(LED_TX, TX_LED_OFF);
#endif
    TIMSK1 = 1 << ICIE1;
    return 1;
  } else {
    return 0;
  }
}
