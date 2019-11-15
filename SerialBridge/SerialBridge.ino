#include <util/crc16.h>

unsigned long lastSer = 0;
char * pEnd; //dummy pointer for strtol


unsigned long lastSendSig = 0;

//if it's a megaavr, figure out which timer we use:


//Microcontroller-specific
//328p

#ifdef __AVR_ATmega328P__

#define RX_PIN_STATE (PINB&1) //RX on pin 8 for input capture. 
#define TX_PIN 7
#define txPIN PIND
#define txBV 128
#define SERIAL_CMD Serial
//#define SERIAL_DBG Serial

#endif

#if defined(__AVR_ATtinyx16__) || defined(__AVR_ATtinyx06__)
#define RX_PIN_STATE (VPORTA.IN&2) //RX on pin A1 for input capture.  pin 14
#define RX_ASYNC0 0x0B
#define TX_PIN 3
#define txPIN VPORTA.IN
#define txBV 128
#define SERIAL_CMD Serial
#endif

#if defined(__AVR_ATtinyx14__) || defined(__AVR_ATtinyx04__)
#define RX_PIN_STATE (VPORTA.IN&8) //RX on pin A3 for input capture.  pin 10
#define RX_ASYNC0 0x0D
#define TX_PIN 0
#define txPIN VPORTA.IN
#define txBV 16
#define SERIAL_CMD Serial
#endif

//Configuration

byte MyAddress = 0;


//#define SERIAL_DBG Serial

//Buffers
byte txBuffer[32];
byte recvMessage[32];
#define MAX_SER_LEN 10
char serBuffer[MAX_SER_LEN];


const unsigned int txOneLength  = 500;
const unsigned int txZeroLength  = 300;
const unsigned int txSyncTime  = 2000;
const unsigned int txTrainLen  = 200;
const unsigned int txRepCount = 12;
const unsigned int txRepDelay = 2000;
const byte txTrainRep  = 30;
const char endMarker = '\r';
const char endMarker2 = '\n';
byte SendVersion = 1;

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
  setupAzzyRFRX();
  setupAzzyRFTX();
  SERIAL_CMD.begin(115200);
  delay(1000);
  SERIAL_CMD.println("OK");
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
    char c = SERIAL_CMD.read();
    if (SerRXmax) {
      if (c != endMarker && c != endMarker2) {
        if (ndx == 1) {
          minb[1] = c;
          ndx = 0;
          byte t = strtol(minb, &pEnd, 16);
          if (SerRXmax == 1) {
            if (t < 32) {
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
            //SerRXidx++;
            txBuffer[SerRXidx++] = t;
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
          SerRXidx = 0;
          SerRXmax = 0;
          resetSer();
        }
      }
    } else {

      if (c != endMarker && c != endMarker2) {
        serBuffer[SerRXidx] = c;
        SerRXidx++;
        if (SerRXidx >= MAX_SER_LEN) {
          SerRXidx = 0;
          SerRXmax = 0;
          //SERIAL_CMD.println(F("ERROR 1"));
          resetSer();
        }
      } else {
        if (SerRXidx) { //index 0? means it's a \r\n pattern.
          serBuffer[SerRXidx] = '\0'; // terminate the string
          SerRXidx = 0;
          SerRXmax = checkCommand();
          if (SerRXmax == 255 ) { //error condition
            SerRXmax = 0;
            SerRXidx = 0;
            //SERIAL_CMD.println(F("ERROR 2"));
            //SERIAL_CMD.println(serBuffer);
            resetSer();
          }
        }
      }
    }
    if (c != endMarker && c != endMarker2) {
      lastSer = millis();
    }
  }
  if (millis() - lastSer > 10000 && lastSer) {
    SerRXidx = 0;
    SerRXmax = 0;
    resetSer();
  }
}

byte preparePayloadFromSerial(byte rcvlen) {
  Serial.println(txBuffer[0], HEX);
  Serial.println(txBuffer[1], HEX);
  Serial.println(txBuffer[2], HEX);
  Serial.println(txBuffer[3], HEX);
  Serial.println(txBuffer[4], HEX);
  Serial.println(txBuffer[5], HEX);
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
    SendVersion = 1;
  } else if (strcmp_P (serBuffer, ATSENDV2) == 0) {
    SendVersion = 2;
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
  for (byte i = 0; i < (rlen - 1); i++) {
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
