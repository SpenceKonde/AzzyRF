#include "AzzyRFTX.h"
#include "AzzyRFRX.h"

#define SERIALBRIDGE_VERSION "2.0"

unsigned long lastSer = 0;
char * pEnd; //dummy pointer for strtol


unsigned long lastSendSig = 0;

byte MyAddress = 0;


//#define SERIAL_DBG Serial

//Buffers
byte recvMessage[32];
#define MAX_SER_LEN 10
char serBuffer[MAX_SER_LEN];

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
    SERIAL_CMD.println(F(SERIALBRIDGE_VERSION));
    SERIAL_CMD.println(F(AZZYRFTX_VERSION));
    SERIAL_CMD.println(F(AZZYRFRX_VERSION));
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





