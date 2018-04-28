/*
This sketch continually listens to a 433 or 315mhz receiver connected to rxpin.
It listens for 4, 8, 16, or 32 byte packets, sent 2ms after the training burst with 0.65ms low between bits, 1.1ms high for 1, 0.6ms for 0.




Description of packet format:

The first byte of packets is:

| SZ1 | SZ0 | A5 | A4 | A3 | A2 | A1 | A0 |

A5~0: Address bits - 6 bit address of the device, compared to MyAddress. Only packets matching address are processed.

SZ1~SZ0: Size setting

SZ1=0, SZ0=0: 4 bytes
SZ1=0, SZ0=1: 8 bytes
SZ1=1, SZ0=0: 16 bytes
SZ1=1, SZ0=1: 32 bytes

The next two bytes go into MyCmd and MyParam, respectively, if the transmission is accepted (ie, device address matches and checksum is OK)

In a 4 byte packet, the first 4 bits of the last byte goes into MyExtParam (extended parameter), and the last 4 bits are the checksum.

For all longer packets, the whole fourth byte goes into MyExtParam, and the final byte is the checksum.

For 4 byte packets, checksum is calculated by XORing the first three bytes, then XORing the first 4 bits of the result with the last 4 bits, and the first 4 bits of the fourth byte.

For longer packets, checksum is calculated by XORing all bytes of the packet.


How to extend this:


Most of the heavy lifting has been done with regards to receiving, transmitting, and ignoring repeated commands. Ideally, you should only have to edit onCommandST() to handle your new commands, and loop() if you want to add new states, poll sensors, etc.

Receiving:

On receiving a valid packet, MyState will be set to CommandST, and MyCmd, MyParam, and MyExtParam will be populated.

Add a test for the new command to onCommandST() that calls code to handle your new command.
If you're receiving a long packet, you can get bytes beyond the first 4 from txrxbuffer[] (these are not stored as a separate variable in order to save on SRAM)


Transmitting:

Populate txrxbuffer with the bytes you want to send (bytes beyond that are ignored), including the size/address byte.
set TXLength to the length of the packet you are sending. Do not set up the checksum - this is done by doTransmit();

Call doTransmit(rep) to send it, where rep is the number of times you want to repeat the transmission to ensure that it arrives. This blocks until it finishes sending.


The example commands are:

0xF2 - Respond with a 4-byte packet containing the values at 2 locations on the EEPROM, at the addresses (MyParam) and (MyParam+MyExtParam). The first byte is the address of this device, and the first half of the fourth byte is the checksum of the received command (likely not useful here).

0xF4 - Respond with a packet (MyParam) bytes long, repeated (MyExtParam) times. For testing reception. This calculates the checksum, but doesn't set the size and address byte to anything meaningful.

*/

#define ListenST 1
#define CommandST 2
#include <EEPROM.h>
#include <TinyWireM.h>

//Pin definitions:

//pins 0,1: Serial
//pins 15,16: Serial1 (programming only)
//pins 16,12: I2C

#define LED1 13
#define LED2 11
#define LED3 4
#define LED4 5
#define LED5 6
#define LED6 8
//#define BTN0 3
//#define BTN4 5
//#define BTN3 6
//#define BTN2 11
//#define BTN1 8

#define LED_RX LED1
//#define LED_TX LED3
#define LED_START LED1
//#define BTN_ACK BTN1

//#define SHUT_PIN 8


#define LED_ON 0
#define LED_OFF 1

#define SerialCmd Serial
#define SerialDbg Serial1
#define MAX_SER_LEN 10
char serBuffer[MAX_SER_LEN];


#define HEX_OUT
//#define HEX_IN
#define USE_ACK
//#define AUTO_ACK



#define rxpin 7
#define rxPIN PINA
#define rxBV 2

#define txpin 14
#define txPIN PINB
#define txBV 8


#define CommandForgetTime 2000 //short, for testing

#define RX_MAX_LEN 256 //Used to set the size of txrx buffer in BITS (and checked against this to prevent overflows from messing stuff up)

#define EEPROM_OFFSET_CONF 32
#define EEPROM_LENGTH_CONF 28

const char AT24R[] PROGMEM = {"AT+24R"};
const char ATSEND[] PROGMEM = {"AT+SEND"};
const char ATSENDM[] PROGMEM = {"AT+SENDM"};
const char ATSENDL[] PROGMEM = {"AT+SENDL"};
const char ATSENDE[] PROGMEM = {"AT+SENDE"};
const char ATCONF[] PROGMEM = {"AT+CONF"};
const char ATHEX[] PROGMEM = {"AT+HEX?"};
const char ATADRQ[] PROGMEM = {"AT+ADR?"};
const char ATADR[] PROGMEM = {"AT+ADR"};
const char AT24W[] PROGMEM = {"AT+24W"};
const char AT24WL[] PROGMEM = {"AT+24WL"};
const char AT24RL[] PROGMEM = {"AT+24RL"};
const char ATVERS[] PROGMEM = {"AT+VERS?"};
#ifdef USE_ACK
const char ATACK[] PROGMEM = {"AT+ACK"};
#endif


//These set the parameters for transmitting.

/*
#define txOneLength 550 //length of a 1
#define txZeroLength 300 //length of a 0
#define txLowTime 420 //length of the gap between bits
#define txTrainRep 30 //number of pulses in training burst
#define txSyncTime 2000 //length of sync
#define txTrainLen 200 //length of each pulse in training burst



//These set the parameters for receiving; any packet where these criteria are not met is discarded.
// Version 2.0
int rxSyncMin=1900; //minimum valid sync length
int rxSyncMax=2100; //maximum valid sync length
int rxZeroMin=100; //minimum length for a valid 0
int rxZeroMax=300; //maximum length for a valid 0
int rxOneMin=400; //minimum length for a valid 1
int rxOneMax=600; //maximum length for a valid 1
int rxLowMax=450; //longest low before packet discarded



// Version 2.1
#define rxSyncMin 1900 //minimum valid sync length
#define rxSyncMax 2100 //maximum valid sync length
#define rxZeroMin 120 //minimum length for a valid 0
#define rxZeroMax 400 //maximum length for a valid 0
#define rxOneMin 450 //minimum length for a valid 1
#define rxOneMax 750 //maximum length for a valid 1
#define rxLowMax 600 //longest low before packet discarded
*/
// Version 2.2

unsigned int rxSyncMin  = 1750;
unsigned int rxSyncMax  = 2250;
unsigned int rxZeroMin  = 100;
unsigned int rxZeroMax  = 390;
unsigned int rxOneMin  = 410;
unsigned int rxOneMax  = 700;
unsigned int rxLowMax  = 700;
unsigned int txOneLength  = 500;
unsigned int txZeroLength  = 300;
unsigned int txLowTime  = 400;
unsigned int txSyncTime  = 2000;
unsigned int txTrainLen  = 200;
byte txTrainRep  = 30;

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
byte defaultAT24i2ca = 0x50;


const unsigned long units[] = {1000, 60000, 900000, 14400000, 3600000, 1, 10, 86400000}; //units for the 8/12/16-bit time values.

int highLengths[64];
int lowLengths[64];

byte MyAddress = 0;


//Pin state tracking and data for receiving.
byte lastPinState;
unsigned long lastPinHighTime;
unsigned long lastPinLowTime;
unsigned long lastTempHighTime = 0;
unsigned long lastTempLowTime = 0;
byte rxdata;
byte lastTempPinState;
int bitsrx;
byte rxing;
byte rxaridx;
unsigned char txrxbuffer[RX_MAX_LEN >> 3];
byte SerRXidx;
unsigned long lastSer = 0;
byte SerRXmax;
byte SerCmBuff[16];
byte SerCmd;

unsigned long led3OffAt;
unsigned long led4OffAt;

char * pEnd; //dummy pointer for sto



//byte MyState;
//unsigned char MyCmd;
//unsigned char MyParam;
//unsigned char MyExtParam;
unsigned long curTime;
int count = 0;
int badcsc = 0;
int pksize = 32;
byte TXLength;
unsigned long lastChecksum; //Not the same as the CSC - this is our hack to determine if packets are identical
unsigned long forgetCmdAt;
int reccount;

#ifdef USE_ACK
byte lastCmdSent;
byte lastCscSent;
#endif

void showHex (const byte b, const byte c = 0); //declare this function so it will compile correctly with the optional second argument.

void setup() {
  lastPinState = 0;
  lastPinLowTime = 0;
  lastPinHighTime = 0;
  rxdata = 0;
  bitsrx = 0;
  rxing = 0;
  initializeOutputs();
  resetListen();
  pinMode(txpin, OUTPUT);
  pinMode(rxpin, INPUT);
  SerialDbg.begin(9600);
  if (EEPROM.read(0) < 255) {
    //initFromEEPROM();
    //SerialDbg.println(F("Load from EEPROM"));
  }
  SerialCmd.begin(9600);
  pinMode(1,INPUT_PULLUP);
  digitalWrite(LED_START, LED_ON);
  TinyWireM.begin();
  delay(1000);
  digitalWrite(LED_START, LED_OFF);
  SerialDbg.println(F("Startup OK"));
  //SerialDbg.print(decode8(123));


}


void loop() {
  //if (MyState == ListenST) {
  onListenST();
  if (rxing == 1) {
    PORTC &= ~1;
    return; //don't do anything else while actively receiving.
  } else {
    PORTC |= 1;
#ifdef LED_SER
    digitalWrite(LED_SER, rxing >> 1 ? LED_ON : LED_OFF);
#endif
    ClearCMD(); //do the command reset only if we are in listenst but NOT receiving.
    processSerial();
    if (lastSer & (millis() - lastSer  > (rxing == 2 ? 20000 : 10000))) {
      resetSer();
    }

    //if (digitalRead(BTN0) == 0 && led4OffAt == 0) {
      //digitalWrite(LED4, LED_ON);
      //led4OffAt = millis() + 1000;
      //SerialCmd.println(F("BTN1"));
      //prepareTestPayload();
      //doTransmit();

//    }
  }
  //} else if (MyState == CommandST) {
  //  onCommandST();
  //} else {
  //  MyState = ListenST; //in case we get into a bad state somehow.
  //}
}

void writeConfigToEEPROM() {
  for (byte i = 0; i < EEPROM_LENGTH_CONF; i++) {
    EEPROM.write(i + EEPROM_OFFSET_CONF, txrxbuffer[i]);
  }
  initFromEEPROM();
}



void initFromEEPROM() {
  byte tAddr = EEPROM.read(0);
  if (tAddr > 127) {
    MyAddress = tAddr & 0x3F;
    if (tAddr & 0x40) {
      if (EEPROM.read(1) < 255) {
#ifdef OSCCAL
        OSCCAL = EEPROM.read(3);
#else
        OSCCAL0 = EEPROM.read(3);
#endif
        delay(50); //let's be cautious;
      }
    }

    byte tIsConf = EEPROM.read(32);
    if (tIsConf < 255) {
      SerialDbg.println(F("Loading config"));
      //MyAddress = tAddr;
      txSyncTime = EEPROM.read(33) + (EEPROM.read(32) << 8); //length of sync
      txTrainRep = EEPROM.read(34); //number of pulses in training burst
      txTrainLen = EEPROM.read(36) + (EEPROM.read(35) << 8); //length of each pulse in training burst
      txOneLength = EEPROM.read(38) + (EEPROM.read(37) << 8); //length of a 1
      txZeroLength = EEPROM.read(40) + (EEPROM.read(39) << 8); //length of a 0
      txLowTime = EEPROM.read(42) + (EEPROM.read(41) << 8); //length of the gap between bits
      rxSyncMin = EEPROM.read(44) + (EEPROM.read(43) << 8); //minimum valid sync length
      rxSyncMax = EEPROM.read(46) + (EEPROM.read(45) << 8); //maximum valid sync length
      rxZeroMin = EEPROM.read(48) + (EEPROM.read(47) << 8); //minimum length for a valid 0
      rxZeroMax = EEPROM.read(50) + (EEPROM.read(49) << 8); //maximum length for a valid 0
      rxOneMin = EEPROM.read(52) + (EEPROM.read(51) << 8); //minimum length for a valid 1
      rxOneMax = EEPROM.read(54) + (EEPROM.read(53) << 8); //maximum length for a valid 1
      rxLowMax = EEPROM.read(56) + (EEPROM.read(55) << 8); //longest low before packet discarded
      txRepDelay = EEPROM.read(58) + (EEPROM.read(57) << 8); //delay between repetitions of a packet
      txRepCount = EEPROM.read(59); //default number of repetitions.
    } else {
      SerialDbg.println(F("No config to load"));
    }
  }
}

void processSerial() {
  static char minb[3] = {0, 0, 0};
  static byte ndx = 0;
  char endMarker = '\r';
  char endMarker2 = '\n';
  while (SerialCmd.available() > 0) {
  PINA=4;
  
    if (SerRXidx < SerRXmax && rxing == 2) {
#ifdef HEX_IN
      if (ndx == 1) {
        minb[1] = SerialCmd.read();
        ndx = 0;
        txrxbuffer[SerRXidx] = strtol(minb, &pEnd, 16);
        SerRXidx++;
      } else {
        minb[0] = SerialCmd.read();
        ndx = 1;
      }
#else
      txrxbuffer[SerRXidx] = SerialCmd.read();
      SerRXidx++;
#endif

      if (SerRXidx == SerRXmax) {
        ndx = 0;
        if (SerCmd == 0) {
          if (SerRXmax == 28) {
            writeConfigToEEPROM();
          } else {
            preparePayloadFromSerial();
            doTransmit();
          }
        } else if (SerCmd == 1) { //AT+ADR
          MyAddress = txrxbuffer[0];
          //  EEPROM.write(0,MyAddress)
        } else if (SerCmd == 2) { //AT+24R
#ifdef HEX_OUT
          SerialCmd.print(F("="));
          showHex(readAT24(defaultAT24i2ca, txrxbuffer[0] << 8 + txrxbuffer[1]), 1);
#else
          SerialCmd.println(readAT24(defaultAT24i2ca, txrxbuffer[0] << 8 + txrxbuffer[1]));
#endif
        } else if (SerCmd == 3) { //AT+24W
          writeAT24(defaultAT24i2ca, txrxbuffer[0] << 8 + txrxbuffer[1], txrxbuffer[2]);
        } else if (SerCmd == 4) { //AT+24RL
          readAT24(defaultAT24i2ca, (txrxbuffer[0] << 8) + txrxbuffer[1], txrxbuffer[2], txrxbuffer + 3);
          #ifdef HEX_OUT
          SerialCmd.print(F("="));
          #endif
          for (byte i = 0; i < txrxbuffer[2]; i++) {
#ifdef HEX_OUT
            showHex(txrxbuffer[i + 3], 1);
#else
            SerialCmd.println(txrxbuffer[i + 3]);
#endif
          }
        } else if (SerCmd == 5) { //AT+24WL
          writeAT24(defaultAT24i2ca, (txrxbuffer[0] << 8 + txrxbuffer[1]), txrxbuffer[2], txrxbuffer + 3);
        }
        resetSer();
      } else if (SerRXidx == 3 && SerCmd == 5) {
        SerialCmd.println(txrxbuffer[2]);
        SerialDbg.println(F("Adjusting char RX"));
        SerRXmax = txrxbuffer[2] + 3;
      }
    } else {
      char rc = SerialCmd.read();
      if (rc != endMarker && rc != endMarker2) {
        serBuffer[SerRXidx] = rc;
        SerRXidx++;
        if (SerRXidx >= MAX_SER_LEN) {
          SerRXidx = 0;
          SerialCmd.println(F("ERROR"));
          resetSer();
        }
      } else {
        if (SerRXidx) { //index 0? means it's a \r\n pattern.
          serBuffer[SerRXidx] = '\0'; // terminate the string
          SerRXidx = 0;
          checkCommand();
        }
      }
    }
    lastSer = millis();
  }
  PORTA|=1;
}

void checkCommand() {
  if (strcmp_P (serBuffer, ATSEND) == 0) {
    SerRXmax = 4;
    rxing = 2;
  } else if (strcmp_P (serBuffer, ATSENDM) == 0) {
    SerRXmax = 7;
    rxing = 2;
  } else if (strcmp_P (serBuffer, ATSENDL) == 0) {
    SerRXmax = 15;
    rxing = 2;
  } else if (strcmp_P (serBuffer, ATSENDE) == 0) {
    SerRXmax = 31;
    rxing = 2;
  } else if (strcmp_P (serBuffer, ATCONF) == 0) {
    SerRXmax = 28;
    rxing = 2;
  } else if (strcmp_P (serBuffer, ATHEX) == 0) {
#ifdef HEX_IN
#ifdef HEX_OUT
    SerialCmd.println(F("03"));
#else
    byte a = 2;
    SerialCmd.println(a);
#endif
#else
#ifdef HEX_OUT
    SerialCmd.println(F("01"));
#else
    byte a = 0;
    SerialCmd.println(a);
#endif
#endif

  } else if (strcmp_P (serBuffer, ATADRQ) == 0) { //AT+ADR?
    SerialCmd.println(MyAddress);
  } else if (strcmp_P (serBuffer, ATADR) == 0) {
    SerCmd = 1;
    SerRXmax = 1;
    rxing = 2;
  } else if (strcmp_P (serBuffer, ATVERS) == 0) {
    SerialCmd.println("AzzyRF v2.2");
  } else if (strcmp_P (serBuffer, AT24R) == 0) {
    SerCmd = 2;
    SerRXmax = 2;
    rxing = 2;
  } else if (strcmp_P (serBuffer, AT24W) == 0) {
    SerCmd = 3;
    SerRXmax = 3;
    rxing = 2;
  } else if (strcmp_P (serBuffer, AT24RL) == 0) {
    SerCmd = 4;
    SerRXmax = 3;
    rxing = 2;
  } else if (strcmp_P (serBuffer, AT24WL) == 0) {
    SerCmd = 5;
    SerRXmax = 19;
    rxing = 2;
    #ifdef USE_ACK
  } else if (strcmp_P (serBuffer, ATACK) == 0) { //AT+ADR?
    if (lastChecksum) {
        prepareAckPayload();
        doTransmit();
    } else {
      SerialCmd.println(F("ERROR"));
      resetSer();
    }
    #endif
  } else {
    SerialCmd.println(F("ERROR"));
    SerialCmd.println(serBuffer);
    resetSer();
  }
  if (rxing == 2) {
#ifdef HEX_IN
    SerialCmd.print(F(">"));
#else
    SerialCmd.print(F("#"));
#endif
  }


}

void resetSer() {
  //SerialDbg.println(lastSer);
  //SerialDbg.println(millis());
  SerCmd = 0;
  if (lastSer) { //if we've gotten any characters since last reset, print newline to signify completion.
    lastSer = 0;
    SerialCmd.println();
  }
  if (rxing == 2) {
    rxing = 0;
  }
  for (int i = 0; i < 16; i++) {
    SerCmBuff[i] = 0;
  }
  SerRXmax = 0;
  SerRXidx = 0;
}

void preparePayloadFromSerial() {
  txrxbuffer[0] = (txrxbuffer[0] & 0x3F);
  if (SerRXmax > 4) {
    txrxbuffer[0] = txrxbuffer[0] | (SerRXmax == 7 ? 0x40 : (SerRXmax == 15 ? 0x80 : 0xC0));
    TXLength = SerRXmax + 1;
  } else {
    txrxbuffer[3] = txrxbuffer[3] << 4;
    TXLength = 4;
  }
}

void prepareAckPayload() {
  byte plen = txrxbuffer[0] >> 6;
  plen = 4 << plen;
  txrxbuffer[0] = (txrxbuffer[0] & 0x3F);
  txrxbuffer[2] = txrxbuffer[1];
  txrxbuffer[1] = 0xE8;
  txrxbuffer[3] = (txrxbuffer[plen - 1] & 0x0F) << 4;
  TXLength = 4;
}

void prepareTestPayload() {
  txrxbuffer[0] = 0x00; //address zero
  txrxbuffer[2] = millis() & 255;
  txrxbuffer[1] = 0xF8;
  txrxbuffer[3] = 0x50;
  TXLength = 4;
}

void doTransmit() {
  doTransmit(txRepCount);
}

void doTransmit(byte rep) { //rep is the number of repetitions
SerialDbg.println("dotransmit ok");
#ifdef LED_TX
  digitalWrite(LED_TX, LED_ON);
#endif
#ifdef SHUT_PIN
  digitalWrite(SHUT_PIN, 1);
#endif
digitalWrite(txpin,0); // known state
  byte txchecksum = 0;
  for (byte i = 0; i < TXLength - 1; i++) {
    txchecksum = txchecksum ^ txrxbuffer[i];
  }
  if (TXLength == 4) {
    txchecksum = (txchecksum & 0x0F) ^ (txchecksum >> 4) ^ ((txrxbuffer[3] & 0xF0) >> 4);
    txrxbuffer[3] = (txrxbuffer[3] & 0xF0) + (txchecksum & 0x0F);
  } else {
    txrxbuffer[TXLength - 1] = txchecksum;
  }
#ifdef USE_ACK
  lastCmdSent = txrxbuffer[1];
  lastCscSent = txchecksum;
#endif
  for (byte r = 0; r < rep; r++) {
    for (byte j = 0; j <= 2 * txTrainRep; j++) {
      delayMicroseconds(txTrainLen);
      //digitalWrite(txpin, j & 1);
      txPIN=txBV;
    }
    delayMicroseconds(txSyncTime);
    txPIN=txBV;
    delayMicroseconds(txSyncTime);
    for (byte k = 0; k < TXLength; k++) {
      //send a byte
      for (int m = 7; m >= 0; m--) {
        txPIN=txBV;
        if ((txrxbuffer[k] >> m) & 1) {
          delayMicroseconds(txOneLength);
        } else {
          delayMicroseconds(txZeroLength);
        }
        txPIN=txBV;
        delayMicroseconds(txLowTime);
      }
      //done with that byte
    }
    //done with sending this packet;
    digitalWrite(txpin, 0); //make sure it's off;
    //interrupts();
    delayMicroseconds(txRepDelay); //wait 2ms before doing the next round.
  }
#ifdef USE_ACK
  if (txrxbuffer[1] == 0xE8) {
    SerialCmd.println(F("RX ACK"));
  } else {
#endif
    SerialCmd.println(F("TX OK"));
#ifdef USE_ACK
  }
#endif
  TXLength = 0;
#ifdef LED_TX
  digitalWrite(LED_TX, LED_OFF);
#endif

#ifdef SHUT_PIN
  digitalWrite(SHUT_PIN, 0);
#endif
}


void outputPayload() {
    for (byte i=0;i<32;i++) {
      if (highLengths[i]!=-1 || lowLengths[i]!=-1) {
        SerialDbg.print(i);
        SerialDbg.print(F(":"));
      }
    if (highLengths[i]!=-1) {
      SerialDbg.print(F("H:"));
      SerialDbg.print(highLengths[i]);
    }
    if (lowLengths[i]!=-1) {
      SerialDbg.print(F("L:"));
      SerialDbg.print(lowLengths[i]);
    }
    if (highLengths[i]!=-1 || lowLengths[i]!=-1) {
        SerialDbg.println();
      }
  }
#ifdef USE_ACK
  if (txrxbuffer[1] == 0xE8) {
    if ( txrxbuffer[2] == lastCmdSent && (txrxbuffer[3] >> 4) == (lastCscSent & 0x0F)) {
      SerialCmd.println(F("ACK"));
    } else {
      SerialDbg.println(F("Other ACK"));
    }
  } else {
#endif
    byte tem = txrxbuffer[0] >> 6;
    tem = (4 << tem) - 1;
#ifdef HEX_OUT
    SerialCmd.print(F("+"));
    for (byte x = 0; x < tem ; x++) {
      showHex(txrxbuffer[x], 1);
    }
    if (tem == 3) { //means it was a short
      showHex((txrxbuffer[3] & 0xF0) >> 4, 1);
    }
#else
    SerialCmd.print(tem == 3 ? 4 : tem);
    SerialCmd.print(F(","));
    for (byte x = 0; x < tem; x++) {
      SerialCmd.print(txrxbuffer[x]);
    }
    if (tem == 3) { //means it was a short
      SerialCmd.print((txrxbuffer[3] & 0xF0) >> 4);
    }
#endif
#ifdef USE_ACK
#ifdef AUTO_ACK
#ifdef BTN_ACK
    if (digitalRead(BTN_ACK) == 0) {
#endif
      if (((txrxbuffer[0] & 0x3F) == MyAddress) && (txrxbuffer[1] != 0xE8)) {
        prepareAckPayload();
        delay(1000);
        doTransmit();
      }
#ifdef BTN_ACK
    }
#endif
#endif
  }
#endif
  SerialCmd.println();
}

void onListenST() {

  curTime = micros();
#ifdef rxPIN
  byte pinState = (rxPIN & rxBV) ? 1 : 0;
#else
  byte pinState = digitalRead(rxpin);
#endif
  if (pinState == lastPinState) {
    return;
  } else {
    //digitalWrite(LED1,pinState);
    lastPinState = pinState;
  }
  if (pinState == 0) {
    lastPinLowTime = curTime;
    unsigned long bitlength = lastPinLowTime - lastPinHighTime;
    if (rxing == 1) {
      //digitalWrite(LED1,0);
      if (bitlength > rxZeroMin && bitlength < rxZeroMax) {
        rxdata = rxdata << 1;
      } else if (bitlength > rxOneMin && bitlength < rxOneMax ) {
        rxdata = (rxdata << 1) + 1;
      } else {
        if (bitsrx > 10) {
          SerialDbg.print(F("Reset wrong high len "));
          SerialDbg.print(bitlength);
          SerialDbg.print(" ");
          SerialDbg.println(bitsrx);
          //delay(2000);
        }
        resetListen();
        return;
      }
      highLengths[bitsrx]=bitlength;
      bitsrx++;
      if (bitsrx == 2) {
        pksize = 32 << rxdata;
#if (RX_MAX_LEN < 256)
        if (pksize > RX_MAX_LEN) {
          SerialDbg.println(F("Packet this size not supported"));
          resetListen();
          return;
        }
#endif

      } else if ((bitsrx & 0x07) == 0 && bitsrx) {
        txrxbuffer[(bitsrx >> 3) - 1] = rxdata;
        showHex(rxdata);
        rxdata = 0;
      }
      //SerialDbg.println(bitsrx);
      if (bitsrx == pksize) {
        SerialDbg.println(F("RX done"));
        parseRx();
        //parseRx2(txrxbuffer,pksize/8);
        resetListen();
      }

      return;
    }
  } else {
    lastPinHighTime = curTime;
    unsigned long bitlength =lastPinHighTime - lastPinLowTime;
    if (bitlength > rxSyncMin && bitlength < rxSyncMax && rxing == 0) {
      rxing = 1;
      return;
    }
    if (rxing==1) {
      lowLengths[bitsrx-1]=bitlength;
      if (bitlength > rxLowMax && rxing == 1) {
        
        if (bitsrx > 10) {
        SerialDbg.print("rxlow");
        SerialDbg.print(lastPinHighTime - lastPinLowTime);
        SerialDbg.print(" ");
        SerialDbg.println(bitsrx);
      //delay(2000);
        }
        resetListen();
        return;
      }
    }
  }
}




void parseRx() { //uses the globals.
  SerialDbg.println(F("Parsing"));
  unsigned char calccsc = 0;
  unsigned char rcvAdd = txrxbuffer[0] & 0x3F;
  if (rcvAdd == MyAddress || MyAddress == 0) {
    if (lastChecksum != calcBigChecksum(byte(pksize / 8))) {
      lastChecksum = calcBigChecksum(byte(pksize / 8));
      if (pksize == 32) { //4 byte packet
        calccsc = txrxbuffer[0] ^ txrxbuffer[1] ^ txrxbuffer[2];
        calccsc = (calccsc & 15) ^ (calccsc >> 4) ^ (txrxbuffer[3] >> 4);
        if (calccsc == (txrxbuffer[3] & 15)) {
          //MyCmd = txrxbuffer[1];
          //MyParam = txrxbuffer[2];
          //MyExtParam = txrxbuffer[3] >> 4;

          /* showHex(txrxbuffer[0], 0);
           SerialDbg.print(F(":"));
           showHex(MyCmd);
           SerialDbg.print(F(":"));
           showHex(MyParam);
           SerialDbg.print(F(":"));
           showHex(MyExtParam);
           SerialDbg.println(); */
          SerialDbg.println(F("Valid RX"));
          outputPayload();
        } else {
          SerialDbg.println(F("Bad CSC RX"));
          //outputPayload();
        }
      } else {
        for (byte i = 1; i < (pksize / 8); i++) {
          calccsc = calccsc ^ txrxbuffer[i - 1];
        }
        if (calccsc == txrxbuffer[(pksize / 8) - 1]) {
          //MyCmd = txrxbuffer[1];
          //MyParam = txrxbuffer[2];
          //MyExtParam = txrxbuffer[3];
          /*
          for (byte i = 0; i < (pksize / 8); i++) {
            showHex(txrxbuffer[i]);
            SerialDbg.print(":");
          }
          SerialDbg.println();
          */
          SerialDbg.println(F("Valid long RX"));
          outputPayload();
        } else {
          SerialDbg.println(F("Bad CSC long RX"));
        }
      }
    } else {
      SerialDbg.println(F("Already got it"));
    }
  } else {
    SerialDbg.println(F("Not for me"));
  }
}



unsigned long calcBigChecksum(byte len) {
  unsigned long retval = 0;
  for (byte i = 0; i < len; i++) {
    retval += ((unsigned long)txrxbuffer[i] << ((i & 3) * 8));
  }
  return retval;
}

void resetListen() {
  //SerialDbg.println(F("reset listen"));
  bitsrx = 0;
  rxdata = 0;
  rxing = 0;
  rxaridx = 0;
  for (byte i=0;i<32;i++) {
    if (highLengths[i]!=-1) {
      //SerialDbg.print(F("H:"));
      //SerialDbg.print(highLengths[i]);
      highLengths[i]=-1;
    }
    if (lowLengths[i]!=-1) {
      //SerialDbg.print(F("L:"));
      //SerialDbg.println(lowLengths[i]);
      lowLengths[i]=-1;
    }
  }
}


//decode times
unsigned long decode8(byte inp) {
  return (inp & 0x3F) * units[inp >> 6];
}
unsigned long decode12(unsigned int inp) {
  return (inp & 0x01FF) * units[(inp >> 9) & 0x07];
}
unsigned long decode16(unsigned int inp) {
  return (inp & 0x1FFF) * units[inp >> 13];
}


void ClearCMD() {  //This handles clearing of the commands, and also clears the lastChecksum value, which is used to prevent multiple identical packets received in succession from being processed.
  if (lastChecksum && forgetCmdAt==0) {
    forgetCmdAt = millis() + CommandForgetTime;
    //MyParam = 0;
    //MyExtParam = 0;
    //MyCmd = 0;
  } else if (millis() > forgetCmdAt) {
    forgetCmdAt = 0;
    lastChecksum = 0;
  }
}


byte readAT24(byte haddr, unsigned int addr) {
  TinyWireM.beginTransmission(haddr);
  TinyWireM.send((byte)(addr >> 8));
  TinyWireM.send((byte)(addr & 255));
  TinyWireM.endTransmission();
  TinyWireM.requestFrom(haddr, 1);
  return TinyWireM.receive();
}

void readAT24(byte haddr, unsigned int addr, byte len, byte * dat) {
  TinyWireM.beginTransmission(haddr);
  TinyWireM.send((byte)(addr >> 8));
  TinyWireM.send((byte)(addr & 255));
  TinyWireM.endTransmission();
  TinyWireM.requestFrom(haddr, len);
  for (byte i = 0; i < len; i++) {
    dat[i] = TinyWireM.receive();
    SerialDbg.println(dat[i]);
  }
}

void writeAT24(byte haddr, unsigned int addr, byte len, byte * dat) {
  TinyWireM.beginTransmission(haddr);
  TinyWireM.send((byte)(addr >> 8));
  TinyWireM.send((byte)(addr & 255));
  SerialDbg.println(dat[len - 1]);
  SerialDbg.println(len);
  TinyWireM.send(dat, len);
  TinyWireM.endTransmission();
  delay(10);
  SerialDbg.println(F("Wrote block"));
}

void writeAT24(byte haddr, unsigned int addr, byte dat) {
  TinyWireM.beginTransmission(haddr);
  TinyWireM.send((byte)(addr >> 8));
  TinyWireM.send((byte)(addr & 255));
  TinyWireM.send(dat);
  TinyWireM.endTransmission();
  delay(10);
  SerialDbg.println(F("Wrote byte"));

}


void initializeOutputs() {
#ifdef LED1
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, LED_OFF);
#endif
#ifdef LED2
  pinMode(LED2, OUTPUT);
  digitalWrite(LED2, LED_OFF);
#endif
#ifdef LED3
  pinMode(LED3, OUTPUT);
  digitalWrite(LED3, LED_OFF);
#endif
#ifdef LED4
  pinMode(LED4, OUTPUT);
  digitalWrite(LED4, LED_OFF);
#endif
#ifdef LED5
  pinMode(LED5, OUTPUT);
  digitalWrite(LED5, LED_OFF);
#endif
#ifdef LED6
  pinMode(LED6, OUTPUT);
  digitalWrite(LED6, LED_OFF);
#endif
#ifdef SHUT_PIN
  pinMode(SHUT_PIN, OUTPUT);
  digitalWrite(SHUT_PIN, 0);
#endif
#ifdef BTN0
  pinMode(BTN0, INPUT_PULLUP);
#endif
#ifdef BTN1
  pinMode(BTN1, INPUT_PULLUP);
#endif
#ifdef BTN2
  pinMode(BTN2, INPUT_PULLUP);
#endif
#ifdef BTN3
  pinMode(BTN3, INPUT_PULLUP);
#endif
#ifdef BTN4
  pinMode(BTN4, INPUT_PULLUP);
#endif

}

void showHex (const byte b, const byte c) {
  // try to avoid using sprintf
  char buf [3] = { ((b >> 4) & 0x0F) | '0', (b & 0x0F) | '0', 0};
  if (buf [0] > '9')
    buf [0] += 7;
  if (buf [1] > '9')
    buf [1] += 7;

  if (c) {
    SerialCmd.print(buf);
  }
  else {
    SerialDbg.print(buf);
  }
}
