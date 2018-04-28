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

//Pin definitions:

//pins 0 and 1 for xtal
#define LED1 2
#define LED2 3
#define LED3 4
//pins 5 and 6 used for serial 1 (command)
#define txpin 7
//pin 8 and 9 for serial 0 (program+debug)
#define rxpin 10

#define LED_ON 0
#define LED_OFF 1

//#define TWO_WIRE_FLOW



#define CommandForgetTime 1000 //short, for testing

#define rcvled LED1
#define RX_MAX_LEN 256 //Used to set the size of txrx buffer (and checked against this to prevent overflows from messing stuff up)

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
// Version 2.1
int rxSyncMin = 1900; //minimum valid sync length
int  rxSyncMax = 2100; //maximum valid sync length
int  rxZeroMin = 120; //minimum length for a valid 0
int  rxZeroMax = 400; //maximum length for a valid 0
int  rxOneMin = 450; //minimum length for a valid 1
int  rxOneMax = 750; //maximum length for a valid 1
int  rxLowMax = 600; //longest low before packet discarded

int  txOneLength = 550; //length of a 1
int  txZeroLength = 300; //length of a 0
int txLowTime = 420; //length of the gap between bits
int txTrainRep = 30; //number of pulses in training burst
int txSyncTime = 2000; //length of sync
int txTrainLen = 200; //length of each pulse in training burst


unsigned long units[] = {1000, 60000, 900000, 14400000, 3600000, 1, 10, 86400000}; //units for the 8/12/16-bit time values.


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

char * pEnd; //dummy pointer for sto

#define SerialDbg Serial
#define SerialCmd Serial1
#define HEX_OUT
//#define HEX_IN
#define MAX_SER_LEN 10
char serBuffer[MAX_SER_LEN];
//#define USE_ACK

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



void setup() {
  lastPinState = 0;
  lastPinLowTime = 0;
  lastPinHighTime = 0;
  rxdata = 0;
  bitsrx = 0;
  rxing = 0;
  //MyState = ListenST;
  digitalWrite(LED1, LED_OFF);
  digitalWrite(LED2, LED_OFF);
  #ifdef TWO_WIRE_FLOW
  pinMode(LED3, INPUT);
  #else
  pinMode(LED3, OUTPUT);
  digitalWrite(LED3, LED_OFF);
  #endif
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(txpin, OUTPUT);
  pinMode(rxpin, INPUT);
  Serial.begin(9600);
  if (EEPROM.read(0) < 255) {
    //initFromEEPROM();
    //SerialDbg.println(F("Load from EEPROM"));
  }
  Serial1.begin(9600);
  digitalWrite(LED2, LED_ON);
  delay(1000);
  digitalWrite(LED2, LED_OFF);
  SerialDbg.println(F("Startup OK"));

}


void loop() {
  curTime = micros();
  //if (MyState == ListenST) {
  ClearCMD(); //do the command reset only if we are in listenst but NOT receiving.
  onListenST();
  if (rxing == 1) {
    return; //don't do anything else while actively receiving.
  } else {
    processSerial();
  }
  //} else if (MyState == CommandST) {
  //  onCommandST();
  //} else {
  //  MyState = ListenST; //in case we get into a bad state somehow.
  //}
  if (millis() - lastSer  > 10000) {
    resetSer();
  }
}

void writeConfigToEEPROM() {
  for (byte i = 0; i < 28; i++) {
    EEPROM.write(i, txrxbuffer[i]);
  }
  initFromEEPROM();
}

//Nick Gammon's showHex() function, slightly modified.
void showHex (const byte b, const byte c = 0)
{
  // try to avoid using sprintf
  char buf [3] = { ((b >> 4) & 0x0F) | '0', (b & 0x0F) | '0', 0};
  if (buf [0] > '9')
    buf [0] += 7;
  if (buf [1] > '9')
    buf [1] += 7;

  if (c) {
    SerialCmd.print(buf);
  } else {
    SerialDbg.print(buf);
  }
}  // end of showHex

void initFromEEPROM() {
  /*byte tAddr = EEPROM.read(0);
  if (tAddr < 64) {
    MyAddress = tAddr;
    // Version 2.1 EEPROM.read(2)+(EEPROM.read(1)<<8);

    txSyncTime = EEPROM.read(2) + (EEPROM.read(1) << 8); //length of sync
    txTrainRep = EEPROM.read(3); //number of pulses in training burst
    txTrainLen = EEPROM.read(5) + (EEPROM.read(4) << 8); //length of each pulse in training burst

    txOneLength = EEPROM.read(7) + (EEPROM.read(6) << 8); //length of a 1
    txZeroLength = EEPROM.read(9) + (EEPROM.read(8) << 8); //length of a 0
    txLowTime = EEPROM.read(11) + (EEPROM.read(10) << 8); //length of the gap between bits

    rxSyncMin = EEPROM.read(13) + (EEPROM.read(12) << 8); //minimum valid sync length
    rxSyncMax = EEPROM.read(15) + (EEPROM.read(14) << 8); //maximum valid sync length
    rxZeroMin = EEPROM.read(17) + (EEPROM.read(16) << 8); //minimum length for a valid 0
    rxZeroMax = EEPROM.read(19) + (EEPROM.read(18) << 8); //maximum length for a valid 0
    rxOneMin = EEPROM.read(21) + (EEPROM.read(20) << 8); //minimum length for a valid 1
    rxOneMax = EEPROM.read(23) + (EEPROM.read(22) << 8); //maximum length for a valid 1
    rxLowMax = EEPROM.read(25) + (EEPROM.read(24) << 8); //longest low before packet discarded
  }
  */

}

void processSerial() {
  static char minb[3] = {0, 0, 0};
  static byte ndx = 0;
  char endMarker = '\r';
  char endMarker2 = '\n';
  while (SerialCmd.available() > 0) {

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
        if (SerRXmax == 26) {
          writeConfigToEEPROM();
          } else if (SerRXmax==1) {
            MyAddress=txrxbuffer[0];
          //  EEPROM.write(0,MyAddress)
        } else {
          preparePayloadFromSerial();
          doTransmit(5);
        }
        resetSer();
      }
    } else if (rxing == 0) {
      char rc = SerialCmd.read();
      if (rc != endMarker && rc != endMarker2) {
        serBuffer[ndx] = rc;
        ndx++;
        if (ndx >= MAX_SER_LEN) {
          ndx = 0;
        }
      } else {
        if (ndx) { //index 0? means it's a \r\n pattern.
          serBuffer[ndx] = '\0'; // terminate the string
          ndx = 0;
          checkCommand();
        }
      }
    }
    lastSer = millis();
  }
}

void checkCommand() {
  if (strcmp (serBuffer, "AT+SEND") == 0) {
    SerRXmax = 4;
    rxing = 2;
  } else if (strcmp (serBuffer, "AT+SENDM") == 0) {
    SerRXmax = 7;
    rxing = 2;
  } else if (strcmp (serBuffer, "AT+SENDL") == 0) {
    SerRXmax = 15;
    rxing = 2;
  } else if (strcmp (serBuffer, "AT+SENDE") == 0) {
    SerRXmax = 31;
    rxing = 2;
  } else if (strcmp (serBuffer, "AT+CONF") == 0) {
    SerRXmax = 26;
    rxing = 2;
  } else if (strcmp (serBuffer, "AT+HEX?") == 0) {
#ifdef HEX_IN
#ifdef HEX_OUT
    SerialCmd.println("03");
#else
    byte a = 2;
    SerialCmd.println(a);
#endif
#else
#ifdef HEX_OUT
    SerialCmd.println("01");
#else
    byte a = 0;
    SerialCmd.println(a);
#endif
#endif

  } else if (strcmp (serBuffer, "AT+ADR?") == 0) {
    SerialCmd.println(MyAddress);
  } else if (strcmp (serBuffer, "AT+ADR") == 0) {
    SerRXmax = 1;
    rxing = 2;
  } else {
    SerialCmd.println(F("ERROR"));
    SerialCmd.println(serBuffer);
    resetSer();
  }
  if (rxing == 2) {
    SerialCmd.print(F(">"));
  }


}

void resetSer() {
  if (rxing == 2) {
    rxing = 0;
  }
  if (lastSer) {
    lastSer = 0;
    SerialCmd.println();
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



void doTransmit(int rep) { //rep is the number of repetitions
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
      digitalWrite(txpin, j & 1);
    }
    delayMicroseconds(txSyncTime);
    for (byte k = 0; k < TXLength; k++) {
      //send a byte
      for (int m = 7; m >= 0; m--) {
        digitalWrite(txpin, 1);
        if ((txrxbuffer[k] >> m) & 1) {
          delayMicroseconds(txOneLength);
        } else {
          delayMicroseconds(txZeroLength);
        }
        digitalWrite(txpin, 0);
        delayMicroseconds(txLowTime);
      }
      //done with that byte
    }
    //done with sending this packet;
    digitalWrite(txpin, 0); //make sure it's off;
    //interrupts();
    delayMicroseconds(2000); //wait 2ms before doing the next round.
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
}


void outputPayload() {
#ifdef TWO_WIRE_FLOW 
digitalWrite(LED2,1);
while (digitalRead(LED3)==0){
  ; //do nothing until that pin driven high
}
digitalWrite(LED2,0);
#endif
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
    SerialCmd.print(F("+"));
#ifdef HEX_OUT
    for (byte x = 0; x < tem ; x++) {
      showHex(txrxbuffer[x],1);
    }
    if (tem == 3) { //means it was a short
      showHex((txrxbuffer[3] & 0xF0) >> 4,1);
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
    SerialCmd.println();
#ifdef USE_ACK
    if ((txrxbuffer[0]&0x3F)==MyAddress) {
    prepareAckPayload();
    delay(1000);
    doTransmit(5);
    }
  }
#endif


  SerialCmd.println();
}

void onListenST() {
  byte pinState = digitalRead(rxpin);
  if (pinState == lastPinState) {
    return;
  } else {
    lastPinState = pinState;
  }
  if (pinState == 0) {
    lastPinLowTime = curTime;
    unsigned long bitlength = lastPinLowTime - lastPinHighTime;
    if (rxing == 1) {
      if (bitlength > rxZeroMin && bitlength < rxZeroMax) {
        rxdata = rxdata << 1;
      } else if (bitlength > rxOneMin && bitlength < rxOneMax ) {
        rxdata = (rxdata << 1) + 1;
      } else {
        //if (bitsrx > 10) {
        //  Serial.print(F("Reset wrong high len "));
        //  Serial.print(bitlength);
        //  Serial.print(" ");
        //  Serial.println(bitsrx);
        //}
        resetListen();
        return;
      }
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
        txrxbuffer[(bitsrx >> 3)-1] = rxdata;
        //showHex(rxdata);
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
    if (lastPinHighTime - lastPinLowTime > rxSyncMin && lastPinHighTime - lastPinLowTime < rxSyncMax && rxing == 0) {
      rxing = 1;
      return;
    }
    if (lastPinHighTime - lastPinLowTime > rxLowMax && rxing == 1) {
      resetListen();
      return;
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
          outputPayload();
        }
      } else {
        for (byte i = 1; i < (pksize / 8); i++) {
          calccsc = calccsc ^ txrxbuffer[i - 1];
        }
        if (calccsc == txrxbuffer[(pksize / 8) - 1]) {
          //MyCmd = txrxbuffer[1];
          //MyParam = txrxbuffer[2];
          //MyExtParam = txrxbuffer[3];
          for (byte i = 0; i < (pksize / 8); i++) {
            showHex(txrxbuffer[i]);
            SerialDbg.print(":");
          }
          SerialDbg.println();
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
    retval += (txrxbuffer[i] << (i >> 1));
  }
  return retval;
}

void resetListen() {
  bitsrx = 0;
  rxdata = 0;
  rxing = 0;
  rxaridx = 0;
}

/*
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
*/

void ClearCMD() {  //This handles clearing of the commands, and also clears the lastChecksum value, which is used to prevent multiple identical packets received in succession from being processed.
  if (lastChecksum) {
    forgetCmdAt = millis() + CommandForgetTime;
    //MyParam = 0;
    //MyExtParam = 0;
    //MyCmd = 0;
  } else if (millis() > forgetCmdAt) {
    forgetCmdAt = 0;
    lastChecksum = 0;
  }
}
