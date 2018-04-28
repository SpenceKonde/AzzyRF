/*
This is an adaptation of TXrxbasev21 for use with tiny841 tower light
*/

#define ListenST 1
#define CommandST 2
#include <EEPROM.h>


#define rxpin 10
#define txpin 7
#define txPIN PINA
#define txBV 8
#define CommandForgetTime 1000 //short, for testing

#define ledO 2
#define ledB 3
#define ledG 6//4
#define ledR 5
#define ledW 4//6
#define fridge 8
#define doorup 1
#define doordn 0
#define doordnanalog 11
#define rxmaxlen 32 //
#define FADE_TIME 4

//These set the parameters for transmitting.




/*
//These set the parameters for receiving; any packet where these criteria are not met is discarded.
// Version 2.0
int rxSyncMin=1900; //minimum valid sync length
int rxSyncMax=2100; //maximum valid sync length
int rxZeroMin=100; //minimum length for a valid 0
int rxZeroMax=300; //maximum length for a valid 0
int rxOneMin=400; //minimum length for a valid 1
int rxOneMax=600; //maximum length for a valid 1
int rxLowMax=450; //longest low before packet discarded

*/

// Version 2.2
// Version 2.2
const unsigned int rxSyncMin  = 1750;
const unsigned int rxSyncMax  = 2250;
const unsigned int rxZeroMin  = 100;
const unsigned int rxZeroMax  = 390;
const unsigned int rxOneMin  = 410;
const unsigned int rxOneMax  = 700;
const unsigned int rxLowMax  = 600;
const unsigned int txOneLength  = 500;
const unsigned int txZeroLength  = 300;
const unsigned int txLowTime  = 400;
const unsigned int txSyncTime  = 2000;
const unsigned int txTrainLen  = 200;

const unsigned int txRepDelay = 2000; //delay between consecutive transmissions
const byte txTrainRep  = 30;
#define TXLength 4
const unsigned long units[] = {1000, 60000, 900000, 14400000, 3600000, 1, 10, 86400000}; //units for the 8/12/16-bit time values.


unsigned char MyAddress = 31;


//Pin state tracking and data for receiving.
byte lastPinState;
unsigned long lastPinHighTime;
unsigned long lastPinLowTime;
unsigned long lastTempHighTime = 0;
unsigned long lastTempLowTime = 0;
byte rxdata;
byte lastTempPinState;
byte bitsrx;
byte rxing;
byte rxaridx;
unsigned char txrxbuffer[rxmaxlen >> 3];

byte MyState;
unsigned char MyCmd;
unsigned char MyParam;
unsigned char MyExtParam;
//unsigned long curTime;
int count = 0;
int badcsc = 0;
byte pksize = 32;
//byte TXLength;
unsigned long lastChecksum; //Not the same as the CSC - this is our hack to determine if packets are identical
unsigned long forgetCmdAt;

unsigned long lastfridge;
unsigned long lastup;
unsigned long lastdown;
byte fridgest;
byte upst;
byte downst;
unsigned long fadeAt[] = {0, 0, 0, 0, 0};
unsigned long fadeOff[] = {0, 0, 0, 0, 0};
byte fadest[] = {0, 0, 0, 0, 0};
byte fadedir = 0;
byte fridgeSentWarn;



void setup() {
  OSCCAL0 -= 8;
  lastPinState = 0;
  lastPinLowTime = 0;
  lastPinHighTime = 0;
  rxdata = 0;
  bitsrx = 0;
  rxing = 0;
  MyState = ListenST;
  pinMode(ledO, OUTPUT);
  digitalWrite(ledO, 1);
  delay(500);
  digitalWrite(ledO, 0);
  pinMode(ledB, OUTPUT);
  digitalWrite(ledB, 1);
  delay(500);
  digitalWrite(ledB, 0);
  pinMode(ledG, OUTPUT);
  digitalWrite(ledG, 1);
  delay(500);
  digitalWrite(ledG, 0);
  pinMode(ledR, OUTPUT);
  digitalWrite(ledR, 1);
  delay(500);
  digitalWrite(ledR, 0);
  pinMode(ledW, OUTPUT);
  digitalWrite(ledW, 1);
  delay(500);
  digitalWrite(ledW, 0);
  pinMode(txpin, OUTPUT);
  pinMode(rxpin, INPUT);
  pinMode(doorup, INPUT_PULLUP);
  pinMode(doordn, INPUT); //external pullup
  Serial.begin(9600);
  delay(10000);
  UCSR0B &= ~(1 << RXEN0); //turn off RX but leave TX
  pinMode(fridge, INPUT_PULLUP);
  Serial.println(F("Startup OK"));

}


void loop() {
  if (MyState == ListenST) {
    ClearCMD(); //do the command reset only if we are in listenst but NOT receiving.
    onListenST();
    if (rxing == 1) {
      return; //don't do anything else while actively receiving.
    }
  } else if (MyState == CommandST) {
    onCommandST();
  } else {
    MyState = ListenST; //in case we get into a bad state somehow.
  }
  checkInputs();
  doFade();
}
void doFade() {
  for (byte i = 0; i < 5; i++) {
    if (fadeAt[i] != 0 && fadeAt[i] < millis()) {
      if (fadest[i]) {
        analogWrite(i + 2, (((fadedir >> i) & 0x01 )? ++fadest[i] : --fadest[i]));
        fadeAt[i] = fadest[i] ? millis() + (i != 2 ? FADE_TIME : FADE_TIME << 4) : 0; //hack to make white fade slow
        if (fadest[i] == 255) {
          fadedir |= 1 << i;
        }
      } else {
        if (fadeOff[i] > millis()) {
          fadest[i] = 1;
          analogWrite(i + 2, 1);
          fadedir &= ~1 << i;
        } else { //put the orange and red lights back where they should be if we've moved them.
          if (i == 0) {
            digitalWrite(i + 2, downst);
          } else if (i == 3) {
            digitalWrite(i + 2, !upst);
          }
        }
      }
    }
  }
}
void checkInputs() {
  Serial.print(F("B"));
  Serial.print(upst + 0);
  Serial.print(downst + 0);
  Serial.println(fridgest + 0);
  Serial.print(fadeAt[2] + 0);
  Serial.print(fadest[2] + 0);
  Serial.println(0 + (fadedir >> 2) & 0x01);
  Serial.println(analogRead(doordnanalog));

  if (upst != digitalRead(doorup)) {
    lastup = millis();
    upst = digitalRead(doorup);
    doUpstairs();
  }
  byte curdownst = analogRead(doordnanalog) >> 2; //~255 = door open, ~0 thing triggered, ~128 = door closed
  curdownst = (curdownst > 200 ? 1 : (curdownst > 80 ? 0 : 2));
  if (curdownst != downst) {
    downst = curdownst;
    lastdown = millis();
    doDownstairs();
  }
  if (fridgest != digitalRead(fridge)) {
    lastfridge = millis();
    fridgest = digitalRead(fridge);
    if (fridgest == 0) {
      fridgeSentWarn = 0;
    } else {
      digitalWrite(ledB, 0);
      digitalWrite(ledG, 0);
    }

  }
  if (fridgest == 0) {
    unsigned long fridgetmp = millis() - lastfridge;
    if (fridgetmp > 5000) {
      fridgetmp -= 5000;
      digitalWrite(ledB, fridgetmp % 1200 < 600);
      if (fridgetmp > 10000) {
        fridgetmp -= 10000;
        digitalWrite(ledG, (fridgetmp % 800 < 400));
        if (!fridgeSentWarn) {
          prepareNoticePacket(max(255, 15 + fridgetmp / 1000), 2);
          doTransmit(20);
          fridgeSentWarn = 1;
        }
      }
    }
  }

  delay(200);
}

void doDownstairs() {
  if (downst == 0) {


    fadeAt[0] = millis() + 50;

  } else {
    digitalWrite(ledO, 1);
  }
  if (downst == 2) {
    digitalWrite(ledW, 1);
    fadest[2] = 255;
    fadeAt[2] = millis() + 10000;
  }
  prepareNoticePacket(downst, 0);
  doTransmit(20);
}
void doUpstairs() {
  digitalWrite(ledR, upst);
  prepareNoticePacket(upst, 1);
  doTransmit(20);
}

void onCommandST() {
  //Serial.print("onCommandST");
  Serial.println(MyCmd);
  switch (MyCmd) {
    case 0xF2: {
        //Serial.println("Starting transmit info");
        prepareEEPReadPayload();
        delay(500);
        doTransmit(5);
        MyState = ListenST;
        break;
      }

    case 0xFE: {
        Serial.println("Received test packet");
        digitalWrite(2, 1);
        delay(500);
        digitalWrite(2, 0);
        MyState = ListenST;
        break;
      }
    default:
      Serial.print("Invalid command type");
      Serial.println(MyCmd);
      MyState = ListenST;
  }
}


void prepareEEPReadPayload() {
  //unsigned char Payload1=EEPROM.read(MyParam);
  //unsigned char Payload2=EEPROM.read(MyParam+MyExtParam);
  unsigned char oldcsc = ((MyAddress & 0xF0) >> 4) ^ (MyAddress & 0x0F) ^ (0x0F) ^ (0x02) ^ ((MyParam & 0xF0) >> 4) ^ (MyParam & 0x0F) ^ (MyExtParam & 0x0F);
  txrxbuffer[0] = MyAddress;
  txrxbuffer[1] = EEPROM.read(MyParam);
  txrxbuffer[2] = EEPROM.read(MyParam + MyExtParam);
  txrxbuffer[3] = oldcsc << 4;
  //TXLength=4;
}

void prepareNoticePacket(byte evalue, byte etype) {
  txrxbuffer[0] = MyAddress;
  txrxbuffer[1] = 0xE1;
  txrxbuffer[2] = evalue;
  txrxbuffer[3] = etype << 4;
  //TXLength=4;
}


void doTransmit(byte rep) { //rep is the number of repetitions

  digitalWrite(txpin, 0); // known state
  byte txchecksum = 0;
  for (byte i = 0; i < TXLength - 1; i++) {
    txchecksum = txchecksum ^ txrxbuffer[i];
  }
  //if (TXLength == 4) {
  txchecksum = (txchecksum & 0x0F) ^ (txchecksum >> 4) ^ ((txrxbuffer[3] & 0xF0) >> 4);
  txrxbuffer[3] = (txrxbuffer[3] & 0xF0) + (txchecksum & 0x0F);
  //} else {
  //  txrxbuffer[TXLength - 1] = txchecksum;
  //}
  for (byte r = 0; r < rep; r++) {
    for (byte j = 0; j <= 2 * txTrainRep; j++) {
      delayMicroseconds(txTrainLen);
      //digitalWrite(txpin, j & 1);
      txPIN = txBV;
    }
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
        if ((txrxbuffer[k] >> m) & 1) {
          delayMicroseconds(txOneLength);
        } else {
          delayMicroseconds(txZeroLength);
        }
      }
      //done with that byte
    }
    //done with sending this packet;
    digitalWrite(txpin, 0); //make sure it's off;
    //interrupts();
    delayMicroseconds(txRepDelay); //wait 2ms before doing the next round.
  }

  Serial.print(txrxbuffer[0]);
  Serial.print(txrxbuffer[1]);
  Serial.print(txrxbuffer[2]);
  Serial.print(txrxbuffer[3]);
  Serial.println(F("Transmit done"));
  //TXLength=0;
}



void onListenST() {
  byte pinState = digitalRead(rxpin);
  unsigned long curTime = micros();
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
        //Serial.print("Reset wrong high len ");
        //Serial.print(bitlength);
        //Serial.print(" ");
        //Serial.println(bitsrx);
        resetListen();
        return;
      }
      bitsrx++;
      if (bitsrx == 2) {
        pksize = 32 << rxdata;
        if (pksize > rxmaxlen) {
          //Serial.println("Packet this size not supported");
          resetListen();
          return;
        }
      } else if (bitsrx == 8 * (1 + rxaridx)) {
        txrxbuffer[rxaridx] = rxdata;
        rxdata = 0;
        rxaridx++;
        if  (rxaridx >= 4) //(rxaridx*8==pksize) {
          Serial.println(F("RX done"));
        parseRx();
        //parseRx2(txrxbuffer,pksize/8);
        resetListen();
      }
    }
    return;
  } else {
    lastPinHighTime = curTime;
    if (lastPinHighTime - lastPinLowTime > rxSyncMin && lastPinHighTime - lastPinLowTime < rxSyncMax && rxing == 0) {
      rxing = 1;
      return;
    }
    if (lastPinHighTime - lastPinLowTime > rxLowMax && rxing == 1) {
      //Serial.println(bitsrx);
      resetListen();
      return;
    }
  }
}




void parseRx() { //uses the globals.
  Serial.println(F("Parsing"));
  unsigned char calccsc = 0;
  unsigned char rcvAdd = txrxbuffer[0] & 0x3F;
  if (rcvAdd == MyAddress) {
    if (lastChecksum != calcBigChecksum(byte(pksize / 8))) {
      lastChecksum = calcBigChecksum(byte(pksize / 8));
      //if (pksize==32) { //4 byte packet - commented out, only 4 byte packets will be accepted by this to save space.
      calccsc = txrxbuffer[0] ^ txrxbuffer[1] ^ txrxbuffer[2];
      calccsc = (calccsc & 15) ^ (calccsc >> 4) ^ (txrxbuffer[3] >> 4);
      if (calccsc == (txrxbuffer[3] & 15)) {
        MyCmd = txrxbuffer[1];
        MyParam = txrxbuffer[2];
        MyExtParam = txrxbuffer[3] >> 4;
        MyState = CommandST;
        Serial.println(MyCmd);
        Serial.println(MyParam);
        Serial.println(MyExtParam);
        Serial.println(F("Valid RX"));
      } else {
        Serial.println(F("Bad CSC"));
      }
      /*} else { //4-byte packets only
      	for (byte i=1;i<(pksize/8);i++) {
      		calccsc=calccsc^txrxbuffer[i-1];
      	}
      	if (calccsc==txrxbuffer[(pksize/8)-1]) {
      		MyCmd=txrxbuffer[1];
      		MyParam=txrxbuffer[2];
      		MyExtParam=txrxbuffer[3];
      		MyState=CommandST; //The command state needs to handle the rest of the buffer if sending long commands.
      		Serial.println(MyCmd);
      		Serial.println(MyParam);
      		Serial.println(MyExtParam);
      		Serial.println("Valid long transmission received");
      	} else {
      		Serial.println("Bad CSC on long packet");
      	}
      }*/
    } else {
      Serial.println(F("GOT IT"));
    }
  } else {
    Serial.println(F("Not for me"));
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

//Just for test/debug purposes;


void ClearCMD() {  //This handles clearing of the commands, and also clears the lastChecksum value, which is used to prevent multiple identical packets received in succession from being processed.
  if (MyCmd) {
    forgetCmdAt = millis() + CommandForgetTime;
    MyParam = 0;
    MyExtParam = 0;
    MyCmd = 0;
  } else if (millis() > forgetCmdAt) {
    forgetCmdAt = 0;
    lastChecksum = 0;
  }
}
