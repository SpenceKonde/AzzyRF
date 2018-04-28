#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <util/crc16.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

//Pin definitions:

//pins 0,1: Serial
//pins 15,16: Serial1 (programming only)
//pins 16,12: I2C

#define LED1 13
//#define LED2 11
//#define LED3 4
//#define LED4 5
//#define LED5 6
//#define LED6 8

#define LED_RX LED1
#define LED_START LED1

//#define SHUT_PIN 8


#define LED_ON 1
#define LED_OFF 0




#define CommandForgetTime 10000 //short, for testing

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
//1634

#define USE_ACO
#define RX_PIN_STATE (PINA&4)
#define TX_PIN 14
#define txPIN PINB
#define txBV 8
#define SERIAL_CMD Serial

#define rxpin 6
#define rxPIN PINA
#define rxBV 4

#define TX_PIN 14
#define txpin 14

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


unsigned long units[] = {1000, 60000, 900000, 14400000, 3600000, 1, 10, 86400000}; //units for the 8/12/16-bit time values.


void showHex (const byte b, const byte c = 0); //declare this function so it will compile correctly with the optional second argument.

unsigned long backlightOnAt = 0;

void setup() {
  Wire.begin();
  lcd.begin();

  // Turn on the blacklight and print a message.
  lcd.backlight();
  pinMode(txpin, OUTPUT);
  pinMode(rxpin, INPUT);
  pinMode(13,OUTPUT);
  pinMode(1, INPUT_PULLUP);
  Serial.begin(9600);
  digitalWrite(LED_START, LED_ON);
  delay(1000);
  digitalWrite(LED_START, LED_OFF);
  lcd.print(F("Startup OK"));
  ////xSerialDbg.print(decode8(123));
  delay(1000);
  lcd.noBacklight();
  setupInputCapture();
}

void setupInputCapture() {
  TCCR1A = 0;
  TCCR1B = 0;
  TIFR = bit (ICF1) | bit (TOV1);  // clear flags so we don't get a bogus interrupt
  TCNT1 = 0;          // Counter to zero
#ifdef USE_ACO
  ACSRA = (1 << ACIC) | (1 << ACBG);
#endif
  TIMSK |= 1 << ICIE1; // interrupt on Timer 1 input capture
  // start Timer 1, prescalar of 8, edge select on falling edge
  TCCR1B =  ((F_CPU == 1000000L) ? (1 << CS10) : (1 << CS11)) | 1 << ICNC1; //prescalar 8 except at 1mhz, where we use prescalar of 1, noise cancler active
  //ready to rock and roll
}


void loop() {
  // put your main code here, to run repeatedly:
  byte rlen = handleReceive();
  if (rlen) {
    outputPacket(rlen);
  }
  if (backlightOnAt && (millis() - backlightOnAt > 5000)) {
    lcd.noBacklight();
    lcd.clear();
    backlightOnAt = 0;
  }
}

void outputPacket(byte rlen) { //format of rlen: what is passed back from handleReceive(), first 2 bits are version-1, last 6 are length in bytes

  byte vers = (rlen & 196) >> 6;
  rlen &= 0x3F;
  if (vers == 0) { //version 1
    Serial.print('+');
  } else {
    Serial.print('=');
  }
  for (byte i = 0; i < (rlen - 1); i++) {
    showHex(recvMessage[i], 1);
  }
  if (rlen == 4) {
    showHex(recvMessage[3] >> 4, 1);
  }
  Serial.println();
  displayPacketLCD(rlen, vers);
}

void displayPacketLCD(byte rlen, byte vers) {
  lcd.clear();
  if (readLDR() > 100) {
    lcd.backlight();
    backlightOnAt = millis();
    if (!backlightOnAt) backlightOnAt = 1; //handle millis()==0
  }
  if (rlen == 4) {
    lcd.setCursor(0, 0);
    lcd.print("Received:");
    lcd.setCursor(7, 1);
    lcd.print(vers ? "=" : "+");
    showHex(recvMessage[0], 2);
    lcd.print(":");
    showHex(recvMessage[1], 2);
    lcd.print(":");
    showHex(recvMessage[2], 2);
    lcd.print(":");
    showHex(recvMessage[3] >> 4, 2);
    //lcd.setCursor(7,3);
    //lcd.print(readLDR());
  } else if (rlen == 8 || rlen == 16) {
    lcd.setCursor(0, 0);
    lcd.print("Received:");
    lcd.setCursor(2, 1);
    lcd.print(vers ? '=' : '+');
    for (byte i = 0; i < (rlen == 8 ? 7 : 8); i++) {
      showHex(recvMessage[i], 2);
    }
    if (rlen == 16) {
      lcd.setCursor(2, 2);
      for (byte i = 8; i < 15; i++) {
        showHex(recvMessage[i], 2);
      }
    }
  } else {
    lcd.setCursor(0, 0);
    lcd.print("RX ");
    lcd.print(vers ? '=' : '+');
    for (byte i = 0; i < (rlen == 8 ? 7 : 8); i++) {
      showHex(recvMessage[i], 2);
    }
    lcd.setCursor(4, 1);
    for (byte i = 8; i < 16; i++) {
      showHex(recvMessage[i], 2);
    }
    lcd.setCursor(4, 2);
    for (byte i = 16; i < 24; i++) {
      showHex(recvMessage[i], 2);
    }
    lcd.setCursor(4, 3);
    for (byte i = 24; i < 31; i++) {
      showHex(recvMessage[i], 2);
    }
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
    unsigned long t = (millis() - lastPacketTime);
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
  TIMSK |= 1 << ICIE1;
  return;
}

byte checkCSC() {
  byte rxchecksum = 0;
  byte rxchecksum2 = 0;
  byte rxc2;
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
  unsigned long lastpacketsig = 0;
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
#ifdef USE_ACO
  TCCR1B = (!state) ? (1 << CS11 | 1 << ICNC1) : (1 << CS11 | 1 << ICNC1 | 1 << ICES1); //and set edge
#else
  TCCR1B = state ? (1 << CS11 | 1 << ICNC1) : (1 << CS11 | 1 << ICNC1 | 1 << ICES1); //and set edge
#endif
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
        TIMSK &= ~(1<<ICIE1); //turn off input capture;

      } else {
        bitnum++;
      }
    }
  }
}


int readLDR() {
  int retval = 0;
  analogRead(1);
  for (byte i = 0; i < 4; i++) {
    retval += analogRead(1);
  }
  //lcd.print(retval);
  return retval;
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

  if (c == 2) {
    delay(10);
    lcd.print(buf);
  }  else if (c == 1) {
    Serial.print(buf);
  }
  else {
    //xSerialDbg.print(buf);
  }
}
