#ifndef TXRXLIB
#define TXRXLIB

#include <Arduino.h>

int txOneLength; //length of a 1
int txZeroLength; //length of a 0
int txLowTime; //length of the gap between bits
int txTrainRep; //number of pulses in training burst
int txSyncTime; //length of sync
int txTrainLen; //length of each pulse in training burst

//These set the parameters for receiving; any packet where these criteria are not met is discarded. 
// Version 2.0
int rxSyncMin; //minimum valid sync length
int rxSyncMax; //maximum valid sync length
int rxZeroMin; //minimum length for a valid 0
int rxZeroMax; //maximum length for a valid 0
int rxOneMin; //minimum length for a valid 1
int rxOneMax; //maximum length for a valid 1
int rxLowMax; //longest low before packet discarded

unsigned long units[8]; //units for the 8/12/16-bit time values. 


unsigned char MyAddress;


//Pin state tracking and data for receiving. 
byte lastPinState;
unsigned long lastPinHighTime;
unsigned long lastPinLowTime;
unsigned long lastTempHighTime;
unsigned long lastTempLowTime;
unsigned char rxdata;
byte lastTempPinState;
int bitsrx;
int rxing;
char rxaridx;
unsigned char txrxbuffer[32];


int MyState;
unsigned char MyCmd;
unsigned char MyParam;
unsigned char MyExtParam;
unsigned long curTime;
int count;
int badcsc;
int pksize;
byte TXLength;
unsigned long lastChecksum; //Not the same as the CSC - this is our hack to determine if packets are identical
unsigned long forgetCmdAt; 
byte last1;
byte last2;
byte last3;
byte last4;
int reccount;

// digital out stuff

// format: (enable)(pwm)pin - enable = 0, disable =1 (so unprogrammed eeprom reads as disabled) 
byte  digOutOpts[16];


unsigned long digOutOnAt[4];
unsigned long digOutOffAt[4];
unsigned long digOutTimer[4];
byte digOutOnPWM[4];
byte digOutOffPWM[4];
byte digOutFade[4];
byte digOutMode[4];


void doTransmit(int rep);
void onListenST();
void parseRx();
unsigned long calcBigChecksum(byte len);
void resetListen();
unsigned long decode8(byte inp);
unsigned long decode12(unsigned int inp);
unsigned long decode16(unsigned int inp);
void ClearCMD();
#endif
