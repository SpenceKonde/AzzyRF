//These set the parameters for transmitting. 
#define txOneLength 1100 //length of a 1
#define txZeroLength 600 //length of a 0
#define txLowTime 650 //length of the gap between bits
#define txTrainRep 30 //number of pulses in training burst
#define txSyncTime 2000 //length of sync
#define txTrainLen 400 //length of each pulse in training burst

//These set the parameters for receiving; any packet where these criteria are not met is discarded. 
#define rxSyncMin 1900 //minimum valid sync length
#define rxSyncMax 2100 //maximum valid sync length
#define rxZeroMin 475 //minimum length for a valid 0
#define rxZeroMax 650 //maximum length for a valid 0
#define rxOneMin 1000 //minimum length for a valid 1
#define rxOneMax 1150 //maximum length for a valid 1
#define rxLowMax 800 //longest low before packet discarded

#define ListenST 1
#define CommandST 2
#define rxpin 8
#define txpin 7
#define rpin 10
#define gpin 9
#define bpin 11
#define wpin 6
#define tpin 17
#define lpin 16
#define errpin 12
#include <EEPROM.h>
#define erroron 0
#define erroroff 1
#define VERS 2

//char tpin=17;


unsigned char MyAddress=31;
unsigned char EVERS=0;
int RBright=0;
int GBright=0;
int BBright=0;
int WBright=0;
int lastPinState;
unsigned long lastPinHighTime;
unsigned long lastPinLowTime;;
unsigned long lastTempHighTime=0;
unsigned long lastTempLowTime=0;
unsigned long rxdata;
int lastTempPinState;
int bitsrx;
int rxing;
int MyState=ListenST;
unsigned long curTime;
unsigned long LatchAt;
unsigned long ForgetCmdAt;
unsigned long tempcheck=0;
unsigned long turnOffErrorAt=0;
int count=0;
int badcsc=0;
int maxtemp=300;
int tempavg=0;
int lightavg=0;
int lasttempavg=0;
int lastlightavg=0;

unsigned char MyCmd;
unsigned char MyParam;
unsigned char MyExtParam;
unsigned char MyLastCmd;
unsigned char MyLastParam;
unsigned char MyLastExtParam;

int pksize=32;
char rxaridx;
unsigned char txrxbuffer[32];
byte TXLength;
unsigned long lastChecksum; //Not the same as the CSC - this is our hack to determine if packets are identical
unsigned long forgetCmdAt; 

void setup() {
        lastPinState=0;
        lastPinLowTime=0;
        lastPinHighTime=0;
        LatchAt=0;
        rxdata=0;
        bitsrx=0;
        rxing=0;
        MyState=1;
		loadLED();
        MyAddress=EEPROM.read(0)&0x3F;
        EVERS=EEPROM.read(1);
        if (EEPROM.read(0x10) < 255) {
          maxtemp=EEPROM.read(0x10)*4+80;
        }
        pinMode(rxpin,INPUT);
        pinMode(tpin,INPUT);
        pinMode(errpin,OUTPUT);
        digitalWrite(errpin,erroroff);
        pinMode(rpin,OUTPUT);
        pinMode(gpin,OUTPUT);
        pinMode(bpin,OUTPUT);
        pinMode(wpin,OUTPUT);
	Serial.begin(9600);
        Serial.println("Startup");
        delay(500);
        doLatch();
        Serial.println("Startup");
}



void loop() {
	if (rxing==0) { //analog reads are too slow for receiving to work
		int temp=analogRead(tpin);
		tempavg=(tempavg*9+temp)/10; //this rolling average smooths out the analog inputs, otherwise they jitter. 
		lasttempavg=tempavg;
		int light=analogRead(lpin);
		lightavg=(lightavg*9+light)/10;
	}
	if (tempavg > maxtemp && millis() > tempcheck) { //make sure it's not overheating. Check vs tempcheck is to give the device a chance to cool down after overheating first detected.
		handleOvertemp();
	} 
	if (millis() > tempcheck) { //other behavior disabled when overheated. 
		tempcheck=0;
		curTime=micros();
		//Start state functions
		if (MyState==1) {
			ClearCMD();
			onListenST();
		} else if (MyState==2) {
			onCommandST();
		} else {
			Serial.println("Bad state - resetting");
			MyState=1;
		}
		//End state functions

		if (LatchAt && LatchAt < millis()) {
			Serial.println("Latching after delay");
			doLatch();
		}
		if (turnOffErrorAt && (turnOffErrorAt < millis())) {
			turnOffErrorAt=0;
			digitalWrite(errpin,erroroff);
		}
		if (EEPROM.read(0x1B)==1) {
			handleLightLevel();
		}
	}
}

void onCommandST() {
	if (MyCmd==0x30||MyCmd==0x31) {
    	if (MyExtParam&1) {
    		RBright=MyParam;
    	}
    	if (MyExtParam&2) {
    		GBright=MyParam;
    	}
    	if (MyExtParam&4) {
    		BBright=MyParam;
    	}
		if (MyExtParam&8) {
    		WBright=MyParam;
    	}
	} else if (MyCmd==0x34) {
		RBright=txrxbuffer[3];
		GBright=txrxbuffer[4];
		BBright=txrxbuffer[5];
		WBright=txrxbuffer[6];
	}
	if (MyCmd==0x31||(MyCmd==0x32&&MyParam==0)||(MyCmd==34&&MyParam==1)) {
		Serial.println("Latching immediately");
		doLatch();
	} else if (MyCmd==0x32) {
                unsigned long latchdelay=MyParam;
		LatchAt=millis()+(latchdelay<<(4+MyExtParam));
	} else if (MyCmd==0x33) {
		suspendPWM(MyParam,MyExtParam);
	} else if (MyCmd==0xF0) {
		if (MyParam==0) {
			Serial.println(tempavg);
			Serial.println(lightavg);
			Serial.println(VERS);
			Serial.println(EVERS);
			Serial.println(RBright);
			Serial.println(GBright);
			Serial.println(BBright);
			Serial.println(WBright);
			Serial.println(millis());
			Serial.println(LatchAt);
			Serial.println(ForgetCmdAt);
			Serial.println(turnOffErrorAt);
		} else {
			prepareStatusPayload();
			delay(500);
			doTransmit(10) ;// Do 10 reps because it's a short packet, and sometimes those don't read well. 
		}
	} else if (MyCmd==0xF1) {
		Serial.println("checking flash");
		if (EEPROM.read(0x10+MyExtParam) != MyParam) {
			Serial.println("writing flash");
			EEPROM.write(0x10+MyExtParam,MyParam);
		}
                TXLength=4;
                doTransmit(5); //Echo the packet back so master knows it's set right. 
	} else if (MyCmd==0x20) {
		prepareSensorPayload();
		Serial.print("del...");
		delay(500);
		Serial.println("ay");
		doTransmit(5);
		MyState=ListenST;
	}
	MyState=1;
}

void handleOvertemp() {
	RBright=RBright*0.8;
	BBright=BBright*0.8;
	GBright=GBright*0.8;
	WBright=WBright*0.8;
	doLatch();
	Serial.print(tempavg);
	Serial.println("Overtemperature!");
	digitalWrite(errpin, erroron);
	turnOffErrorAt=millis()+5000;
	tempcheck=turnOffErrorAt;
}

void prepareSensorPayload() {
	byte outval=0;
	if (MyParam==1) {
		outval=lightavg;
	} else {
		outval=tempavg-80;
	}
	txrxbuffer[0]=MyAddress; //this should be something else 
	txrxbuffer[1]=(outval>>2)&0xFF;
	txrxbuffer[2]=(outval)&0x03;
	txrxbuffer[3]=0;
    TXLength=4;
}

void loadLED() {
	RBright=EEPROM.read(0x1C);
	GBright=EEPROM.read(0x1D);
	BBright=EEPROM.read(0x1E);
	WBright=EEPROM.read(0x1F);
}

void suspendPWM(byte maxbright,byte colors) {
	if (colors&1) {
		analogWrite(rpin,constrain(RBright,0,maxbright));
	}
	if (colors&2) {
		analogWrite(gpin,constrain(GBright,0,maxbright));
	}
	if (colors&4) {
		analogWrite(bpin,constrain(BBright,0,maxbright));
	}
	if (colors&8) {
		analogWrite(wpin,constrain(WBright,0,maxbright));
	}
	Serial.println("Suspending some channels...");
}


void handleLightLevel() {
	lastlightavg=lightavg;
	if (lightavg>>2 > EEPROM.read(0x14) && lastlightavg>>2 <= EEPROM.read(0x14)) {
                Serial.println("light adjust 1");
		if (EEPROM.read(0x15)&3==1){
			suspendPWM(0,15);
		} else if (EEPROM.read(0x15)&3==3) {
			suspendPWM(EEPROM.read(0x15),15);
		} else if (EEPROM.read(0x15)&3==2) {
			loadLED();
			doLatch();
		}
	} else if (lightavg>>2 > EEPROM.read(0x16) && lightavg < EEPROM.read(0x14))  && (( lastlightavg>>2 <= EEPROM.read(0x16)) || lastlightavg>>2 >=EEPROM.read(0x14)) {
                Serial.println("light adjust 2");
		if (EEPROM.read(0x17)&3==1){
			suspendPWM(0,15);
		} else if (EEPROM.read(0x17)&3==2) {
			suspendPWM(EEPROM.read(0x15),15);
		} else if (EEPROM.read(0x17)&3==3) {
			loadLED();
			doLatch();
		}
	} else if (lightavg>>2 > EEPROM.read(0x18) && lightavg < EEPROM.read(0x16))  && (( lastlightavg>>2 <= EEPROM.read(0x18)) || lastlightavg>>2 >=EEPROM.read(0x16)) {
                Serial.println("light adjust 3");
		if (EEPROM.read(0x19)&3==1){
			suspendPWM(0,15);
		} else if (EEPROM.read(0x19)&3==2) {
			suspendPWM(EEPROM.read(0x15),15);
		} else if (EEPROM.read(0x19)&3==3) {
			loadLED();
			doLatch();
		}
	} else if (lightavg>>2 < EEPROM.read(0x18)  && (lastlightavg>>2 >=EEPROM.read(0x18)) {
                Serial.println("light adjust 4");
		if (EEPROM.read(0x1A)&3==1){
			suspendPWM(0,15);
		} else if (EEPROM.read(0x1A)&3==2) {
			suspendPWM(EEPROM.read(0x15),15);
		} else if (EEPROM.read(0x1A)&3==3) {
			loadLED();
			doLatch();
		}
	}
}


void doLatch() {
  analogWrite(rpin,RBright);
  analogWrite(gpin,GBright);
  analogWrite(bpin,BBright);
  analogWrite(wpin,WBright);
  LatchAt=0;
}

void doTransmit(int rep) { //rep is the number of repetitions
	byte txchecksum=0;
	for (byte i=0;i<TXLength-1;i++) {
		txchecksum=txchecksum^txrxbuffer[i];
	}
	if (TXLength==4) {
		txchecksum=(txchecksum&0x0F)^(txchecksum>>4)^((txrxbuffer[3]&0xF0)>>4);
		txrxbuffer[3]=(txrxbuffer[3]&0xF0)+(txchecksum&0x0F);
	} else {
		txrxbuffer[TXLength-1]=txchecksum;
	} 
	for (byte r=0;r<rep;r++) {;
		for (byte j=0; j < txTrainRep; j++) {
			delayMicroseconds(txTrainLen);
			digitalWrite(txpin, 1);
			delayMicroseconds(txTrainLen);
			digitalWrite(txpin, 0);
		}
		delayMicroseconds(txSyncTime);
		for (byte k=0;k<TXLength;k++) {
			//send a byte
			for (int m=7;m>=0;m--) {
				digitalWrite(txpin, 1);
				if ((txrxbuffer[k]>>m)&1) {
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
		delayMicroseconds(2000); //wait 2ms before doing the next round. 
	}
	Serial.println("Transmit done");
	TXLength=0;
}



void onListenST() {
	byte pinState=digitalRead(rxpin);
	if (pinState==lastPinState) {
		return;
	} else {
		lastPinState=pinState;
	}
	if (pinState==0) {
		lastPinLowTime=curTime;
		unsigned long bitlength=lastPinLowTime-lastPinHighTime;
		if (bitlength > 1500) {
			resetListen();
			return;
		} else if (rxing==1) {
			if (bitlength > rxZeroMin && bitlength < rxZeroMax) {
				rxdata=rxdata<<1;
			} else if (bitlength > rxOneMin && bitlength < rxOneMax ) {
				rxdata=(rxdata<<1)+1;
			} else {
				resetListen();
				return;
			}
			bitsrx++;
			if (bitsrx==2) {
				pksize=32<<rxdata;
			} else if (bitsrx==8*(1+rxaridx)) {
				txrxbuffer[rxaridx]=rxdata;
				rxdata=0;
				rxaridx++;
				if (rxaridx*8==pksize) {
					Serial.println("Receive done");
					parseRx();
					//parseRx2(txrxbuffer,pksize/8);
 					resetListen();
				}
			}
			return;
		}   
	} else {
		lastPinHighTime=curTime;
		if (lastPinHighTime-lastPinLowTime > rxSyncMin && lastPinHighTime-lastPinLowTime <rxSyncMax && rxing==0) {
			rxing=1;
			return;
		}
		if (lastPinHighTime-lastPinLowTime > rxLowMax && rxing==1) {
			Serial.println(bitsrx);
			resetListen();
			return;
		}
	}
}




void parseRx() { //uses the globals. 
	Serial.println("Parsing");
	unsigned char calccsc=0;
	unsigned char rcvAdd=txrxbuffer[0]&0x3F;
	if (rcvAdd==MyAddress) {
		if (lastChecksum!=calcBigChecksum(byte(pksize/8))) {
			lastChecksum=calcBigChecksum(byte(pksize/8));
		    if (pksize==32) { //4 byte packet
		    	calccsc=txrxbuffer[0]^txrxbuffer[1]^txrxbuffer[2];
		    	calccsc=(calccsc&15)^(calccsc>>4)^(txrxbuffer[3]>>4);
		    	if (calccsc==(txrxbuffer[3]&15)) {
		    		MyCmd=txrxbuffer[1];
		    		MyParam=txrxbuffer[2];
		    		MyExtParam=txrxbuffer[3]>>4;
		    		MyState=CommandST;
		    		Serial.println(MyCmd);
		    		Serial.println(MyParam);
		    		Serial.println(MyExtParam);
		    		Serial.println("Valid transmission received");
	    		} else {
	    			Serial.println("Bad CSC on 4 byte packet");
	    		}
			} else {
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
					Serial.println("Valid transmission received");
				} else {
					Serial.println("Bad CSC on long packet");
				}  
			}
		} else {
			Serial.println("Already got it");
		} 
	} else {
		Serial.println("Not for me");
	}
}

unsigned long calcBigChecksum(byte len) {
	unsigned long retval=0;
	for (byte i=0;i<len;i++) {
		retval+=(txrxbuffer[i]<<(i>>1));
	}
	return retval;
}

void resetListen() {
    bitsrx=0;
    rxdata=0;
    rxing=0;
    rxaridx=0;
}
void ClearCMD() {  //This handles clearing of the commands, and also clears the lastChecksum value, which is used to prevent multiple identical packets received in succession from being processed.
	if (MyCmd) {
		forgetCmdAt=millis()+10000;
		MyParam=0;
		MyExtParam=0;
		MyCmd=0;
	} else if (millis()>forgetCmdAt) {
		forgetCmdAt=0;
		lastChecksum=0;
	}
}
