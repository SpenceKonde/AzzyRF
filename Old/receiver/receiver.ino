#define ListenST 1
#define CommandST 2
#define FadeST 2
#define rxpin 8
#define rpin 9
#define gpin 10
#define bpin 11
#define wpin 6
#define tpin 17
#define lpin 16
#define errpin 13
#include <EEPROM.h>
#define erroron 0
#define erroroff 1
#define VERS 1

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
        MyAddress=EEPROM.read(0);
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
        delay(500);
        doLatch();
}



void loop() {
	if (rxing==0) { //analog reads are too slow for receiving to work
		int temp=analogRead(tpin);
		tempavg=(tempavg*9+temp)/10; //this rolling average smooths out the analog inputs, otherwise they jitter. 
		lasttempavg=tempavg;
		int light=analogRead(lpin);
		lightavg=(lightavg*9+light)/10;
		lastlightavg=lightavg;
	}
	if (tempavg > maxtemp && millis() > tempcheck) { //make sure it's not overheating. Check vs tempcheck is to give the device a chance to cool down after overheating first detected.
		handleOvertemp();
	} 
	if (millis() > tempcheck) { //other behavior disabled when overheated. 
		tempcheck=0;
		curTime=micros();
		//Start state functions
		if (MyState==1) {
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
		if (ForgetCmdAt && ForgetCmdAt < millis()) {
		    Serial.println("clearing cmd record");
			ClearCMD();
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
	}
	if (MyCmd==0x31||(MyCmd==0x32&&MyParam==0)) {
		Serial.println("Latching immediately");
		doLatch();
	} else if (MyCmd==0x32) {
                unsigned long latchdelay=MyParam;
		LatchAt=millis()+(latchdelay<<(4+MyExtParam));
	} else if (MyCmd==0x34) {
		if (MyExtParam&1) {
    		analogWrite(rpin,constrain(RBright,0,MyParam));
    	}
    	if (MyExtParam&2) {
    		analogWrite(gpin,constrain(GBright,0,MyParam));
    	}
    	if (MyExtParam&4) {
    		analogWrite(bpin,constrain(BBright,0,MyParam));
    	}
		if (MyExtParam&8) {
    		analogWrite(wpin,constrain(WBright,0,MyParam));
    	}
    	Serial.println("Suspending some channels...");
	} else if (MyCmd==0xF0) {
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
	} else if (MyCmd==0xF1) {
		Serial.println("checking flash");
		if (EEPROM.read(0x10+MyExtParam) != MyParam) {
			Serial.println("writing flash");
			EEPROM.write(0x10+MyExtParam,MyParam);
			if (MyExtParam==0) {

			}
		}
	}
	ClearCMD();
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

void allOff() {
	analogWrite(rpin,0);
	analogWrite(gpin,0);
	analogWrite(bpin,0);
	analogWrite(wpin,0);
}
void loadLED() {
	RBright=EEPROM.read(0x1C);
	GBright=EEPROM.read(0x1D);
	BBright=EEPROM.read(0x1E);
	WBright=EEPROM.read(0x1F);
}

void handleLightLevel() {
	if (lightavg>>2 > EEPROM.read(0x14) && lastlightavg>>2 <= EEPROM.read(0x14)) {
		if (EEPROM.read(0x15)&3==1){
			allOff();
		} else if (EEPROM.read(0x15)&3==2) {
			doLatch();
		} else if (EEPROM.read(0x15)&3==3) {
			
			doLatch();
		}
	} else if (lightavg>>2 > EEPROM.read(0x16) && lastlightavg>>2 <= EEPROM.read(0x16)) {
		if (EEPROM.read(0x17)&3==1){
			allOff();
		} else if (EEPROM.read(0x17)&3==2) {
			doLatch();
		} else if (EEPROM.read(0x17)&3==3) {
			loadLED();
			doLatch();
		}
	} else if (lightavg>>2 > EEPROM.read(0x18) && lastlightavg>>2 <= EEPROM.read(0x18)) {
		if (EEPROM.read(0x19)&3==1){
			allOff();
		} else if (EEPROM.read(0x19)&3==2) {
			doLatch();
		} else if (EEPROM.read(0x19)&3==3) {
			loadLED();
			doLatch();
		}
	} else if (lightavg>>2 < EEPROM.read(0x18) && lastlightavg>>2 >= EEPROM.read(0x18)) {
		if (EEPROM.read(0x1A)&3==1){
			allOff();
		} else if (EEPROM.read(0x1A)&3==2) {
			doLatch();
		} else if (EEPROM.read(0x1A)&3==3) {
			loadLED();
			doLatch();
		}
	}
}

void ClearCMD() {
	if (MyCmd) {
		ForgetCmdAt=millis()+10000;
	} else {
		ForgetCmdAt=0;
	}
	MyLastCmd=MyCmd;
	MyLastParam=MyParam;
	MyLastExtParam=MyExtParam;
	MyParam=0;
	MyExtParam=0;
	MyCmd=0;
}

void doLatch() {
  analogWrite(rpin,RBright);
  analogWrite(gpin,GBright);
  analogWrite(bpin,BBright);
  analogWrite(wpin,WBright);
  LatchAt=0;
}

void onListenST() {
	int pinState=digitalRead(rxpin);
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
			if (bitlength > 475 && bitlength < 650) {
				rxdata=rxdata<<1;
				//Serial.print("F");
                //Serial.println(bitlength);
			} else if (bitlength > 1000 && bitlength < 1150 ) {
				rxdata=(rxdata<<1)+1;
				//Serial.print("T");
                //Serial.println(bitlength);
			} else {
				resetListen();
				return;
			}
			bitsrx++;
			if (bitsrx==32) {
				Serial.print("Got 32 bits ");
				Serial.print((rxdata>>24)&0xFF);
				Serial.print(":");
				Serial.print((rxdata>>16)&0xFF);
				Serial.print(":");
				Serial.print((rxdata>>8)&0xFF);
				Serial.print(":");
				Serial.println((rxdata)&0xFF);
				Serial.println(rxdata);
				parseRx(rxdata);
				resetListen();
			}
			return;
		}   
	} else {
		lastPinHighTime=curTime;
                if (lastPinHighTime-lastPinLowTime > 1900 && lastPinHighTime-lastPinLowTime <2100 && rxing==0) {
                  rxing=1;
                  return;
                }
		if (lastPinHighTime-lastPinLowTime > 800 && rxing==1) {
            Serial.println(bitsrx);
			resetListen();
			return;
		}
	}
}

void resetListen() {
	bitsrx=0;
	rxdata=0;
    rxing=0;
}
void parseRx(unsigned long rxd) {
    //Serial.println(tempavg);
    //Serial.println(maxtemp);
	Serial.println("Parsing: ");
	unsigned char rcvB0=(rxd>>24)&0xFF;
	unsigned char rcvCmd=(rxd>>16)&0xFF;
	unsigned char rcvParam=(rxd>>8)&0xFF;
	unsigned char rcvB3=(rxd)&0xFF;
	unsigned char rcvAdd=rcvB0&0x3F;
	unsigned char rcvExtParam=(rcvB3>>4);
	unsigned char calccsc=(rcvB0^rcvCmd^rcvParam);
	calccsc=(calccsc&15)^(calccsc>>4)^rcvExtParam;
	rcvB3=rcvB3&15;
	if (calccsc==rcvB3) {
		if (rcvAdd==MyAddress) {
			Serial.println("Valid transmission received!");
			if (rcvCmd!=MyLastCmd||rcvParam!=MyLastParam||rcvExtParam!=MyLastExtParam) {
				byte cmdaddr=rcvCmd/8;

				if (((EEPROM.read(0xE0+cmdaddr)>>(rcvCmd%8))^1)&1) {
					Serial.print(rcvCmd);
					Serial.println(MyLastCmd);
					Serial.print(rcvParam);
					Serial.println(MyLastParam);
					Serial.print(rcvExtParam);
					Serial.println(MyLastExtParam);
					MyCmd=rcvCmd;
					MyParam=rcvParam;
					MyExtParam=rcvExtParam;
					MyState=CommandST;
				} else {
					Serial.println("Unknown Command");
					Serial.println(cmdaddr);
					Serial.println(EEPROM.read(0xE0+cmdaddr));
					Serial.println(((EEPROM.read(0xE0+cmdaddr)>>(rcvCmd%8))^1));
					digitalWrite(errpin, erroron);
					turnOffErrorAt=millis()+500;
				}
			} else {
				Serial.println("but we already got it");
			} 
		} else {
			Serial.println("Wrong address - not for me");
		}
	} else {
        Serial.print(calccsc);
		Serial.println("No good!");
        badcsc++; 
	}
}
