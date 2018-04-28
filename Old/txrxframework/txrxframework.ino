#define ListenST 1
#define ListenST2 4
#define CommandST 2
#define TransmitST 3
#include <EEPROM.h>

//char tpin=17;


#define rxpin 19
#define txpin 18


unsigned char MyAddress=31;
int lastPinState;
unsigned long lastPinHighTime;
unsigned long lastPinLowTime;;
unsigned long lastTempHighTime=0;
unsigned long lastTempLowTime=0;
unsigned long rxdata;
unsigned long rxdata2[8];
unsigned char txpayload[32];
int lastTempPinState;
int bitsrx;
int rxing;
int MyState;
unsigned char MyCmd;
unsigned char MyParam;
unsigned char MyExtParam;
unsigned long curTime;
int count=0;
int badcsc=0;
int pksize=32;
char rxaridx;


void setup() {
        lastPinState=0;
        lastPinLowTime=0;
        lastPinHighTime=0;
        rxdata=0;
        bitsrx=0;
        rxing=0;
        MyState=2;
	Serial.begin(9600);
}

void loop() {
        curTime=micros();
	if (MyState==ListenST) {
		onListenST();
	} else if (MyState==CommandST){ 
		onCommandST();
	//} else if (MyState==TransmitST){
	//	onTransmitST();
	} else {
		MyState=ListenST; //in case we get into a bad state somehow.
	}
}

void onCommandST() {
	if (MyCmd==0xF2) {
		Serial.println("Starting transmit");
		onTransmit();
                ClearCMD();
                MyState=1;
	} else if (MyCmd==0xF4) {
		Serial.println("Starting transmit");
                Serial.print(MyParam);
                Serial.print(" byte payload");
		onTransmit3(MyParam,MyExtParam);
                ClearCMD();
                MyState=1;
	} else {
		Serial.println("Invalid command type");
		ClearCMD();
		MyState=1;
	}
}

void onTransmit() {
	unsigned char Payload1=EEPROM.read(MyParam);
	unsigned char Payload2=EEPROM.read(MyParam+MyExtParam);
	unsigned char oldcsc=((MyAddress&0xF0)>>4)^(MyAddress&0x0F)^(0x0F)^(0x02)^((MyParam&0xF0)>>4)^(MyParam&0x0F)^(MyExtParam&0x0F);
	delay(1500);
        Serial.println(Payload1);
        Serial.println(Payload2);
        Serial.println(oldcsc);
        Serial.println("sendCommand() called!");
	sendCommand(159,Payload1,Payload2,oldcsc);
}

void onTransmit3(int len,int rep) {
	delay(1500);
	Serial.println("Transmit 3 called...");
	byte payload[len];
	for (byte i=0;i<len;i++) {
		payload[i]=100+i;
		Serial.print("B");
		Serial.print(i);
		Serial.print(": ");
		Serial.println(payload[i]);
	}
	Serial.println("Payload generated.");
	for (byte r=0;r<rep;r++) {
		//now we start sending;
		Serial.print("Sending round ");
		Serial.println(r);
		delay(10);
		for (byte j=0; j < 30; j++) {
			delayMicroseconds(400);
			digitalWrite(txpin, 1);
			delayMicroseconds(400);
			digitalWrite(txpin, 0);
		}
		delayMicroseconds(2000);
		for (byte k=0;k<len;k++) {
			//send a byte
			for (int m=7;m>=0;m--) {
				digitalWrite(txpin, 1);
				if ((payload[k]>>m)&1) {
					delayMicroseconds(1100);
				} else {
					delayMicroseconds(600);
				}
				digitalWrite(txpin, 0);
				delayMicroseconds(650);
			}
			//done with that byte
		}
		//done with sending this packet;
		digitalWrite(txpin, 0); //make sure it's off;
		delayMicroseconds(2000); //wait 2ms before doing the next round. 
	}
	Serial.println("Transmit done");
}



void ClearCMD() {
	MyParam=0;
	MyExtParam=0;
	MyCmd=0;
}

void sendCommand(unsigned int add,unsigned int cmd, unsigned int param, unsigned char ext){
	unsigned char csc=((add&0xF0)>>4)^(add&0x0F)^((cmd&0xF0)>>4)^(cmd&0x0F)^((param&0xF0)>>4)^(param&0x0F)^(ext&0x0F);
	unsigned long data=(add << 8) + cmd;
	data=(data<<16)+(param<<8)+((ext&0x0F)<<4)+csc;
        Serial.println(data);
	for (byte k=0;k<5;k++) {
		for (byte j=0; j < 30; j++) {
			delayMicroseconds(400);
			digitalWrite(txpin, 1);
			delayMicroseconds(400);
			digitalWrite(txpin, 0);
		}
	        
		delayMicroseconds(2000);
		for (int i=31; i>=0; i--) {
			digitalWrite(txpin, 1);
			if ((data>>i)&1) {
				delayMicroseconds(1100);
			} else {
				delayMicroseconds(600);
			}
			digitalWrite(txpin,0);
			delayMicroseconds(650);
		}
		delay(1);
	}
        Serial.println("Done sending!");
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
        //Serial.println("bitlength:");
        //if (bitlength < 1400 && bitlength > 600) {
        //Serial.println(bitlength);
        //}
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
			if (bitsrx==2) {
				pksize=32<<rxdata;
			} else if (bitsrx==32*(1+rxaridx)) {
				rxdata2[rxaridx]=rxdata;
				rxdata=0;
				rxaridx++;
				if (rxaridx*32==pksize) {
					Serial.println(bitsrx);
					Serial.println("Receive done");
					Serial.println(pksize);
					if (pksize==32) {
  						parseRx(rxdata2[0]);
  					}
					parseRx2(rxdata2,pksize/32);
					resetListen();
				}
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
        //Serial.print("end");
	bitsrx=0;
	rxdata=0;
    rxing=0;
    rxaridx=0;
}

void parseRx2(unsigned long rxd[],byte len) {
  
	Serial.println("Parsing long packet");
	for (byte i=0;i<len;i++) {
		for (int j=24;j>=0;j-=8){
			Serial.println((rxd[i]>>j)&0xff);
		}
	}
	Serial.println("Done");
}

void parseRx(unsigned long rxd) {
	Serial.println("Parsing: ");
        Serial.println(rxd);
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
                if (rcvCmd==200) {
      		    	Serial.print(count);
      		    	Serial.println(" received.");
      		    	count=0;
      		    	Serial.print(badcsc);
      		    	Serial.println(" .");
      		    	badcsc=0;
      	        } else {
      	        	count++;
			MyCmd=rcvCmd;
			MyParam=rcvParam;
			MyExtParam=rcvExtParam;
			MyState=CommandST;
      		    	Serial.println(rcvCmd);
      		    	Serial.println(rcvParam);
      		    	Serial.println(rcvExtParam);
      		    	Serial.println(MyState);

      	        }
		Serial.println("Valid transmission received!");
	} else {
        Serial.print(calccsc);
		Serial.println("No good!");
        badcsc++; 
	}
}
