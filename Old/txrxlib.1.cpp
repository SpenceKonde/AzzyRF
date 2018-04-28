#include "txrxlib.h"

#define rxpin 19
#define txpin 18
#define ListenST 1
#define CommandST 2

#define CommandForgetTime 1000


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
		if (rxing==1) {
			if (bitlength > rxZeroMin && bitlength < rxZeroMax) {
				rxdata=rxdata<<1;
			} else if (bitlength > rxOneMin && bitlength < rxOneMax ) {
				rxdata=(rxdata<<1)+1;
			} else {
  				//Serial.print("Reset wrong high len ");
  				//Serial.print(bitlength);
  				//Serial.print(" ");
  				//Serial.println(bitsrx);
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
			//Serial.println(bitsrx);
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
					Serial.println("Valid long transmission received");
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

//decode times 
unsigned long decode8(byte inp) { return (inp&0x3F)*units[inp>>6]; }
unsigned long decode12(unsigned int inp) { return (inp&0x01FF)*units[(inp>>9)&0x07]; }
unsigned long decode16(unsigned int inp) { return (inp&0x1FFF)*units[inp>>13]; }


void ClearCMD() {  //This handles clearing of the commands, and also clears the lastChecksum value, which is used to prevent multiple identical packets received in succession from being processed.
	if (MyCmd) {
		forgetCmdAt=millis()+CommandForgetTime;
		MyParam=0;
		MyExtParam=0;
		MyCmd=0;
	} else if (millis()>forgetCmdAt) {
		forgetCmdAt=0;
		lastChecksum=0;
	}
}
