/*
Remote control outlet driver

This accepts commands 0x40 and 0x41 (Digital Set and Advanced Digital Set) over RF, controlling 3 relays and one 5V PWM output, using protocol v2.1

*/

#define ListenST 1
#define CommandST 2
#include <EEPROM.h>

#define rxpin 3
#define rxPIN PINB
#define rxBV 8
#define CommandForgetTime 10000 

#define btn1 15
#define btn2 14
#define btn3 17
#define btn4 16
#define rcvled 3
#define rxmaxlen 64

//These set the parameters for transmitting. 

//These set the parameters for receiving; any packet where these criteria are not met is discarded. 
// Version 2.1
unsigned int rxSyncMin  = 1750;
unsigned int rxSyncMax  = 2250;
unsigned int rxZeroMin  = 100;
unsigned int rxZeroMax  = 390;
unsigned int rxOneMin  = 410;
unsigned int rxOneMax  = 700;
unsigned int rxLowMax  = 600;

unsigned long units[]={1000,60000,900000,14400000,3600000,1,10,86400000}; //units for the 8/12/16-bit time values. 


byte MyAddress=20;


//Pin state tracking and data for receiving. 
byte lastPinState;
unsigned long lastPinHighTime;
unsigned long lastPinLowTime;
unsigned long lastTempHighTime=0;
unsigned long lastTempLowTime=0;
byte rxdata;
byte lastTempPinState;
byte bitsrx;
byte rxing;
byte rxaridx;
unsigned char txrxbuffer[rxmaxlen>>3];

byte MyState;
unsigned char MyCmd;
unsigned char MyParam;
unsigned char MyExtParam;
unsigned long curTime;
int count=0;
int badcsc=0;
int pksize=32;
//byte TXLength;
unsigned long lastChecksum; //Not the same as the CSC - this is our hack to determine if packets are identical
unsigned long forgetCmdAt; 
//int reccount;

// digital out stuff

// format: (enable)(pwm)pin - enable = 0, disable =1 (so unprogrammed eeprom reads as disabled) 
//byte  digOutOpts[16]={0x09,0x0A,0x0B,0x43,255,255,255,255,255,255,255,255,255,255,255,255};


unsigned long digOutOnAt[4]={0,0,0,0};
unsigned long digOutOffAt[4]={0,0,0,0};
unsigned long digOutTimer[4]={0,0,0,0};  //cycle time for blinking, fade step size for fading
byte digOutOnPWM[4]={255,255,255,255};
byte digOutOffPWM[4]={0,0,0,0};
byte digOutFade[4]={0,0,0,0};
byte digOutMode[4]={0,0,0,0};
//byte last1;
//byte last2;
unsigned long test=0;



void setup() {
	//MyAddress=EEPROM.read(0)&0x3F;
	lastPinState=0;
	lastPinLowTime=0;
	lastPinHighTime=0;
	rxdata=0;
	bitsrx=0;
	rxing=0;
	MyState=ListenST;
	//pinMode(btn1,INPUT_PULLUP); //buttons for testing
	//pinMode(btn2,INPUT_PULLUP);
	//pinMode(btn3,INPUT_PULLUP);
	//pinMode(btn4,INPUT_PULLUP);
	//Serial.begin(9600);
	for (byte i=0;i<4;i++) {
		byte opt=EEPROM.read(i+0x20);

		if (opt<128) {
			if (opt>64) {
				analogWrite(opt&0x3F,EEPROM.read(i+0x10));
			} else {
				digitalWrite(opt&0x3F,EEPROM.read(i+0x10)>>7);
  				pinMode(opt&0x3F,OUTPUT);
			}
		}
	}
	//Serial.println("Startup OK");

}


void loop() {
	curTime=micros();
	if (MyState==ListenST) {
		ClearCMD(); //do the command reset only if we are in listenst but NOT receiving.
		onListenST();
		if (rxing==1){
			return; //don't do anything else while actively receiving.
		}
	} else if (MyState==CommandST){ 
		onCommandST();

	} else {
		MyState=ListenST; //in case we get into a bad state somehow.
	}
//	handleButtons();
	handleDigOut();
}

void handleDigOut() {
	unsigned long curMillis=millis();
	for (byte i=0;i<4;i++) {
		//byte opt=digOutOpts[i];
		byte opt=EEPROM.read(i+0x20);
		if (digOutMode[i]&2){
			if (((digOutMode[i]&4)==4) && curMillis>digOutOffAt[i]) {
				////Serial.println("Reverse fade active");
				if (digOutFade[i]>digOutOffPWM[i] ){
					digOutFade[i]--;
					analogWrite(opt&0x3F,digOutFade[i]);
					digOutOffAt[i]=curMillis+digOutTimer[i];
				} else {
					if (digOutMode[i]&8) {
						digOutMode[i]=2;
						digOutOnAt[i]=curMillis+digOutTimer[i];
					} else if (digOutMode[i]&1) {
						digOutMode[i]=3;
						digOutOnAt[i]=curMillis+digOutTimer[i];
					}
				}
			} else if (((digOutMode[i]&4)==0) && curMillis>digOutOnAt[i]) {
				////Serial.println("Forward fade active");
				if (digOutFade[i]<digOutOnPWM[i] ){
					digOutFade[i]++;
					analogWrite(opt&0x3F,digOutFade[i]);
					digOutOnAt[i]=curMillis+digOutTimer[i];
				} else {
					if (digOutMode[i]&8) {
						digOutMode[i]=6;
						digOutOffAt[i]=curMillis+digOutTimer[i];
					} else if (digOutMode[i]&1) {
						digOutMode[i]=7;
						digOutOffAt[i]=curMillis+digOutTimer[i];
					}
				}
			}
		} else if (digOutOnAt[i] && curMillis > digOutOnAt[i] ) {
			digOutOnAt[i]=digOutTimer[i]*(digOutMode[i]&1)+millis();
			analogWrite(opt&0x3F,digOutOnPWM[i]);
			////Serial.println("Delayed turnon");
		} else if (digOutOffAt[i] && curMillis > digOutOffAt[i]) {
			digOutOffAt[i]=digOutTimer[i]*(digOutMode[i]&1)+millis();
			analogWrite(opt&0x3F,digOutOffPWM[i]);
			////Serial.println("Delayed turnoff");
		}
	}

}


void onCommandST() {
	//Serial.print("onCommandST");
	//Serial.println(MyCmd);
	//Serial.println("and another test");
	switch (MyCmd) {
	//EEPROM.write(0x36,MyCmd);
	case 0x40: {
		//byte opt=digOutOpts[MyExtParam];
		byte opt=EEPROM.read(0x20+MyExtParam);
		//Serial.print(opt);
		//Serial.println("Set command received");
		if (opt < 128) {
			if (opt < 64){ //no pwm
 				//EEPROM.write(0x30,MyParam);
				if (MyParam>127){
  					digitalWrite(opt&0x3F,1);
  				} else {
    				digitalWrite(opt&0x3F,0);
    				}
				//Serial.println("Digital set");
			} else {
 				//EEPROM.write(0x31,MyParam);
				analogWrite(opt&0x3F,MyParam);
				//Serial.println("Analog set");
			}
			//EEPROM.write(0x32,0);
		} 
		digOutOffAt[MyExtParam]=0;
		digOutMode[MyExtParam]=0;
		digOutOnAt[MyExtParam]=0;
		MyState=ListenST;
		break;
	}
	case 0x41: {
		byte pinid=MyParam>>4;
		//Serial.println("Advanced set:");
		//byte opt=digOutOpts[pinid];
		byte opt=EEPROM.read(0x20+pinid);
			//Serial.print("opt is");
			//Serial.println(opt);
		if (opt < 128) {
			digOutOffAt[pinid]=0;
			digOutMode[pinid]=0;
			digOutOnAt[pinid]=0;
			byte digpin=opt&0x3F;
			//if (opt < 64){ //no pwm
			//Serial.print("digital pin is");
			//Serial.println(digpin);
			//Serial.print("MyParam&0x0F is");
			//Serial.println(MyParam&0x0F);
			//switch-case on low 4 bits of the parameter, which specifies the type of operation. 
			//Note the extensive use of fall-through in the interest of code reuse; many of the commands are similar to eachother.
			switch (MyParam&0x0F) {
				case 1:
					if (opt>63) {
						analogWrite(opt&0x3F,txrxbuffer[6]);
						//Serial.println("Set delay set - analog");
					} else {
						digitalWrite(opt&0x3F,txrxbuffer[6]>>7);
						//Serial.println("Set delay set - digital");
					}
					/* falls through */
				case 0:
					if (txrxbuffer[5]>127) {
						digOutOnAt[pinid]=decode16(txrxbuffer[4]+(txrxbuffer[3]<<8))+millis();
						digOutOnPWM[pinid]=txrxbuffer[5];
						//Serial.println(" delay set - on");
						//Serial.println(decode16(txrxbuffer[4]+(txrxbuffer[3]<<8)));
					} else {
						digOutOffAt[pinid]=decode16(txrxbuffer[4]+(txrxbuffer[3]<<8))+millis();
						digOutOffPWM[pinid]=txrxbuffer[5];
						//Serial.println(" delay set - off");
						//Serial.println(decode16(txrxbuffer[4]+(txrxbuffer[3]<<8)));
					}
					break;
				case 4:
					digOutTimer[pinid]=decode16(txrxbuffer[4]+(txrxbuffer[3]<<8))+decode16(txrxbuffer[6]+(txrxbuffer[5]<<8)); //repeat (blink), this is sum of both times provided;
					digOutMode[pinid]=1; //set mode to repeat
					/* falls through */
				case 2:
					digOutOnAt[pinid]=decode16(txrxbuffer[4]+(txrxbuffer[3]<<8))+millis();  //set it to go on after the first delay supplied
					digOutOffAt[pinid]=decode16(txrxbuffer[6]+(txrxbuffer[5]<<8))+digOutOnAt[pinid]; //and off after the second delay supplied. 
					break;
				case 5:
					digOutTimer[pinid]=decode16(txrxbuffer[4]+(txrxbuffer[3]<<8))+decode16(txrxbuffer[6]+(txrxbuffer[5]<<8)); //repeat, inverted
					digOutMode[pinid]=1;
					/* falls through */
				case 3:
					digOutOffAt[pinid]=decode16(txrxbuffer[4]+(txrxbuffer[3]<<8))+millis();  //set it to go off after the first delay supplied
					digOutOnAt[pinid]=decode16(txrxbuffer[6]+(txrxbuffer[5]<<8))+digOutOffAt[pinid]; //and onn after the second delay supplied. 
					break;
				case 10:
					digOutMode[pinid]=1;
					/* falls through */
				case 9:
					if (digOutMode[pinid]==0){ //make sure we haven't already set digOutMode (by fall through)
						digOutMode[pinid]=8;
					}
					/* falls through */
				case 8: //fade
					if (opt<64) {
						////Serial.print(opt);
						//Serial.println("Fade not supported on non-PWM pins");
						digOutMode[pinid]=0;
						break;
					}
					digOutFade[pinid]=txrxbuffer[3];
					if (txrxbuffer[3]<txrxbuffer[6]) {
						digOutOffPWM[pinid]=txrxbuffer[3];
						digOutOnPWM[pinid]=txrxbuffer[6];
						digOutMode[pinid]=2|digOutMode[pinid];
						digOutOnAt[pinid]=millis()+digOutTimer[pinid];
						//Serial.print("digOutOnAt (for fade) set: ");
						//Serial.println(digOutOnAt[pinid]);
					} else {
						digOutOnPWM[pinid]=txrxbuffer[3];
						digOutOffPWM[pinid]=txrxbuffer[6];
						digOutMode[pinid]=6|digOutMode[pinid];
						digOutOffAt[pinid]=millis()+digOutTimer[pinid];
						//Serial.print("digOutOffAt (for fade) set: ");
						//Serial.println(digOutOffAt[pinid]);
					}
					digOutTimer[pinid]=decode16(txrxbuffer[5]+(txrxbuffer[4]<<8))/(digOutOnPWM[pinid]-digOutOffPWM[pinid]);
					//Serial.print("digOutOnPWM (for fade) set: ");
					//Serial.println(digOutOnPWM[pinid]);
					//Serial.print("digOutOffPWM (for fade) set: ");
					//Serial.println(digOutOffPWM[pinid]);
					//Serial.print("digOutMode (for fade) set: ");
					//Serial.println(digOutMode[pinid]);
					//Serial.print("digOutTimer (for fade) set: ");
					//Serial.println(digOutTimer[pinid]);
					analogWrite(digpin,txrxbuffer[3]);
					break;
				//default:
					//Serial.println("Invalid parameter");
			}
			//} else {
			//	digOutState[MyExtParam]=MyParam;
			//	analogWrite(opt&0x3F,MyParam);
			//	//Serial.println("Analog set");
			//}
		} //else {
			//Serial.println("Invalid output");
		//}
	}
	MyState=ListenST;
	break;
	//case 0xF1: {
	//	//Serial.println("checking flash");
	//	if ((EEPROM.read(0x10+MyExtParam) != MyParam)&&MyExtParam<16&&MyCmd==0xF1 ) {
	//		EEPROM.write(0x10+MyExtParam,MyParam);
	//	}
	//}
		/* falls through */
	default:
		//Serial.print("Invalid command type");
		//Serial.println(MyCmd);
		MyState=ListenST;
	}
}

void onListenST() {
	#ifdef rxPIN
  byte pinState = (rxPIN & rxBV) ? 1 : 0;
#else
  byte pinState = digitalRead(rxpin);
#endif
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
				if (pksize>rxmaxlen) {
					//Serial.println("Packet this size not supported");
					resetListen();
					return;
				}
			} else if (bitsrx==8*(1+rxaridx)) {
				txrxbuffer[rxaridx]=rxdata;
				rxdata=0;
				rxaridx++;
				if (rxaridx*8==pksize) {
					//Serial.println("Receive done");
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
	//Serial.println("Parsing");
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
		    		//Serial.println(MyCmd);
		    		//Serial.println(MyParam);
		    		//Serial.println(MyExtParam);
		    		//Serial.println("Valid transmission received");
		    		//EEPROM.write(0x38,MyCmd);
	    		} else {
	    			//EEPROM.write(0x39,0);
	    			//Serial.println("Bad CSC on 4 byte packet");
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
					//Serial.println(MyCmd);
					//Serial.println(MyParam);
					//Serial.println(MyExtParam);
					//Serial.println("Valid long transmission received");
				} else {
					//Serial.println("Bad CSC on long packet");
				}  
			}
		} //else {
			//Serial.println("Already got it");
		//} 
	} //else {
	    //EEPROM.write(0x39,0);
		//Serial.println("Not for me");
	//}
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

/*
//Just for test/debug purposes;
void parseRx2(unsigned char rxd[],byte len) {

	//Serial.println("Parsing long packet");
	for (byte i=0;i<len;i++) {
		//Serial.println(rxd[i]);
	}
	//Serial.println("Done");
}
*/

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
