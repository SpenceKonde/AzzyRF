#define ListenST 1
#define CommandST 2
#define rxpin 0
#define rpin 5
#define gpin 6
#define bpin 7
#define wpin 8


unsigned char MyAddress=31;

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
int count=0;
int badcsc=0;

unsigned char MyCmd;
unsigned char MyParam;
unsigned char MyExtParam;

void setup() {
        lastPinState=0;
        lastPinLowTime=0;
        lastPinHighTime=0;
        rxdata=0;
        bitsrx=0;
        rxing=0;
        MyState=1;
	//Serial.begin(9600);
}

void loop() {
        curTime=micros();
        if (MyState==1) {
          onListenST();
        } else {
          onCommandST();
        }
}

void onCommandST() {
  if (MyCmd==0x30) {
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
    //Serial.println("Assigned brightness");
    doLatch();
    //Serial.println("Returning to Listen");
    MyState=1;
  } else {
    //Serial.println("invalid command");
    MyState=1;
  }
}

void doLatch() {
  analogWrite(rpin,RBright);
  analogWrite(gpin,GBright);
  analogWrite(bpin,BBright);
  analogWrite(wpin,WBright);
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
				//Serial.print("Got 32 bits ");
				//Serial.print((rxdata>>24)&0xFF);
				//Serial.print(":");
				//Serial.print((rxdata>>16)&0xFF);
				//Serial.print(":");
				//Serial.print((rxdata>>8)&0xFF);
				//Serial.print(":");
				//Serial.println((rxdata)&0xFF);
				//Serial.println(rxdata);
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
                        //Serial.println(bitsrx);
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
}
void parseRx(unsigned long rxd) {
	//Serial.println("Parsing: ");
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
			//Serial.println("Valid transmission received!");
			MyCmd=rcvCmd;
			MyParam=rcvParam;
			MyExtParam=rcvExtParam;
			MyState=CommandST;
		} else {
			//Serial.println("Wrong address - not for me");
		}
	} else {
                //Serial.print(calccsc);
		//Serial.println("No good!");
                badcsc++; 
	}
}
