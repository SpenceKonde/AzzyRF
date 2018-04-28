#define ListenST 1
#define CommandST 2
#define FadeST 2
#define rxpin 19

unsigned char MyAddress=91;

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
int MyState;
unsigned long curTime;

unsigned char MyCmdB1;
unsigned char MyCmdB2;

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
	onListenST();
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
			if (bitlength > 575 && bitlength < 750) {
				rxdata=rxdata<<1;
				Serial.print("F");
                Serial.println(bitlength);
			} else if (bitlength > 1200 && bitlength < 1350 ) {
				rxdata=(rxdata<<1)+1;
				Serial.print("T");
                Serial.println(bitlength);
			} else {
				resetListen();
				return;
			}
			bitsrx++;
			if (bitsrx==32) {
				Serial.print("Got 32 bits ");
				Serial.print((rxdata>>24)&0xFF)
				Serial.print(":");
				Serial.print((rxdata>>16)&0xFF)
				Serial.print(":");
				Serial.print((rxdata>>8)&0xFF)
				Serial.print(":");
				Serial.println((rxdata)&0xFF)
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
		if (lastPinHighTime-lastPinLowTime > 700 && rxing==1) {
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
void parseRx(rxd) {
	Serial.println("Parsing: ")
	unsigned char rcvAdd=(rxd>>24)&0xFF;
	unsigned char rcvB1=(rxd>>16)&0xFF;
	unsigned char rcvB2=(rxd>>8)&0xFF;
	unsigned char rcvCsc=(rxd)&0xFF;
	Serial.println(rcvAdd);
	Serial.println(rcvB1);
	Serial.println(rcvB2);
	Serial.println(rcvCsc);
	if ((rcvAdd+rcvB1+rcvB2)&0xFF==rcvCsc) {
		if (rcvAdd==MyAddress) {
			Serial.println("Valid transmission received!")
			MyCmdB1=rcvB1;
			MyCmdB2=rcvB2;
			//MyCmdB3=rcvCsc;
			//MyState=CommandST;
		} else {
			Serial.println("Wrong address - not for me")
		}
	} else {
		Serial.println("No good!")
	}
}