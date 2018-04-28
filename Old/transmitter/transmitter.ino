#define ListenST 1
#define CommandST 2
#define FadeST 2
#define txpin 19
#include <EEPROM.h>


void setup() {
	Serial.begin(9600);
	pinMode(txpin,OUTPUT);
}

void loop() {
	delay(10000);
        Serial.println("sending command now this is a nice long message to keep the light on");
	sendCommand(31,0x31,0x40,0x0F);
}

void sendCommand(unsigned int add,unsigned int cmd, unsigned int param, unsigned char ext){
	unsigned char csc=((add&0xF0)>>4)^(add&0x0F)^((cmd&0xF0)>>4)^(cmd&0x0F)^((param&0xF0)>>4)^(param&0x0F)^(ext&0x0F);
	unsigned long data=(add << 8) + cmd;
	data=(data<<16)+(param<<8)+((ext&0x0F)<<4)+csc;
	for (byte j=0; j < 30; j++) {
		delayMicroseconds(400);
		digitalWrite(txpin, 1);
		delayMicroseconds(400);
		digitalWrite(txpin, 0);
	}
        
	delayMicroseconds(2000);
	for (int i=31; i>=0; i--) {
		if ((data>>i)&1) {
			digitalWrite(txpin, 1);
			delayMicroseconds(1100);
		} else {
			digitalWrite(txpin, 1);
			delayMicroseconds(600);
		}
		digitalWrite(txpin,0);
		delayMicroseconds(650);
	}
}

