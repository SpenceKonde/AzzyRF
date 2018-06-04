# AzzyRF
433mhz OOK RF protocol for Arduino. 

No, these have not been converted into a library yet. 


### Hardware

These work with any of the cheap OOK RF modules - though the effectiveness of these varies greatly between modules. 

For best results, I recommend the RXB-12, RXB-14, or other device based on the Synoxo STN470/SYN480. These are under $2 on ebay, and outperform almost everything else. 

The quality of the transmitter matters much less - SYN115-based ones are excellent, but so are many of the really cheap ones with just a SAW filter. 

TX may use any pin for data. RX must use the input capture (ICP) or AIN pin soas to use the input capture functionality. 

### Packet 

Packets are either 4, 8, 16, or 32 bytes long. 

The length is set by the first two bits of the first byte. 

Each device is assigned an address (or address 0 to listen to all packets); the destination of a packet is specified by the other 6 bits of the first byte. 

The last byte of the 8, 16, and 32 byte packets is the checksum. For 4-byte packets, this is the last 4 bits. 

### Checksum
Since OOK receivers will spew gibberish when not receiving any real signal, it is important to ensure that the data received isn't garbage, and has been received correctly. A checksum is needed. In version 2.2, this was done by simply XORing the data. In version 2.3, the crc8 function provided by the avr standard libraries is used instead; this provides improved detection of errors. 

There are two complications:
* In 4-byte packets, the checksum is 4 bits, not 8. This is generated by xoring the high and low nybbles of the checksum of the first three bytes. 
* The checksum is used to differentiate between v2.2 and v2.3 packets. When a packet is sent or decoded, both checksums are calculated. If they are the same, 1 is added to the v2.3 checksum.

### Signal description

A packet is sent several times. Each packet has the following structure:
* A "training burst" of ~20 cycles with 400us period, 50% duty cycle to give receivers'AGC a chance to adjust. 
* A 2ms high
* A 2ms low
* data
* a 5ms pause before next repetition

In the data section, a 0 is 300us, 1 is 500us. Each bit consists of a high followed by a low, of equal length (to keep the duty cycle about 50% for AGC). 

