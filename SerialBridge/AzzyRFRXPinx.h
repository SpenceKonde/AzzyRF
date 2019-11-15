#ifdef __AVR_ATmega328P__
#define RX_PIN_STATE (PINB&1) //RX on pin 8 for input capture. 
#endif

#if defined(__AVR_ATtinyx16__) || defined(__AVR_ATtinyx06__)
#define RX_PIN_STATE (VPORTA.IN&2) //RX on pin A1 for input capture.  pin 14
#define RX_ASYNC0 0x0B
#endif

#if defined(__AVR_ATtinyx14__) || defined(__AVR_ATtinyx04__)
#define RX_PIN_STATE (VPORTA.IN&8) //RX on pin A3 for input capture.  pin 10
#define RX_ASYNC0 0x0D
#endif
