#ifdef __AVR_ATmega328P__

#define RX_PIN_STATE (PINB&1) //RX on pin 8 for input capture. 
#define SERIAL_CMD Serial
//#define SERIAL_DBG Serial

#endif

#if defined(__AVR_ATtinyx16__) || defined(__AVR_ATtinyx06__)
#define RX_PIN_STATE (VPORTA.IN&2) //RX on pin A1 for input capture.  pin 14
#define RX_ASYNC0 0x0B
#define SERIAL_CMD Serial
#endif

#if defined(__AVR_ATtinyx12__) || defined(__AVR_ATtinyx02__)
#define RX_PIN_STATE (VPORTA.IN&8) //RX on pin A3 for input capture.  pin 4
#define RX_ASYNC0 0x0D
#define SERIAL_CMD Serial
#endif
