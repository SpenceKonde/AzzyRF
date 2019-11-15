#ifdef __AVR_ATmega328P__
#define TX_PIN 7
#define txPIN PIND
#define txBV 128
#endif

#if defined(__AVR_ATtinyx16__) || defined(__AVR_ATtinyx06__)
#define TX_PIN 3
#define txPIN VPORTA.IN
#define txBV 128
#endif

#if defined(__AVR_ATtinyx14__) || defined(__AVR_ATtinyx04__)
#define TX_PIN 0
#define txPIN VPORTA.IN
#define txBV 16
#endif
