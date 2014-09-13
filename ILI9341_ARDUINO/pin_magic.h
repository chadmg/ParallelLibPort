#ifndef _pin_magic_
#define _pin_magic_

// This header file serves two purposes:CS_ACTIVE
//
// 1) Isolate non-portable MCU port- and pin-specific identifiers and
//    operations so the library code itself remains somewhat agnostic
//    (PORTs and pin numbers are always referenced through macros).
//
// 2) GCC doesn't always respect the "inline" keyword, so this is a
//    ham-fisted manner of forcing the issue to minimize function calls.
//    This sometimes makes the library a bit bigger than before, but fast++.
//    However, because they're macros, we need to be SUPER CAREFUL about
//    parameters -- for example, write8(x) may expand to multiple PORTsetWriteDirInline
//    writes that all refer to x, so it needs to be a constant or fixed
//    variable and not something like *ptr++ (which, after macro
//    expansion, may increment the pointer repeatedly and run off into
//    la-la land).  Macros also give us fine-grained control over which
//    operations are inlined on which boards (balancing speed against
//    available program space).

// When using the TFT shield, control and data pins exist in set physical
// locations, but the ports and bitmasks corresponding to each vary among
// boards.  A separate set of pin definitions is given for each supported
// board type.
// When using the TFT breakout board, control pins are configurable but
// the data pins are still fixed -- making every data pin configurable
// would be much too slow.  The data pin layouts are not the same between
// the shield and breakout configurations -- for the latter, pins were
// chosen to keep the tutorial wiring manageable more than making optimal
// use of ports and bitmasks.  So there's a second set of pin definitions
// given for each supported board.

// Shield pin usage:write8inline
// LCD Data Bit :    7    6    5    4    3    2    1    0
// Digital pin #:    7    6   13    4   11   10    9    8
// Uno port/pin :  PD7  PD6  PB5  PD4  PB3  PB2  PB1  PB0
// Mega port/pin:  PH4  PH3  PB7  PG5  PB5  PB4  PH6  PH5
// Leo port/pin :  PE6  PD7  PC7  PD4  PB7  PB6  PB5  PB4
// Due port/pin : PC23 PC24 PB27 PC26  PD7 PC29 PC21 PC22
// Breakout pin usage:
// LCD Data Bit :   7   6   5   4   3   2   1   0
// Uno dig. pin :   7   6   5   4   3   2   9   8
// Uno port/pin : PD7 PD6 PD5 PD4 PD3 PD2 PB1 PB0
// Mega dig. pin:  29  28  27  26  25  24  23  22
// Mega port/pin: PA7 PA6 PA5 PA4 PA3 PA2 PA1 PA0 (one contiguous PORT)
// Leo dig. pin :   7   6   5   4   3   2   9   8
// Leo port/pin : PE6 PD7 PC6 PD4 PD0 PD1 PB5 PB4
// Due dig. pin :  40  39  38  37  36  35  34  33
// Due port/pin : PC8 PC7 PC6 PC5 PC4 PC3 PC2 PC1 (one contiguous PORT. -ish…)

// Pixel read operations require a minimum 400 nS delay from RD_ACTIVE
// to polling the input pins.  At 16 MHz, one machine cycle is 62.5 nS.
// This code burns 7 cycles (437.5 nS) doing nothing; the RJMPs are
// equivalent to two NOPs each, final NOP burns the 7th cycle, and the
// last line is a radioactive mutant emoticon.
#define DELAY7        \
  asm volatile(       \
    "rjmp .+0" "\n\t" \
    "rjmp .+0" "\n\t" \
    "rjmp .+0" "\n\t" \
    "nop"      "\n"   \
    ::);

//#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega328__) || defined(__AVR_ATmega8__)

 // Arduino Uno, Duemilanove, etc.



  #define write8inline(d) {                          \
    PORTD = (PORTD & B00000011) | ((d) & B11111100); \
    PORTB = (PORTB & B11111100) | ((d) & B00000011); \
    WR_STROBE; }
  #define read8inline(result) {                       \
    RD_ACTIVE;                                        \
    DELAY7;                                           \
    result = (PIND & B11111100) | (PINB & B00000011); \
    RD_IDLE; }
  #define setWriteDirInline() { DDRD |=  B11111100; DDRB |=  B00000011; }
  #define setReadDirInline()  { DDRD &= ~B11111100; DDRB &= ~B00000011; }

 

  // As part of the inline control, macros reference other macros...if any
  // of these are left undefined, an equivalent function version (non-inline)
  // is declared later.  The Uno has a moderate amount of program space, so
  // only write8() is inlined -- that one provides the most performanceCS_ACTIVE
  // benefit, but unfortunately also generates the most bloat.  This is
  // why only certain cases are inlined for each board.
  #define write8 write8inline



//#elif defined(__SAM3X8E__)
//
//// Arduino Due
//
// #ifdef USE_ADAFRUIT_SHIELD_PINOUT
//
//  #define RD_PORT PIOA				/*pin A0 */	
//  #define WR_PORT PIOA				/*pin A1 */
//  #define CD_PORT PIOA				/*pin A2 */
//  #define CS_PORT PIOA				/*pin A3 */
//  #define RD_MASK 0x00010000
//  #define WR_MASK 0x01000000
//  #define CD_MASK 0x00800000
//  #define CS_MASK 0x00400000
//
//  #define write8inline(d) { \
//   PIO_Set(PIOD, (((d) & 0x08)<<(7-3))); \
//   PIO_Clear(PIOD, (((~d) & 0x08)<<(7-3))); \
//   PIO_Set(PIOC, (((d) & 0x01)<<(22-0)) | (((d) & 0x02)<<(21-1))| (((d) & 0x04)<<(29-2))| (((d) & 0x10)<<(26-4))| (((d) & 0x40)<<(24-6))| (((d) & 0x80)<<(23-7))); \
//   PIO_Clear(PIOC, (((~d) & 0x01)<<(22-0)) | (((~d) & 0x02)<<(21-1))| (((~d) & 0x04)<<(29-2))| (((~d) & 0x10)<<(26-4))| (((~d) & 0x40)<<(24-6))| (((~d) & 0x80)<<(23-7))); \
//   PIO_Set(PIOB, (((d) & 0x20)<<(27-5))); \
//   PIO_Clear(PIOB, (((~d) & 0x20)<<(27-5))); \
//   WR_STROBE; }
//
//  #define read8inline(result) { \    
//   RD_ACTIVE;   \
//   delayMicroseconds(1);      \
//   result = (((PIOC->PIO_PDSR & (1<<23)) >> (23-7)) | ((PIOC->PIO_PDSR & (1<<24)) >> (24-6)) | \
//             ((PIOB->PIO_PDSR & (1<<27)) >> (27-5)) | ((PIOC->PIO_PDSR & (1<<26)) >> (26-4)) | \
//             ((PIOD->PIO_PDSR & (1<< 7)) >> ( 7-3)) | ((PIOC->PIO_PDSR & (1<<29)) >> (29-2)) | \
//             ((PIOC->PIO_PDSR & (1<<21)) >> (21-1)) | ((PIOC->PIO_PDSR & (1<<22)) >> (22-0))); \
//   RD_IDLE;}
//
//  #define setWriteDirInline() { \
//   PIOD->PIO_MDDR |=  0x00000080; /*PIOD->PIO_SODR =  0x00000080;*/ PIOD->PIO_OER |=  0x00000080; PIOD->PIO_PER |=  0x00000080; \
//   PIOC->PIO_MDDR |=  0x25E00000; /*PIOC->PIO_SODR =  0x25E00000;*/ PIOC->PIO_OER |=  0x25E00000; PIOC->PIO_PER |=  0x25E00000; \
//   PIOB->PIO_MDDR |=  0x08000000; /*PIOB->PIO_SODR =  0x08000000;*/ PIOB->PIO_OER |=  0x08000000; PIOB->PIO_PER |=  0x08000000; }
//
//  #define setReadDirInline() { \
//	  pmc_enable_periph_clk( ID_PIOD ) ;	  pmc_enable_periph_clk( ID_PIOC ) ;	  pmc_enable_periph_clk( ID_PIOB ) ; \
//   PIOD->PIO_PUDR |=  0x00000080; PIOD->PIO_IFDR |=  0x00000080; PIOD->PIO_ODR |=  0x00000080; PIOD->PIO_PER |=  0x00000080; \
//   PIOC->PIO_PUDR |=  0x25E00000; PIOC->PIO_IFDR |=  0x25E00000; PIOC->PIO_ODR |=  0x25E00000; PIOC->PIO_PER |=  0x25E00000; \
//   PIOB->PIO_PUDR |=  0x08000000; PIOB->PIO_IFDR |=  0x08000000; PIOB->PIO_ODR |=  0x08000000; PIOB->PIO_PER |=  0x08000000; }
//
//   // Control signals are ACTIVE LOW (idle is HIGH)
//   // Command/Data: LOW = command, HIGH = data
//   // These are single-instruction operations and always inline
//   #define RD_ACTIVE  RD_PORT->PIO_CODR |= RD_MASK
//   #define RD_IDLE    RD_PORT->PIO_SODR |= RD_MASK
//   #define WR_ACTIVE  WR_PORT->PIO_CODR |= WR_MASK
//   #define WR_IDLE    WR_PORT->PIO_SODR |= WR_MASK
//   #define CD_COMMAND CD_PORT->PIO_CODR |= CD_MASK
//   #define CD_DATA    CD_PORT->PIO_SODR |= CD_MASK
//   #define CS_ACTIVE  CS_PORT->PIO_CODR |= CS_MASK
//   #define CS_IDLE    CS_PORT->PIO_SODR |= CS_MASK









 // Stuff common to all Arduino AVR board types:
 // When using the TFT breakout board, control pins are configurable.
 #define RD_ACTIVE  *rdPort &=  rdPinUnset
 #define RD_IDLE    *rdPort |=  rdPinSet
 #define WR_ACTIVE  *wrPort &=  wrPinUnset
 #define WR_IDLE    *wrPort |=  wrPinSet
 #define CD_COMMAND *cdPort &=  cdPinUnset
 #define CD_DATA    *cdPort |=  cdPinSet
 #define CS_ACTIVE  *csPort &=  csPinUnset
 #define CS_IDLE    *csPort |=  csPinSet




// Data write strobe, ~2 instructions and always inline
#define WR_STROBE { WR_ACTIVE; WR_IDLE; }

// These higher-level operations are usually functionalized,
// except on Mega where's there's gobs and gobs of program space.

// Set value of TFT register: 8-bit address, 8-bit value
#define writeRegister8inline(a, d) { \
  CD_COMMAND; write8(a); CD_DATA; write8(d); }

// Set value of TFT register: 16-bit address, 16-bit value
// See notes at top about macro expansion, hence hi & lo temp vars
#define writeRegister16inline(a, d) { \
  uint8_t hi, lo; \
  hi = (a) >> 8; lo = (a); CD_COMMAND; write8(hi); write8(lo); \
  hi = (d) >> 8; lo = (d); CD_DATA   ; write8(hi); write8(lo); }

// Set value of 2 TFT registers: Two 8-bit addresses (hi & lo), 16-bit value
#define writeRegisterPairInline(aH, aL, d) { \
  uint8_t hi = (d) >> 8, lo = (d); \
  CD_COMMAND; write8(aH); CD_DATA; write8(hi); \
  CD_COMMAND; write8(aL); CD_DATA; write8(lo); }

#endif // _pin_magic_
