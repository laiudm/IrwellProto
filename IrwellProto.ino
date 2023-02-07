///////////////////////////////////////////////////////////////////////////////////
//  G6LBQ Irwell HF Transceiver VFO - Version 1.0
//  stm32 + si5351a VFO With BFO & Conversion Oscilator
//   
//  Expanded with Multiple SI5351 & I/O Expanders by G6LBQ  
//
//  Created by G6LBQ on 15/09/2020 
//
//  RGB colours used for the display - Added by G6LBQ
//
//  255-0-0     Red
//  0-0-0       Black
//  255-255-0   Yellow
//  255-255-255 White
//  0-255-0     Green
//  50-50-50    Dark Grey
//  100-100-100 Grey
//  235-0-200   Pink
//  0-255-255   Turquoise
//  0-0-255     Blue
///////////////////////////////////////////////////////////////////////////////////

//#define REQUIRESERIALCONNECTED    // uncomment to stall on startup until usb serial is connected
#define MOCKI2C     // uncomment this to mock transmission to the 2x pcf8574 , Si5351s

#define OPTICAL_ENCODER         // uncomment this line for an optical encoder
#ifdef OPTICAL_ENCODER
#define ROTARY_LOW_SENSITIVITY 10   // encoder sensitivity for menu operations.
#define ROTARY_HIGH_SENSITIVITY 4   // encoder sensitivity for VFO.
#else
#define ROTARY_LOW_SENSITIVITY  1   // encoder sensitivity for menu operations.
#define ROTARY_HIGH_SENSITIVITY 1   // encoder sensitivity for VFO.
#endif

//---------- Library include ----------
#include <stdarg.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>

#include "si5351a2.h" 
#include "src/Ucglib.h"
#include "ButtonEvents.h"

//----------   Encoder setting  ---------------

#define ENC_A     PB12                    // Rotary encoder A
#define ENC_B     PB13                    // Rotary encoder B

//---------- Serial Debug Output -----------------------

// All output on the usb port comes through this routine.
// Be aware that writing while a host isn't connected could make this print
// run slowly, unless precautions are taken. See:
// http://docs.leaflabs.com/static.leaflabs.com/pub/leaflabs/maple-docs/latest/lang/api/serialusb.html#lang-serialusb and
// http://docs.leaflabs.com/static.leaflabs.com/pub/leaflabs/maple-docs/latest/lang/api/serialusb.html#lang-serialusb-safe-print 

void debugPrintf(const char * format, ...) {
  char buff[100];
  va_list va;
  va_start(va, format);
  vsnprintf(buff, sizeof buff, format, va);
  va_end(va);
  if (Serial.isConnected() && Serial.getDTR()) {
    Serial.println(buff);
  }
}


#define traceError(format, ...)
#define traceI2C(format, ...)
#define traceEEPROM(format, ...)
#define traceDisplay(format, ...)
#define traceLog(format, ...)

// uncomment the following lines to re-enable tracing
#define traceError(format, ...) debugPrintf(format, __VA_ARGS__)
#define traceI2C(format, ...) debugPrintf(format, __VA_ARGS__)
//#define traceEEPROM(format, ...) debugPrintf(format, __VA_ARGS__)
//#define traceDisplay(format, ...) debugPrintf(format, __VA_ARGS__)
#define traceLog(format, ...) debugPrintf(format, __VA_ARGS__)

//----------   TFT setting  ------------------- 

#define   __CS    PB10                        
#define   __DC    PB0                     
#define   __RST   PB1                        

#include "CacheDisplay.h"
Display ucg(__DC, __CS, __RST);

//----------  Button Setting -----------------

// define event names for the key events
typedef enum {EVT_NOCHANGE, EVT_PA0_BTNUP, EVT_PA0_LONGPRESS, EVT_PA1_BTNUP, EVT_PA1_LONGPRESS, EVT_PC14_BTNUP, EVT_PC14_LONGPRESS, EVT_PC15_BTNUP, EVT_PC15_LONGPRESS, 
EVT_PB11_BTNUP, EVT_PB11_LONGPRESS, EVT_PB3_BTNUP, EVT_PB3_LONGPRESS ,EVT_PB4_BTNUP, EVT_PB4_LONGPRESS , EVT_PB5_BTNUP, EVT_PB5_LONGPRESS } keyEvents;

ButtonEvents b = ButtonEvents(EVT_NOCHANGE);

//----------   I/O Assign  ------------------- 

#define   OUT_LSB      PB15               // Data line for controlling modes of operation                  
#define   OUT_USB      PA8                // Data line for controlling modes of operation                  
#define   OUT_CW       PA9                // G6LBQ added extra mode selection output                  
#define   OUT_AM       PA10               // G6LBQ added extra mode selection output
#define   OUT_FM       PB14               // G6LBQ changed from PA11 to PB14
#define   SW_BAND      PA1                // G6LBQ 29/11/2022 changed from PA0 to PA1
#define   SW_STEP      PC15               // G6LBQ 29/11/2022 changed from PA1 to PC15
#define   SW_TX        PB8                // G6LBQ 29/11/2022 changed from PB11 to PB8               
#define   SW_MODE      PA0                // G6LBQ 29/11/2022 changed from PC14 to PA0  
#define   SW_MENUON    PB4                // G6LBQ 29/11/2022 changed from PC15 to PB4
#define   SW_VFO       PC14               // G6LBQ 08/12/2022 changed from PA4  to PC14
#define   SW_ATTEN     PB11               // G6LBQ 29/11/2022 changed from PA2 to PB11
#define   SW_RFPRE     PB5                // G6LBQ 29/11/2022 added button to control RF Preamplifier
#define   SW_FILT      PB3                // G6LBQ 29/11/2022 added button to control DSP noise reduction

#define   OUT_ATT0     PA2
#define   OUT_ATT1     PA3
#define   OUT_ATT2     PA6

#define   OUT_RFPRE    PB9                // G6LBQ 29/11/2022 added control out for RF Preamplifier               

#define   LED          PC13


//---------  Si5351 Assignments ---------------------------
#define   VFO_PORT     0
#define   VFO_CHL      0
#define   BFO_PORT     1
#define   BFO_CHL      2
#define   CONV_PORT    2
#define   CONV_CHL     1


//----------   CW Tone  ------------------- 

#define   CW_TONE     700                 // 700Hz

//---------- Modes ---------------------

typedef enum {MODE_LSB, MODE_USB, MODE_CW, MODE_AM, MODE_FM} modes;

//---------- Variable setting ----------

#define _N(a) (sizeof(a)/sizeof(a[0]))

enum vfo_t { VFOA=0, VFOB=1, SPLIT=2 };

// state variables stored in eeprom 
// all these variables _MUST_ be 16bit or 32bit
// their initialisation values are stored in the eeprom on a "factory" reset
// As a convenience they are in the order they appear in the menu

int16_t  mode = MODE_USB;
int16_t  atten = 0;
int16_t  rfpre = 0;     //G6LBQ 29/11/2022 added for RF PreAmp
int16_t  filt = 0;
int16_t  bandval = 3;
int16_t  stepsize = 3;  
uint16_t vfosel = VFOA;
int16_t  rit = 0;
int16_t  ritFreq = 0;
uint32_t xtalfreq = 25000000;
uint32_t ifFreq[]  = {11056570,   11059840, 11057048, 11061850, 11061850};
int32_t  vfo[] = { 7074000, 14074000 };
uint32_t firstIF = 45000000;
uint16_t eeprom_version;

#define get_version_id() 1

// end of state variables stored in eeprom

uint8_t lastTXInput = 0;   // captures last state and current state of TX input line.
bool    transmitting = false;  // status indication - used to calcute VFO, BFO freqs

uint32_t EEPROMautoSave = 0;
#define EEPROMautoSaveTiming 5000  // in milliseconds. Too frequently would wear out the flash

static const uint32_t fstepRates[] = {1, 10, 100, 1000, 10000, 100000, 1000000};
#define fstepRatesSize (sizeof(fstepRates)/sizeof(fstepRates[0]))

static const uint32_t conversionOffsets[] { 
  56059000, // LSB
  33941000, // USB
  56059000, // CW
  56059000, // AM
  56059000  // FM
};

//------------- menu system --------------------------------

enum menu_t { NO_MENU, MENU_SELECTED, MENU_VALUE };
uint8_t menumode = 0;  // 0=not in menu, 1=selects menu item, 2=selects parameter value //todo - use enums
int8_t  menu = 1;  // current parameter id selected in menu


enum action_t { UPDATE, UPDATE_MENU, NEXT_MENU, LOAD, SAVE, SKIP, NEXT_CH };
enum params_t {NULL_, MODE, FILTER, BAND, STEP, VFOSEL, RIT, RITFREQ, ATTEN, RFPRE, SIFXTAL, IF_LSB, IF_USB, IF_CW, IF_AM, IF_FM, FIRSTIF, FREQA, FREQB, VERS, ALL=0xff};   // G6LBQ 29/11/2022 added RFPRE
#define N_PARAMS 16                // number of (visible) parameters    // G6LBQ 29/11/2022 increased Params from 15 to 16 so RFPRE is included
#define N_ALL_PARAMS (N_PARAMS+3)  // total of all parameters



//---------- Rotary Encoder Processing -----------------------

volatile int8_t encoder_val = 0;
 
void initRotary() {
  pinMode( ENC_A, INPUT_PULLUP);
  pinMode( ENC_B, INPUT_PULLUP);
  attachInterrupt( ENC_A, Rotary_enc, CHANGE);
  attachInterrupt( ENC_B, Rotary_enc, CHANGE);
}

// There were all sorts of problems getting the rotary encoder to work reliably.
// This algorithm seems to work more reliably:
// A "rotation event" is when both inputs have gone low;  rotation direction is determined by which input was the last to go low
// Once a rotation event has occurred, detect new rotation events only after both inputs have returned high again.

void Rotary_enc() {
  static bool rotationEvent = false;  // this could possibly be rolled into the last_state variable, but code readability?
  static uint8_t last_state = 0;

  uint8_t state = (digitalRead(ENC_B) << 1) | digitalRead(ENC_A);
  if (!rotationEvent && (state==0x0)) {
      rotationEvent = true;
      if (last_state & 1) encoder_val++; else encoder_val--;
  } else if (rotationEvent && (state==0x3)) {
    rotationEvent = false;
  }
  last_state = state;
}

int8_t lastReadVal;
int lastSensitivity = 1;

int8_t readEncoder(int sensitivity) {
  if (sensitivity != lastSensitivity) {
    encoder_val = 0;  // prevent frequency increment on exiting the menu system (high sensitivity only)
  }
  lastReadVal = encoder_val/sensitivity;
  lastSensitivity = sensitivity;
  return lastReadVal;
}

void resetEncoder() {
  encoder_val -= lastReadVal*lastSensitivity;
}

//--------- Band Pass Filter Bank Selection ---------------------------------

void write_PCF8574(uint8_t address, uint8_t data) {
#ifndef MOCKI2C
  Wire.begin();
  Wire.beginTransmission(address);
  Wire.write(data);
  Wire.endTransmission();
#endif

  traceI2C("write_PCF8574 to 0x%.2x: 0x%.2x", address, data);
}

void init_BPF() {
  write_PCF8574(0x38, 0b00000000);  //initialize PCF8574 I/O expander 1 for BPF filters 1 to 5 
  write_PCF8574(0x39, 0b00000000);  //initialize PCF8574 I/O expander 2 for BPF filters 6 to 10
}

void select_BPFbank(int8_t port, int8_t bank) {
  static int8_t currentPort = -1;
  static int8_t currentBank = -1;
  if ((port == currentPort) && (bank == currentBank))
    // the correct bank is already selected; nothing to do here
    return;
  init_BPF();   // turn off all banks
  write_PCF8574(port, bank);
  currentPort = port;
  currentBank = bank;
}

void select_BPF(uint32_t freq) {

  // HF Band Pass Filter logic added by G6LBQ
  if        (freq <  1600000){ select_BPFbank(0x38, 0b00000001);
  } else if (freq <  2000000){ select_BPFbank(0x38, 0b00000010);
  } else if (freq <  3000000){ select_BPFbank(0x38, 0b00000100);
  } else if (freq <  4000000){ select_BPFbank(0x38, 0b00001000);
  } else if (freq <  6000000){ select_BPFbank(0x38, 0b00010000);
  } else if (freq <  8000000){ select_BPFbank(0x39, 0b00000001);
  } else if (freq < 11000000){ select_BPFbank(0x39, 0b00000010);
  } else if (freq < 15000000){ select_BPFbank(0x39, 0b00000100);
  } else if (freq < 22000000){ select_BPFbank(0x39, 0b00001000);
  } else if (freq < 30000000){ select_BPFbank(0x39, 0b00010000);
  } 
}

//--------- Low Pass Filter Bank Selection ---------------------------------

#define LPF_PCF_ADDR 0x3a
#define MOCK_LPF

// Duplicate of write_PCF8574, but is added to allow this I2C interface to be 
// easily mocked out independently of the other I2C interfaces.
// This interface is a future addition to the hardware

void write_LPF_PCF8574(uint8_t address, uint8_t data) {
#ifndef MOCK_LPF
  Wire.begin();
  Wire.beginTransmission(address);
  Wire.write(data);
  Wire.endTransmission();
#endif

  traceI2C("write_LPF_PCF8574 to 0x%.2x: 0x%.2x", address, data);
}

void init_LPF() {
  write_LPF_PCF8574(LPF_PCF_ADDR, 0b00000000);  //initialize PCF8574 I/O expander for LPF filters 1 to 6
}

void select_LPFbank(int8_t bank) {
  static int8_t currentBank = -1;
  if (bank == currentBank)
    // the correct bank is already selected; nothing to do here
    return;
  write_LPF_PCF8574(LPF_PCF_ADDR, bank);
  currentBank = bank;
}

void select_LPF(uint32_t freq) {

  // Low Pass Filter selection logic. G6LBQ to add appropriate freq values
  if        (freq <  1600000){ select_LPFbank(0b00000001);
  } else if (freq <  2000000){ select_LPFbank(0b00000010);
  } else if (freq <  3000000){ select_LPFbank(0b00000100);
  } else if (freq < 10000000){ select_LPFbank(0b00001000);
  } else if (freq < 20000000){ select_LPFbank(0b00010000);
  } else if (freq < 30000000){ select_LPFbank(0b00100000);
  }
}

//--------- DSP Filter Bank Selection ---------------------------------
//--------- Added by G6LBQ 10/12/2022 ---------------------------------
#define DSP_PCF_ADDR 0x3b
//#define MOCK_DSP

void write_DSP_PCF8574(uint8_t address, uint8_t data) {
#ifndef MOCK_DSP
  Wire.begin();
  Wire.beginTransmission(address);
  Wire.write(data);
  Wire.endTransmission();
#endif

  traceI2C("write_DSP_PCF8574 to 0x%.2x: 0x%.2x", address, data);
}

void init_DSP() {
  write_DSP_PCF8574(DSP_PCF_ADDR, 0b11111111);  //initialize PCF8574 I/O expander for DSP filters
}

void select_DSPbank(int8_t bank) {
  static int8_t currentBank = -1;
  if (bank == currentBank)
    // the correct bank is already selected; nothing to do here
    return;
  write_DSP_PCF8574(DSP_PCF_ADDR, bank);
  currentBank = bank;
}

void select_DSP(uint32_t filt) {

  // DSP Filter selection logic.
  // BHI Module N2=Bit3 N1=Bit2 N0=Bit1
  // BHI Module DSP enable/disable = Bit4
  if        (filt ==0){ select_DSPbank(0b11110111);   // DSP filter is off
  } else if (filt ==1){ select_DSPbank(0b11111000);   // Level 1
  } else if (filt ==2){ select_DSPbank(0b11111001);   // Level 2
  } else if (filt ==3){ select_DSPbank(0b11111010);   // Level 3
  } else if (filt ==4){ select_DSPbank(0b11111011);   // Level 4
  } else if (filt ==5){ select_DSPbank(0b11111100);   // Level 5
  } else if (filt ==6){ select_DSPbank(0b11111101);   // Level 6
  } else if (filt ==7){ select_DSPbank(0b11111110);   // Level 7
  } else if (filt ==8){ select_DSPbank(0b11111111);   // Level 8 - DSP filter is on full
  }
}

void select_Attenuator(uint16_t atten) {
  digitalWrite(OUT_ATT0, atten==1);
  digitalWrite(OUT_ATT1, atten==2);
  digitalWrite(OUT_ATT2, atten==3);
}

void select_RFPreamplifier(uint16_t rfpre) {    // G6LBQ 29/11/2022 added for RF PreAmplifier
  digitalWrite(OUT_RFPRE, rfpre==1);
}

// -----     Routine to interface to the TCA9548A -----

#define TCAADDR 0x70

void tcaselect (uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}


// -----     Routines to interface to the Si5351s-----

uint32_t currentFrequency[] = { -1, -1, -1 };   // track output frequences for changes
uint32_t currentXtalFreq = -1;

void _setFrequency(uint8_t port, uint8_t channel, uint32_t frequency, uint32_t xtalFreq) {
#ifndef MOCKI2C
  tcaselect(port);
  si5351aSetFrequency(channel, frequency, xtalfreq);
#endif
  traceI2C("SI5351 port: %i, chl: %i, freq: %i, xtalFreq: %i", port, channel, frequency, xtalfreq);
}

void setFrequency(uint8_t port, uint8_t channel, uint32_t frequency, uint32_t xtalFreq) {
  if (xtalFreq != currentXtalFreq) {
    // invalidate the "cache"
    currentFrequency[0] = - 1; currentFrequency[1] = - 1; currentFrequency[2] = - 1;;
    currentXtalFreq = xtalFreq;
  }
  // don't need to track port and channel; port is unique
  if (currentFrequency[port] == frequency)
    // it's already set to the correct frequency; nothing further to do here
    return;
  _setFrequency(port, channel, frequency, xtalFreq);
  currentFrequency[port] = frequency;
}

void disableFrequency(uint8_t port, uint8_t channel) {
  currentFrequency[port] = -1;  // erase the "cached" frequency
#ifndef MOCKI2C
  tcaselect(port);
  si5351aOutputOff(channel);
#endif
  traceI2C("Si5351 disable port: %i, chl: %i", port, channel);
}


//----------  EEPROM Routines  ---------

int eeprom_addr;
#define EEPROM_OFFSET 0x0

void eeprom_init() {
  uint16_t status = EEPROM.init();  // I think default parameters are fine
  if (status != EEPROM_OK)
    traceError("Eeprom init error: %i", status);
}

// The menu system reads & writes in units of 16 bits

void eeprom_read_block (void *__dst, const void *__src, size_t __n) {
  for (int i=0; i<__n; i++) {
    uint16_t status = EEPROM.read( ((int)__src)+i,  (uint16_t *)__dst+i );
    if (status != EEPROM_OK) {
      traceError("Eeprom read error: %i. Read from %i", status, *(int *)__src); 
    }
  }
  traceEEPROM("eeprom_read_block: from addr %i, length %i, value: %i", (int) __src, (int) __src, *(uint16_t *)__dst);
  };

void eeprom_write_block(const void *__src, void *__dst, size_t __n) {
  traceEEPROM("eeprom_write_block: to addr %i, length: %i, value: %i", (int) __dst, __n, *(uint16_t *)__src);
  for (int i=0; i<__n; i++) {
    uint16_t status = EEPROM.write( ((int)__dst)+i, *((uint16_t *)__src+i) ); // only update "eeprom" if the value has changed.
    if (status != EEPROM_OK) {
      traceError("Eeprom write error: %i. Write to %i", status, (int) __dst);
    }
  }
};

// -----------menu system labels ----------------------------------

const char* mode_label[]       = { "LSB", "USB", "CW ", "AM ", "FM " };
const char* filt_label[]       = { "OFF ", " 1  ", " 2  ", " 3  ", " 4  ", " 5  ", " 6  ", " 7  ", "FULL" };

// Band information
const char* band_label[]       = {  "160m",    "80m",    "60m",    "40m",    "30m",    "20m",     "17m",     "15m",    "12m",     "10m",   "MW" };
const uint8_t bandMode[]       = { MODE_LSB, MODE_LSB,  MODE_LSB, MODE_LSB,  MODE_CW, MODE_USB,  MODE_USB,  MODE_USB, MODE_USB, MODE_USB, MODE_AM };
const uint32_t bandFreq[]      = {  1900000,  3700000,   5460000,  7100000, 10100000, 14100000,  18100000,  21250000, 24925000, 28250000, 1458000 };

const char* stepsize_label[]   = { "1Hz   ", "10Hz  ", "100Hz ", "1KHz  ", "10KHz ", "100KHz", "1MHz  " };
const char* vfosel_label[]     = { "VFO A", "VFO B"/*, "Split"*/ };
const char* offon_label[]      = {"OFF", "ON"};
const char* atten_label[]      = {"OFF ", "6dB ", "12dB", "18dB"};
const char* rfpre_label[]      = {"OFF ", "ON "};


//------------- display subsystem  --------------------------------

// Ideally only use these fonts which I've tested. Font must be monospace
#define fontTiny    ucg_font_9x15_mr
#define fontSmaller ucg_font_10x20_mr
#define fontSmall   ucg_font_profont17_mr
#define fontSmall4  ucg_font_courB14_mr
#define fontSmall5  ucg_font_courR10_mr
#define fontMedium  ucg_font_inb16_mr
#define fontLarge   ucg_font_inb24_mr
#define fontLarger  ucg_font_inr33_mr
#define fontHuge    ucg_font_inb33_mn    

void paintBackground() {
  // paint any borders, colour fills, unchanging text to make it attractive

  // paint a border around the main frequency display
  ucg.setColor(255, 165, 0);
  ucg.drawRBox(1,1,314,55,5);
  ucg.setColor(255,255,255);
  ucg.drawRFrame(1,1,314,55,5);
  ucg.drawRFrame(2,2,312,53,5);
  ucg.setColor(47, 47, 47);
  ucg.drawRFrame(3,3,310,51,5);
  
  // paint a border around the secondary frequency display
  ucg.setColor(178, 34, 34);
  ucg.drawRBox(1, 60, 314, 40, 5);
  ucg.setColor(255, 255, 255);
  ucg.drawRFrame(1, 60, 314, 40, 5);
  ucg.drawRFrame(2, 61, 312, 38, 5);

  // paint a border around the Mode
  ucg.setColor(250, 0, 0);
  ucg.drawRBox(1, 119, 93, 26, 5);
  ucg.setColor(255,255,255);
  ucg.drawRFrame(1, 119, 93, 26, 5);

  // paint a border around the Attn
  ucg.setColor(0, 0, 205);
  ucg.drawRBox(1, 150, 93, 26, 5);
  ucg.setColor(255, 255, 255);
  ucg.drawRFrame(1, 150, 93, 26, 5);

  // paint a border around the Filter
  ucg.setColor(72, 61, 139);
  ucg.drawRBox(99, 150, 93, 26, 5);
  ucg.setColor(255,255,255);
  ucg.drawRFrame(99, 150, 93, 26, 5);
  
  // paint a border around the RF Pre-Amplifier
  ucg.setColor(0, 139, 139);
  ucg.drawRBox(99, 119, 93, 26, 5);
  ucg.setColor(255,255,255);
  ucg.drawRFrame(99, 119, 93, 26, 5);

  // paint a border around the Step size
  ucg.setColor(254, 140, 0);
  ucg.drawRBox(198, 119, 116, 26, 5);
  ucg.setColor(255,255,255);
  ucg.drawRFrame(198, 119, 116, 26, 5);

  // paint a border around the RIT offset
  ucg.setColor(255, 0, 0);
  ucg.drawRBox(198, 150, 116, 26, 5);
  ucg.setColor(255,255,255);
  ucg.drawRFrame(198, 150, 116, 26, 5);

  // paint id
  ucg.setFont(fontTiny);
  ucg.setPrintPos(10,115);
  ucg.setColor(235,0,200);
  ucg.print( "---G6LBQ Irwell HF Transceiver---");
}

void printFreq(uint32_t freq) {
  char b1[20];
  char b2[20];
  sprintf(b1,"%8i",freq);
  sprintf(b2,"%.2s.%.3s.%.3s", &b1[0], &b1[2], &b1[5]);
  ucg.print(b2);
}

void updateScreen() {
  char buff[20];
 // printPrimaryVFO
  ucg.setPrintPos( 12, 44); ucg.setColor(1, 255, 165, 0); ucg.setFont(fontHuge);  ucg.setColor(79, 79, 79); printFreq(vfo[vfosel]);
  ucg.setPrintPos(278, 40); ucg.setColor(1, 255, 165, 0); ucg.setFont(fontLarge); ucg.setColor(255, 255, 0);   ucg.print(vfosel ? "B" : "A");
    
  // printSecondaryVFO
  ucg.setPrintPos(  26,  91); ucg.setColor(1, 178, 34, 34); ucg.setFont(ucg_font_inb24_mr); ucg.setColor(255, 0, 0); printFreq(vfo[vfosel^1]);
  ucg.setPrintPos( 278,  91); ucg.setColor(1, 178, 34, 34); ucg.setFont(ucg_font_inb24_mr); ucg.setColor(255, 145, 0); ucg.print(vfosel^1 ? "B" : "A");
    
  // printMode
  ucg.setPrintPos( 7, 138); ucg.setFont(fontSmaller); ucg.setColor(0, 255, 255, 255); ucg.setColor(1, 250, 0, 0); ucg.print("Mode"); 
  ucg.setPrintPos( 53, 138); ucg.setColor(1, 250, 0, 0);  ucg.setColor(255, 255, 0); ucg.print(mode_label[mode]);

  // print attentuator setting
  ucg.setPrintPos( 7, 169); ucg.setFont(fontSmaller); ucg.setColor(0, 255, 255, 255); ucg.setColor(1, 0, 0, 205); ucg.print("Attn");
  ucg.setPrintPos( 53, 169); ucg.setColor(1, 0, 0, 205); ucg.setColor(255, 255, 0); ucg.print(atten_label[atten]);

  // print DSP filter setting
  ucg.setPrintPos( 105, 169); ucg.setFont(fontSmaller); ucg.setColor(0, 255, 255, 255); ucg.setColor(1, 72, 61, 139); ucg.print("DspF");
  ucg.setPrintPos( 151, 169); ucg.setColor(1, 72, 61, 139); ucg.setColor(255, 255, 0); ucg.print(filt_label[filt]);

  // print RF Premaplifier setting
  ucg.setPrintPos( 105, 138); ucg.setFont(fontSmaller); ucg.setColor(0, 255, 255, 255); ucg.setColor(1, 0, 139, 139); ucg.print("PreA");
  ucg.setPrintPos( 151, 138); ucg.setColor(1, 0, 139, 139); ucg.setColor(255, 255, 0); ucg.print(rfpre_label[rfpre]);
    
  // printStep
  ucg.setPrintPos( 204, 138); ucg.setFont(fontSmaller); ucg.setColor(0, 255, 255, 255); ucg.setColor(1, 254, 140, 0); ucg.print("Step");
  ucg.setPrintPos( 249, 138); ucg.setColor(1,254, 140, 0);  ucg.setColor(255, 255, 0); ucg.print(stepsize_label[stepsize]);
    
  // printRIT
  ucg.setPrintPos( 204, 169); ucg.setFont(fontSmaller); ucg.setColor(0, 255, 255, 255); ucg.setColor(1, 255, 0, 0); ucg.print("RIT"); ucg.print(rit?"+":"-");
  //ucg.setPrintPos( 144, 176); ucg.print(offon_label[rit]); ucg.print("  ");
  sprintf(buff,"%-+5i", ritFreq);
  ucg.setPrintPos( 249, 169); ucg.setColor(1, 255, 0, 0);  ucg.setColor(255, 255, 0); ucg.print(buff);
}

void printBlanks(){
  ucg.print("          ");
}

void printTXstate(bool tx) {  // TODO - may need to rework this
  ucg.setPrintPos( 115, 205);
  if (tx) {
    ucg.setColor(255,0,0);
    ucg.drawRFrame(80, 187, 150, 23, 0);
    ucg.drawRFrame(79, 186, 152, 25, 0);
    ucg.drawRFrame(78, 185, 154, 27, 0);
    ucg.setFont(fontSmaller); ucg.setColor(255,0,0); ucg.print("---TX---"); //ucg.setColor(255,255,255);
  } else {
    ucg.setColor(0,0,0);
    ucg.drawRFrame(80, 187, 150, 23, 0);
    ucg.drawRFrame(79, 186, 152, 25, 0);
    ucg.drawRFrame(78, 185, 154, 27, 0);
    ucg.print("          ");
  }
}

//--------------- Business Logic---------------------------------------------

void updateModeOutputs(uint8_t mode) {
  digitalWrite(OUT_LSB, mode==MODE_LSB);
  digitalWrite(OUT_USB, mode==MODE_USB);
  digitalWrite(OUT_CW,  mode==MODE_CW);       // G6LBQ added 1/11/20
  digitalWrite(OUT_AM,  mode==MODE_AM);       // G6LBQ added 1/11/20
  digitalWrite(OUT_FM,  mode==MODE_FM);       // G6LBQ added 15/08/22
}

// change this to void updateAllFreq() when going to dual-conversion
//void updateAllFreqDualConversion() {
void updateAllFreq() {
  int32_t freq = vfo[vfosel];
  int32_t finalIF = ifFreq[mode];
  int32_t freqRIT = rit ? ritFreq : 0;
  
  select_BPF(freq);
  select_LPF(freq);
  select_DSP(filt);
  updateModeOutputs(mode);
  
  // set the VFO to put the desired input frequency at 45MHz
  // set the Conv Oscillator to bring 45MHz to the firstIF frequency
  // set each so that the net result is for frequency inversion to occur (to keep the 
  // rest of the radio unchanged

//#define HIGHSIDEVFO                             //Uncomment for single conversion, comment out for dual conversion
#ifdef HIGHSIDEVFO
  uint32_t vfofreq = firstIF + (freq + freqRIT);  // causes frequency inversion
  uint32_t convFreq = firstIF - finalIF;          // no frequency inversion
#else  
  uint32_t vfofreq = firstIF - (freq + freqRIT);  // low side VFO, no frequency inversion
  uint32_t convFreq = firstIF + finalIF;          // causes frequency inversion
#endif  
  
  setFrequency(VFO_PORT, VFO_CHL, vfofreq, xtalfreq);
  setFrequency(CONV_PORT, CONV_CHL, convFreq, xtalfreq); // no frequency inversion
  switch(mode) {
    case MODE_USB: 
    case MODE_LSB:
      setFrequency(BFO_PORT, BFO_CHL, finalIF, xtalfreq);
      break;
    case MODE_CW:
      setFrequency(BFO_PORT, BFO_CHL, finalIF + CW_TONE, xtalfreq);
      break;
    case MODE_AM:
      disableFrequency(BFO_PORT, BFO_CHL);
      break;
    case MODE_FM:
      disableFrequency(BFO_PORT, BFO_CHL);
      break;
  }
}

// change this to updateAllFreqHide() when going to dual-conversion
//void updateAllFreq() {                          //Uncomment for single conversion, comment out for dual conversion  
void updateAllFreqHide() {                      //Uncomment for dual conversion, comment out for single conversion  
  int32_t freq = vfo[vfosel];
  int32_t finalIF = ifFreq[mode];
  int32_t freqRIT = rit ? ritFreq : 0;
  
  select_BPF(freq);
  select_LPF(freq);
  updateModeOutputs(mode);
  uint32_t vfofreq = freq + finalIF + freqRIT;
  setFrequency(VFO_PORT, VFO_CHL, vfofreq, xtalfreq);
  switch(mode) {
    case MODE_USB: 
    case MODE_LSB:
      setFrequency(BFO_PORT, BFO_CHL, finalIF, xtalfreq);
      break;
    case MODE_CW:
      setFrequency(BFO_PORT, BFO_CHL, finalIF + CW_TONE, xtalfreq);
      break;
    case MODE_AM:
      disableFrequency(BFO_PORT, BFO_CHL);
      break;
    case MODE_FM:
      disableFrequency(BFO_PORT, BFO_CHL);
      break;
  }
}

//--------------- value change triggers --------------------------------------

void setEEPROMautoSave() {
  EEPROMautoSave = millis() + EEPROMautoSaveTiming;
}

void triggerVFOChange() {
  updateAllFreq();
  updateScreen();
}

void triggerBandChange(int menu) {
  mode = bandMode[bandval];
  vfo[vfosel] = bandFreq[bandval];
  triggerValueChange(0);
  setEEPROMautoSave();
}

// Anything could have changed so calculate _everything_
void triggerValueChange(int menu) {
  select_Attenuator(atten);
  select_RFPreamplifier(rfpre);     // G6LBQ 29/11/2022 added for RF PreAmplifier
  updateAllFreq();
  updateScreen();
}

void triggerNoop(int menu) {}


//-------------- encoder tune rate  -----------

void setstepUp(){
  stepsize = (stepsize + 1) % fstepRatesSize;
  updateScreen();
  setEEPROMautoSave();
}

void setstepDown() {
  if (stepsize==0) stepsize = fstepRatesSize;
  stepsize--;
  updateScreen();
  setEEPROMautoSave();
}

void bandUp() {
  bandval = (bandval + 1) % _N(band_label);
  triggerBandChange(0);
  setEEPROMautoSave();
}

void bandDown() {
  if (bandval==0) bandval = _N(band_label);
  bandval--;
  triggerBandChange(0);
  setEEPROMautoSave();
}

void setmodeUp() {
  mode = (mode + 1) % _N(mode_label);
  triggerValueChange(0);
  setEEPROMautoSave();
}

void setmodeDown() {
  if (mode==0) mode = _N(mode_label);
  mode--;
  triggerValueChange(0);
  setEEPROMautoSave();
}

// menu system code


void clearMenuArea(){
  ucg.setFont(fontSmaller); ucg.setColor(0, 255, 255, 255);
  ucg.setPrintPos( 3, 232);
  printBlanks(); printBlanks();
  ucg.setPrintPos( 155, 232);
  printBlanks(); printBlanks();
}

void printmenuid(uint8_t menuid){ // output menuid in x.y format
  static const char separator[] = {'.', ' '};
  uint8_t ids[] = {(uint8_t)(menuid >> 4), (uint8_t)(menuid & 0xF)};
  for(int i = 0; i < 2; i++){
    uint8_t id = ids[i];
    if(id >= 10){
      id -= 10;
      ucg.print('1');
    }
    ucg.print(char('0' + id));
    ucg.print(separator[i]);
  }
}

void printlabel(uint8_t action, uint8_t menuid, const char* label){
  ucg.setFont(fontSmaller); ucg.setColor(0, 255, 255, 0); ucg.setColor(1, 0, 0, 0);
  if(action == UPDATE_MENU){
    ucg.setPrintPos( 3, 232);
    printmenuid(menuid);
    ucg.print(label); printBlanks(); printBlanks();
    ucg.setFont(fontSmaller); ucg.setColor(0, 255, 0, 0);
    ucg.setPrintPos( 155, 232);                         
    if(menumode >= MENU_VALUE) ucg.print('>');
  } else { // UPDATE (not in menu)
    ucg.setPrintPos( 3, 232); ucg.print(label); ucg.print(": ");
  }
}

void actionCommon(uint8_t action, uint16_t *ptr, uint8_t size){
  switch(action){
    case LOAD:
      eeprom_read_block((void *)ptr, (const void *)eeprom_addr, size);
      break;
    case SAVE:
      eeprom_write_block((const void *)ptr, (void *)eeprom_addr, size);
      break;
    case SKIP:  // for calculating the eeprom_addr for a menu item later in the list
      break;
  }
  eeprom_addr += size;
}

template<typename T> void paramAction(uint8_t action, volatile T& value, uint8_t menuid, const char* label, const char* enumArray[], int32_t _min, int32_t _max, void (*trigger)(int m)){
  
  switch(action){
    case UPDATE:
    case UPDATE_MENU: {
      int32_t delta = readEncoder(ROTARY_LOW_SENSITIVITY);
      if (sizeof(T) == sizeof(uint32_t)) delta *= fstepRates[stepsize];  // large menu items use the tune-rate
      value = (int32_t)value + delta;
      if(     value < _min) value = _min;
      else if(value > _max) value = _max;
      resetEncoder();

      printlabel(action, menuid, label);  // print normal/menu label
      if(enumArray == NULL){  // print value
        if((_min < 0) && (value >= 0)) ucg.print('+');  // add + sign for positive values, in case negative values are supported
        ucg.print(value);
      } else {
        ucg.print(enumArray[value]);
      }
      printBlanks(); printBlanks();
      if (delta)
        trigger(menuid);  // only trigger if there's a change to the value
      break;
    }
      
    default:
      actionCommon(action, (uint16_t *)&value, sizeof(value)/2); // /2 because the eeprom lib stores in units of 16 bits
      break;
  }
}

int8_t paramAction(uint8_t action, uint8_t id = ALL) { // list of parameters
  if((action == SAVE) || (action == LOAD)){
    // first calculate eeprom_addr for this item
    eeprom_addr = EEPROM_OFFSET;
    for(uint8_t _id = 1; _id < id; _id++) 
      paramAction(SKIP, _id);
  }
 
  switch(id){    
    case ALL:     for(id = 1; id != N_ALL_PARAMS+1; id++) paramAction(action, id);  // for all parameters
    // Visible parameters
    case MODE:    paramAction(action, mode,           0x11,    "Mode -----",      mode_label,        0, _N(mode_label)-1, triggerValueChange); break;
    case FILTER:  paramAction(action, filt,           0x12,    "NR Filter -",     filt_label,       0, _N(filt_label)-1, triggerValueChange); break;
    case BAND:    paramAction(action, bandval,        0x13,    "Band -----",      band_label,        0, _N(band_label)-1, triggerBandChange ); break;
    case STEP:    paramAction(action, stepsize,       0x14,    "Tune Rate ",      stepsize_label,    0, _N(stepsize_label)-1, triggerValueChange); break;
    case VFOSEL:  paramAction(action, vfosel,         0x15,    "VFO Mode -",      vfosel_label,      0, _N(vfosel_label)-1, triggerValueChange); break;
    case RIT:     paramAction(action, rit,            0x16,    "RIT ------",      offon_label,       0, _N(offon_label)-1, triggerValueChange); break;
    case RITFREQ: paramAction(action, ritFreq,        0x17,    "RIT Offset",      NULL,    -1000,    1000, triggerValueChange); break;
    case ATTEN:   paramAction(action, atten,          0x18,    "Atten ----",      atten_label,       0, _N(atten_label)-1, triggerValueChange); break;
    case RFPRE:   paramAction(action, rfpre,          0x19,    "RF Preamp ",      rfpre_label,       0, _N(rfpre_label)-1, triggerValueChange); break;    // G6LBQ 29/11/2022 added for RF PreAmplifier
    case SIFXTAL: paramAction(action, xtalfreq,       0x81,    "RefFreq --",      NULL, 24975000,    25025000, triggerValueChange); break;
    case IF_LSB:  paramAction(action, ifFreq[0],      0x82,    "IF-LSB ---",      NULL, 8000000,     12000000, triggerValueChange); break;
    case IF_USB:  paramAction(action, ifFreq[1],      0x83,    "IF-USB ---",      NULL, 8000000,     12000000, triggerValueChange); break;
    case IF_CW:   paramAction(action, ifFreq[2],      0x84,    "IF-CW ----",      NULL, 8000000,     12000000, triggerValueChange); break;
    case IF_AM:   paramAction(action, ifFreq[3],      0x85,    "IF-AM ----",      NULL, 8000000,     12000000, triggerValueChange); break;
    case IF_FM:   paramAction(action, ifFreq[4],      0x86,    "IF-FM ----",      NULL, 8000000,     12000000, triggerValueChange); break;
    case FIRSTIF: paramAction(action, firstIF,        0x87,    "First-IF -",      NULL, 42000000,    48000000, triggerValueChange); break;
    
    // invisible parameters. These are here only for eeprom save/restore
    case FREQA:   paramAction(action, vfo[VFOA],         0,        NULL,           NULL,         0,                    0, triggerNoop       ); break;
    case FREQB:   paramAction(action, vfo[VFOB],         0,        NULL,           NULL,         0,                    0, triggerNoop       ); break;
    case VERS:    paramAction(action, eeprom_version,    0,        NULL,           NULL,         0,                    0, triggerNoop       ); break;
    
    // case NULL_:   menumode = NO_MENU; clearMenuArea(); break;
    default:      if((action == NEXT_MENU) && (id != N_PARAMS)) 
                      id = paramAction(action, max(1 /*0*/, min(N_PARAMS, id + ((readEncoder(ROTARY_LOW_SENSITIVITY) > 0) ? 1 : -1))) ); break;  // keep iterating util menu item found
  }
  return id;  // needed?
}

void processMenuKey() {
    if     (menumode == 0){ menumode = 1; paramAction(UPDATE_MENU, menu);}  // enter menu mode
    else if(menumode == 1){ menumode = 2; paramAction(UPDATE_MENU, menu);}            // enter value selection screen
    else if(menumode >= 2){ paramAction(SAVE, menu); menumode = 0; clearMenuArea();}  // save value, and return to default screen
}

void processEnterKey() {
  if     (menumode == 1){ menumode = 0; clearMenuArea();}  
  else if(menumode >= 2){ menumode = 1; paramAction(UPDATE_MENU, menu); paramAction(SAVE, menu); } // save value, and return to menu mode

}

void processMenu() {
  if (menumode) {
    int8_t encoder_change = readEncoder(ROTARY_LOW_SENSITIVITY);
    if((menumode == 1) && encoder_change){
       menu += readEncoder(ROTARY_LOW_SENSITIVITY);   // Navigate through menu
      menu = max(1 , min(menu, N_PARAMS));
      menu = paramAction(NEXT_MENU, menu);  // auto probe next menu item (gaps may exist)
      resetEncoder();
    }
    if(encoder_change)
      paramAction(UPDATE_MENU, menu);  // update param with encoder change and display.
  }
}


//------------------  Initialization -------------------------
 
void setup() {
#ifdef REQUIRESERIALCONNECTED
  while (!Serial)
    ;
  traceLog("Starting setup\n", 0);
#endif    

  initRotary();
  
  ucg.begin(UCG_FONT_MODE_SOLID);
  ucg.clearScreen();
  ucg.setRotate270();
  ucg.setColor(1, 0, 0, 0);       // set foreground color
 
  b.add(SW_BAND, EVT_PA1_BTNUP, EVT_PA1_LONGPRESS);      // G6LBQ 29/11/2022 Changed from PA0 to PA1
  b.add(SW_STEP, EVT_PC15_BTNUP, EVT_PC15_LONGPRESS);    // G6LBQ 29/11/2022 Changed from PA1 to PC15
  b.add(SW_VFO,  EVT_PC14_BTNUP, EVT_PC14_LONGPRESS);    // G6LBQ 08/12/2022 Changed from PA4 to PC14  
  b.add(SW_MODE, EVT_PA0_BTNUP, EVT_PA0_LONGPRESS);      // G6LBQ 29/11/2022 Changed from PC14 to PA0  
  b.add(SW_MENUON, EVT_PB4_BTNUP, EVT_PB4_LONGPRESS);    // G6LBQ 29/11/2022 Changed from PC15 to PB4
  b.add(SW_ATTEN,EVT_PB11_BTNUP, EVT_PB11_LONGPRESS);    // G6LBQ 29/11/2022 Changed from PA6 to PB11
  b.add(SW_RFPRE,EVT_PB5_BTNUP, EVT_PB5_LONGPRESS);      // G6LBQ 29/11/2022 Added for RF PreAmplifier
  b.add(SW_FILT,EVT_PB3_BTNUP, EVT_PB3_LONGPRESS);       // G6LBQ 29/11/2022 Added for DSP Noise Reduction
   
  
  pinMode(LED, OUTPUT);
  pinMode(SW_TX,INPUT_PULLUP);
  pinMode(OUT_LSB,OUTPUT);                    // LSB Mode 
  pinMode(OUT_USB,OUTPUT);                    // USB Mode
  pinMode(OUT_CW,OUTPUT);                     // CW Mode - G6LBQ added additional mode selection
  pinMode(OUT_AM,OUTPUT);                     // AM Mode - G6LBQ added additional mode selection
  pinMode(OUT_FM,OUTPUT);                     // G6LBQ added 15/08/22
  pinMode(OUT_ATT0,OUTPUT);
  pinMode(OUT_ATT1,OUTPUT);
  pinMode(OUT_ATT2,OUTPUT);
  pinMode(OUT_RFPRE,OUTPUT);                  // G6LBQ 29/11/2022 added output to control RF PreAmp

  init_BPF();
  init_LPF();

  eeprom_init();
  // Load parameters from EEPROM, reset to factory defaults when 
  // 1. stored values are from a different version
  // 2. SW_STEP pressed during power-up

  traceEEPROM("Input SW_STEP value before delay is: %i", SW_STEP); 
  delay(200); // ensure input buttons are valid by giving any external Capacitances plenty of time to charge
  traceEEPROM("Input SW_STEP value after delay is: %i",digitalRead(SW_STEP)); 
  traceEEPROM("checking version", 0);
  paramAction(LOAD, VERS);
  traceEEPROM("On initial load eeprom_version: %i", eeprom_version);
  if((eeprom_version != get_version_id()) || !digitalRead(SW_STEP)){
  //if( (eeprom_version != get_version_id()) ){
    if (!digitalRead(SW_STEP)) {
      traceEEPROM("forced eeprom reload", 0);
    }
    traceEEPROM("Reload - eeprom_version: %i", eeprom_version);
    eeprom_version = get_version_id();
    paramAction(SAVE);  // save default parameter values
    ucg.setFont(fontTiny); ucg.setPrintPos( 0, 44); ucg.setColor(235,0,200); ucg.print("Reset settings..");
    delay(1000);
    ucg.clearScreen();
  }
  paramAction(LOAD);  // load all parameters
  traceEEPROM("After Load-all eeprom_version: %i", eeprom_version);

  paintBackground();
  triggerValueChange(0);
}

//----------  Main program  ------------------------------------

void loop() {

  digitalWrite(LED, !digitalRead(LED));

  int event = b.getButtonEvent();
  switch (event) {
     case EVT_PB3_BTNUP:
       traceLog("PB3_BTNUP\n", 0);
       break;
       
     case EVT_PB4_BTNUP: // was the RIT button
      processMenuKey();
      break;      
 
    case EVT_PC15_BTNUP:     // STEP button
      setstepDown();
      break;
    case EVT_PC15_LONGPRESS: // STEP button
      setstepUp();
      break;
      
    case EVT_PA1_BTNUP:     // BAND button
      bandUp();             // G6LBQ Added 23/09/22
      break;
    case EVT_PA1_LONGPRESS:
      bandDown();           // G6LBQ Added 23/09/22 
      break;
      
    case EVT_PC14_BTNUP:    // VFO A/B Select
      vfosel = vfosel^1;
 //   rit = rit^1;          // toggle the bottom bit onoff
      triggerValueChange(0); 
      setEEPROMautoSave();
      break;

    case EVT_PC14_LONGPRESS: // VFO A/B Select
      vfosel = vfosel^1;
      triggerValueChange(0);
      setEEPROMautoSave();
      break;  

    case EVT_PA0_BTNUP:      // MODE button
      setmodeDown();
      break;
    case EVT_PA0_LONGPRESS:  // MODE button
      setmodeUp();
      break;        

    case EVT_PB5_BTNUP:                        // G6LBQ 29/11/2022 added for RF PreAmp button
      rfpre = (rfpre + 1) % _N(rfpre_label);
      triggerValueChange(0);
      break;

   case EVT_PB5_LONGPRESS:                     // G6LBQ 29/11/2022 added for RF PreAmp button
      rfpre = rfpre - 1;
      if (rfpre < 0) {
        rfpre = _N(rfpre_label) - 1;
      }
      triggerValueChange(0);
      break;
    
    case EVT_PB11_BTNUP:                        // ATTENUATOR button
      atten = (atten + 1) % _N(atten_label);
      triggerValueChange(0);
      break;

   case EVT_PB11_LONGPRESS:                     // ATTENUATOR button
      atten = atten - 1;
      if (atten < 0) {
        atten = _N(atten_label) - 1;
      }
      triggerValueChange(0);
      break;

    case EVT_NOCHANGE:
      break; // nothing to do  
  }

  if (menumode) {
    processMenu();
  } else {
    // only process the encoder when not in menu mode. This is because encoder counts could occur during the slow screen painting
    if (readEncoder(ROTARY_HIGH_SENSITIVITY)) {
      // update the frequency that's tuned
      vfo[vfosel] += readEncoder(ROTARY_HIGH_SENSITIVITY)*fstepRates[stepsize];
      if (vfo[vfosel] < 500000) vfo[vfosel] = 500000;
      if (vfo[vfosel] > 30000000) vfo[vfosel] = 30000000;
      triggerVFOChange();
      resetEncoder();
      setEEPROMautoSave();
    }
  }

  // transmit input processing: 
  // Assume the signal is electronically created and does _NOT_ need debouncing
  // Look for transitions on the input signal
  lastTXInput = (lastTXInput<<4) | !(digitalRead(SW_TX)); //SX_TX is active low
  switch(lastTXInput) {
    case 0x01:
      // transition from rx to tx
      transmitting = true;
      updateAllFreq();
      printTXstate(transmitting);
      break;
    case 0x10:
      // transition from tx to rx
      transmitting = false;
      printTXstate(transmitting);
      updateAllFreq();
      break;
  }

  if (EEPROMautoSave && (millis()>EEPROMautoSave)) {
    EEPROMautoSave = 0;
    paramAction(SAVE);  // save current parameter values. Only changed values are actually written
  }
  
  // debug output
  if (Serial.available()) {
    int ch = Serial.read();
    if (ch == 't') {
      // debug measure screen performance - updateScreen() and changing fonts.
      long dt = micros();
      updateScreen();
      updateScreen();
      dt = micros() - dt;
      traceLog("Time for for updateScreen: %i\n", dt);
      dt = micros();
      ucg.setFont(fontHuge);
      ucg.getStrWidth(" ");
      ucg.setFont(fontLarge);
      ucg.getStrWidth(" ");
      ucg.setFont(fontMedium);
      ucg.getStrWidth(" ");
      ucg.setFont(fontTiny);
      ucg.getStrWidth(" ");
      dt = micros() - dt;
      traceLog("Time for for setFont, getStrWidth: %i\n", dt);
      
    } else if (ch == 'd') { 
      // debug only to measure screen write time
      ucg.setPrintPos( 0, 198);
      long dt = micros();
      ucg.print("Time this string1234");
      dt = micros() - dt;
      traceLog("Time for 20 chars: %i\n", dt);
    } else if (ch == 's') {
      // debug only to display some state variables
      traceLog("menumode: %i\n", menumode);
      traceLog("mode: %i\n", mode);
      traceLog("filt: %i\n", filt);
      traceLog("stepsize: %i\n", stepsize);
      traceLog("vfosel: %i\n", vfosel);
      traceLog("rit: %i\n", rit);
      traceLog("ritFreq: %i\n", ritFreq);
      traceLog("vfo[VFOA]: %i\n", vfo[VFOA]);
      traceLog("vfo[VFOB]: %i\n\n\n", vfo[VFOB]);
    }
  }
}
  
