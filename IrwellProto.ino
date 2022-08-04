///////////////////////////////////////////////////////////////////////////////////
//   G6LBQ Irwell HF Transceiver VFO - Version 1.0
//   stm32 + si5351a VFO With BFO & Conversion Oscilator
//   
//   Expanded with Multiple SI5351 & I/O Expanders by G6LBQ  
//
//   Created by G6LBQ on 15/09/2020 
///////////////////////////////////////////////////////////////////////////////////


#define BLUEPILL    // uncomment to tweak some i/o ports for the blue pill, and enable Serial
#define MOCKI2C     // uncomment this to mock transmission to the 2x pcf8574 , Si5351s
#define TRACEI2C    // uncomment to generate debug traces on I2C outputs. BLUEPILL must be defined for Serial to work
#define DEBUG

//---------- Library include ----------

#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>

#include "si5351a2.h" 
#include "src/Ucglib.h"
#include "ButtonEvents.h"

//----------   Encoder setting  ---------------

#define ENC_A     PB12                    // Rotary encoder A
//#define ENC_A     PB14                    // Rotary encoder A
#define ENC_B     PB13                    // Rotary encoder B

//----------   TFT setting  ------------------- 

#define   __CS    PB10                    // G6LBQ changed from PB10 to PB3 to free up PB10 for 2nd I2C bus    
#define   __DC    PB0                     // D/C
#define   __RST   PB1                     // RESET   

Ucglib_ILI9341_18x240x320_HWSPI ucg(__DC, __CS, __RST);

//----------  Button Setting -----------------

// define event names for the key events
typedef enum {EVT_NOCHANGE, EVT_PA0_BTNUP, EVT_PA0_LONGPRESS, EVT_PA1_BTNUP, EVT_PA1_LONGPRESS, EVT_PC14_BTNUP, EVT_PC14_LONGPRESS, EVT_PC15_BTNUP, EVT_PC15_LONGPRESS } keyEvents;

ButtonEvents b = ButtonEvents(EVT_NOCHANGE);

//----------   I/O Assign  ------------------- 

#define   OUT_LSB      PB15               // Data line for controlling modes of operation                  
#define   OUT_USB      PA8                // Data line for controlling modes of operation                  
#define   OUT_CW       PA9                // G6LBQ added extra mode selection output                  
#define   OUT_AM       PA10               // G6LBQ added extra mode selection output
#define   SW_BAND      PA0
#define   SW_STEP      PA1                 
#define   SW_MODE      PC14                 
#define   SW_RIT       PC15

//#define   SW_MODE      PB7                 
//#define   SW_RIT       PA0

#ifdef BLUEPILL
#define   LED          PC13
#define   SW_TX        PA3                // need to reassign to move away from led
#else                  
#define   SW_TX        PC13               // G6LBQ> PTT - connect to Gnd for TX
#endif

#define debugOut       PB5                // a couple of debug outputs
#define debugTriggered PB4


#define   METER        PA2                // G6LBQ changed from PA1 to PA2    

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

#define _N(a) sizeof(a)/sizeof(a[0])

enum vfo_t { VFOA=0, VFOB=1, SPLIT=2 };

// state variables stored in eeprom 
// all these variables _MUST_ be 16bit or 32bit
// their initialisation values are stored in the eeprom on a "factory" reset
// As a convenience they are in the order they appear in the menu

int16_t  vfomode[] = { MODE_USB, MODE_USB };
int16_t  filt = 0;
int16_t  bandval = 3;
int16_t  stepsize = 3;  //todo revisit - uSDX uses an enum
uint16_t vfosel = VFOA;
int16_t  rit = 0;
int16_t  ritFreq = 0;
uint32_t xtalfreq = 25000000;
uint32_t ifFreq[]  = {11056570,   11059840, 11058400, 11058200, 11058200};
uint32_t BFOFreq[] = {11056570,   11059840, 11058400,        0,        0};
int32_t  vfo[] = { 7074000, 14074000 };
int32_t  convFreq = 45000000;
uint16_t eeprom_version;

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


//---------- Rotary Encoder Processing -----------------------

volatile int8_t encoder_val = 0;
int32_t interruptCount = 0;
int32_t lastInterruptCount = -1;
int_fast32_t interruptsTimeExpired = 0; 
 
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

  digitalWrite(debugOut,!digitalRead(debugOut));
  interruptCount++;   // investigate very high interupt rate
  uint8_t state = (digitalRead(ENC_B) << 1) | digitalRead(ENC_A);
  if (!rotationEvent && (state==0x0)) {
      rotationEvent = true;
      if (last_state & 1) encoder_val++; else encoder_val--;
  } else if (rotationEvent && (state==0x3)) {
    rotationEvent = false;
  }
  last_state = state;
}

int readId;
int8_t lastReadVal;

// detect if there are dropped counts between reading and reseting.
int8_t readEncoder(int id) {
  //noInterrupts();     
  lastReadVal = encoder_val;
  //encoder_val = 0;  // future change; not possible yet
  //interrupts();
  readId = id;
  return lastReadVal;
}

void resetEncoder(int id) {
  if (encoder_val != lastReadVal) {
    //aha, these values would be missed if we weren't careful
    Serial.print("reset Encoder. reader id: ");Serial.print(readId); Serial.print(" reseter: "); Serial.print(id); 
    Serial.print(" encoder_val: "); Serial.print(encoder_val); Serial.print(" lastReadVal: "); Serial.println(lastReadVal);
  }
  encoder_val -= lastReadVal; // cheat & see if it improves things? Yes. There are lots of dropped counts otherwise
  //encoder_val = 0;
}


//--------- PCF8574 Interfacing ----------------------------------

void write_PCF8574(uint8_t address, uint8_t data) {
#ifndef MOCKI2C
  Wire.begin();
  Wire.beginTransmission(address);
  Wire.write(0b00000000);
  Wire.endTransmission();
#endif
#ifdef TRACEI2C
  Serial.print("write_PCF8574 to "); Serial.print(address); Serial.print(": "); Serial.println(data);
#endif
}

void init_PCF8574() {
  write_PCF8574(0x38, 0b00000000);  //initialize PCF8574 I/O expander 1 for BPF filters 1 to 5 
  write_PCF8574(0x39, 0b00000000);  //initialize PCF8574 I/O expander 2 for BPF filters 6 to 10
}

//--------- Filter Bank Selection ---------------------------------

void select_bank(int8_t bank) {
  static int8_t currentBank = -1;
  // bank 0 corresponds to 0x38, port 1, bank 6 is 0x39, port 1 etc.
  if (bank == currentBank)
    // the correct bank is already selected; nothing to do here
    return;
  init_PCF8574();   // turn off all banks
  int bit = bank % 6;
  int bankOffset = bank/6;
  write_PCF8574(0x38 + bankOffset, 1<<bit);
  currentBank = bank;
}

void select_BPF(uint32_t freq) {

  // HF Band Pass Filter logic added by G6LBQ
  if        (freq <  1600000){ select_bank(0);
  } else if (freq <  2000000){ select_bank(1);
  } else if (freq <  3000000){ select_bank(2);
  } else if (freq <  4000000){ select_bank(3);
  } else if (freq <  6000000){ select_bank(4);
  } else if (freq <  8000000){ select_bank(5);
  } else if (freq < 11000000){ select_bank(6);
  } else if (freq < 15000000){ select_bank(7);
  } else if (freq < 22000000){ select_bank(8);
  } else if (freq < 30000000){ select_bank(9);
  } 
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
#ifdef TRACEI2C
  Serial.print("Si5351 port: "); Serial.print(port); Serial.print(", chl: "); Serial.print(channel); Serial.print(", freq: "); Serial.print(frequency);
  Serial.print(", xtalFreq: "); Serial.println(xtalfreq);
#endif
}

void setFrequency(uint8_t port, uint8_t channel, uint32_t frequency, uint32_t xtalFreq) {
  Serial.print("setFrquency port: "); Serial.print(port); Serial.print(", chl: "); Serial.print(channel); Serial.print(", freq: "); Serial.println(frequency);
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
#ifdef TRACEI2C
  Serial.print("Si5351 disable port: "); Serial.print(port); Serial.print(", chl: "); Serial.println(channel);
#endif
}


//----------  EEPROM Routines  ---------

int eeprom_addr;
#define EEPROM_OFFSET 0x0
#define get_version_id() 3

void eeprom_init() {
  EEPROM.init();  // I think default parameters are fine
}

// temporary functions to test eeprom - remove when eeprom has been tested a bit more
void EEPROM_read(uint16 Address, uint16 *Data){
  Serial.print("EEPROM_read: from "); Serial.print((int) Address); Serial.print(", To: "); Serial.println((int)Data);
  if (Address==23)
    *Data = 1234; // test
  else
    *Data = 0x4321; // for 32 bits = 1126253345
}

void EEPROM_write(uint16 Address, uint16 Data) {
  Serial.print("EEPROM_write: to "); Serial.print((int) Address); Serial.print(", Data: "); Serial.println(Data);
}

// The menu system reads & writes in units of 16 bits

void eeprom_read_block (void *__dst, const void *__src, size_t __n) {
  //Serial.print("eeprom_read_block: from "); Serial.print((int) __src); Serial.print(", length: "); Serial.println(__n);
  for (int i=0; i<__n; i++) {
    EEPROM.read( ((int)__src)+i,  (uint16_t *)__dst+i );
  }
};

void eeprom_write_block(const void *__src, void *__dst, size_t __n) {
  //Serial.print("eeprom_write_block: to "); Serial.print((int) __dst); Serial.print(", length: "); Serial.println(__n);
  for (int i=0; i<__n; i++) {
    EEPROM.write( ((int)__dst)+i, *((uint16_t *)__src+i) ); // only update "eeprom" if the value has changed. 
  }
};

// -----------menu system labels ----------------------------------

const char* mode_label[]       = { "LSB", "USB", "CW ", "AM ", "FM " };
const char* filt_label[]       = { "Full", "7", "6", "5", "4", "3", "2", "1" };

// Band information - last 4 are made up. TODO - provide proper values
const char* band_label[]       = { "160m",      "80m",    "60m",   "40m",    "30m",    "20m",    "17m",    "15m",    "12m",     "10m",     "6m" };
const uint8_t bandMode[]  =      { MODE_LSB, MODE_LSB,  MODE_CW, MODE_USB, MODE_USB, MODE_USB,  MODE_AM,  MODE_FM, MODE_USB, MODE_USB, MODE_USB };
const uint32_t bandFreq[] =      {  3500000,  7000000, 10100000, 14000000, 21000000, 28000000, 30000000, 31000000, 32000000, 33000000, 34000000 };

const char* stepsize_label[]   = { "1Hz", "10Hz", "100Hz", "1KHz", "10KHz", "100KHz", "1MHz" };
const char* vfosel_label[]     = { "VFO A", "VFO B"/*, "Split"*/ };
const char* offon_label[]      = {"OFF", "ON"};


//------------- display subsystem  --------------------------------

void printBlanks(){
  //Serial.println(" printBlanks");
  ucg.print("        ");
}
void setCursor(int x, int y) {
  //Serial.print(" setCursor("); Serial.print(x); Serial.print(", "); Serial.print(y);Serial.println(")");
  ucg.setPrintPos(x*12, y*22+22);
}

void printVFO(int sel) {
  ucg.print(vfosel_label[sel]); ucg.print(" ");ucg.print(mode_label[vfomode[sel]]); ucg.print(" ");ucg.print(vfo[sel]);printBlanks();
}

void printPrimaryVFO(int sel) {
  setCursor(0, 3); printVFO(sel);
}

void printSecondaryVFO(int sel) {
  setCursor(0, 4); printVFO(sel);
}

void printStep(int stepsize) {
  setCursor(0, 5); ucg.print("Tune Rate ");ucg.print(stepsize_label[stepsize]);printBlanks();
}

void printRITFreq(int16_t freq) {
  setCursor(9, 6); ucg.print(freq); printBlanks();
}

void printRIT(int rit, int16_t freq) {
  setCursor(0, 6); ucg.print("RIT ");ucg.print(offon_label[rit]); ucg.print("  ");
  printRITFreq(freq);
}

void printTXstate(bool tx) {
  setCursor(8, 7);
  if (tx) {
    ucg.setColor(255,0,0); ucg.print("--TX--"); ucg.setColor(255,255,255);
  } else {
    ucg.print("      ");
  }
}
//--------------- Business Logic---------------------------------------------

#define firstIF 45000000L       // Added by G6LBQ 01/07/2022



void updateModeOutputs(uint8_t mode) {
  digitalWrite(OUT_LSB, mode==MODE_LSB);
  digitalWrite(OUT_USB, mode==MODE_USB);
  digitalWrite(OUT_CW,  mode==MODE_CW);       // G6LBQ added 1/11/20
  digitalWrite(OUT_AM,  mode==MODE_AM);       // G6LBQ added 1/11/20
}

// Original code was for single conversion IF at 11.059MHz so 11.059MHz + VFO Frequency
// 01/07/2022 Changes are for dual conversion so 45MHz firstIF + VFO Frequency 

void updateAllFrquencyOutputsDualConversionOld(uint8_t mode, int32_t freq, int32_t ifshift, int32_t freqRIT, bool transmitting, int32_t xtalFreq) {
  select_BPF(freq);
  updateModeOutputs(mode);
  uint32_t vfofreq = freq + firstIF + freqRIT;
  if ((mode==MODE_CW) && transmitting)  // is in the original code. Is this correct? Isn't rx offset from the tuned freq to create the tone (or BFO adds)?
    vfofreq += CW_TONE;
  setFrequency(VFO_PORT, VFO_CHL, vfofreq, xtalFreq);
  
  if (mode!=MODE_AM) {
    setFrequency(BFO_PORT, BFO_CHL, ifshift, xtalFreq);
  } else {
    disableFrequency(BFO_PORT, BFO_CHL);
  }
  setFrequency(CONV_PORT, CONV_CHL, conversionOffsets[mode], xtalFreq);
}
/*
void updateAllFrquencyOutputsDualConversion(uint8_t mode, int32_t freq, int32_t ifshift, int32_t freqRIT, bool transmitting) {
  select_BPF(freq);
  updateModeOutputs(mode);
  uint32_t vfofreq = freq + firstIF;
  setFrequency(VFO_PORT, VFO_CHL, vfofreq);
  switch(mode) {
    case MODE_USB: MODE_LSB:
      setFrequency(BFO_PORT, BFO_CHL, bfoFreq);
      break;
    case MODE_CW:
      setFrequency(BFO_PORT, BFO_CHL, bfoFreq + CW_TONE);
      break;
    case MODE_AM:
      disableFrequency(BFO_PORT, BFO_CHL);
      break;
    case MODE_FM:
      disableFrequency(BFO_PORT, BFO_CHL);
      break;
  }
}
*/

void updateAllFrquencyOutputs(uint8_t mode, int32_t freq, int32_t finalIF, int32_t bfoFreq, int32_t freqRIT, bool transmitting, int32_t xtalFreq) {
  select_BPF(freq);
  updateModeOutputs(mode);
  uint32_t vfofreq = freq + finalIF + freqRIT;
  setFrequency(VFO_PORT, VFO_CHL, vfofreq, xtalFreq);
  switch(mode) {
    case MODE_USB: MODE_LSB:
      setFrequency(BFO_PORT, BFO_CHL, bfoFreq, xtalFreq);
      break;
    case MODE_CW:
      setFrequency(BFO_PORT, BFO_CHL, bfoFreq + CW_TONE, xtalFreq);
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
  //Serial.print("setEEPROMautoSave = "); Serial.println(EEPROMautoSave);
}

void updateAllFreq() {
  updateAllFrquencyOutputs(vfomode[vfosel], vfo[vfosel], ifFreq[vfomode[vfosel]], BFOFreq[vfomode[vfosel]], rit ? ritFreq : 0, transmitting, xtalfreq);
  setEEPROMautoSave();
}

void triggerVFOChange() {
  updateAllFreq();
  printPrimaryVFO(vfosel);
}

void triggerRITChange(int menu) {
  updateAllFreq();
  printRITFreq(ritFreq);
}

void triggerBandChange(int menu) {
  vfomode[vfosel] = bandMode[bandval];
  vfo[vfosel] = bandFreq[bandval];
  triggerValueChange(0);
}

// Anything could have changed so calculate _everything_
void triggerValueChange(int menu) {
  // give all values to the output routine
  updateAllFreq();
  printPrimaryVFO(vfosel);
  printSecondaryVFO(vfosel^1);
  printStep(stepsize);
  printRIT(rit, ritFreq);
}

void triggerNoop(int menu) {}



//-------------- encoder tune rate  -----------

void setstepUp(){
  stepsize = (stepsize + 1) % fstepRatesSize;
  printStep(stepsize);
  setEEPROMautoSave();
}

void setstepDown() {
  if (stepsize==0) stepsize = fstepRatesSize;
  stepsize--;
  printStep(stepsize);
  setEEPROMautoSave();
}

void bandUp() {
  bandval = (bandval + 1) % _N(band_label);
  triggerBandChange(0);
}

void bandDown() {
  if (bandval==0) bandval = _N(band_label);
  bandval--;
  triggerBandChange(0);
}


//------------- menu system --------------------------------

enum menu_t { NO_MENU, MENU_SELECTED, MENU_VALUE };
uint8_t menumode = 0;  // 0=not in menu, 1=selects menu item, 2=selects parameter value //todo - use enums
int8_t  menu = 1;  // current parameter id selected in menu


enum action_t { UPDATE, UPDATE_MENU, NEXT_MENU, LOAD, SAVE, SKIP, NEXT_CH };
enum params_t {NULL_, MODE, FILTER, BAND, STEP, VFOSEL, RIT, RITFREQ, SIFXTAL, IF_LSB, IF_USB, IF_CW, IF_AM, IF_FM, BFO_LSB, BFO_USB, BFO_CW, FREQA, FREQB, MODEA, MODEB, VERS, ALL=0xff};
#define N_PARAMS 16                 // number of (visible) parameters
#define N_ALL_PARAMS (N_PARAMS+5)  // total of all parameters


void show_banner(){
  setCursor(0, 0);
  ucg.print("Irwell TXCVR V0.1");
  printBlanks(); printBlanks();
  setCursor(0, 1);
  printBlanks(); printBlanks();
}

void printmenuid(uint8_t menuid){ // output menuid in x.y format
  Serial.print("printmenuid "); Serial.println(menuid);
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
  if(action == UPDATE_MENU){
    setCursor(0, 0);
    printmenuid(menuid);
    ucg.print(label); printBlanks(); printBlanks();
    setCursor(0, 1); // value on next line
    if(menumode >= MENU_VALUE) ucg.print('>');
  } else { // UPDATE (not in menu)
    setCursor(0, 1); ucg.print(label); ucg.print(": ");
  }
}

void actionCommon(uint8_t action, uint8_t *ptr, uint8_t size){
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
  int32_t delta = readEncoder(1);
  switch(action){
    case UPDATE:
    case UPDATE_MENU:
      if (sizeof(T) == sizeof(uint32_t)) delta *= fstepRates[stepsize];  // large menu items use the tune-rate
      value = (int32_t)value + delta;
      if(     value < _min) value = _min;
      else if(value > _max) value = _max;
      resetEncoder(2);

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
      
    default:
      actionCommon(action, (uint8_t *)&value, sizeof(value)/2); // /2 because the eeprom lib stores in units of 16 bits
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
    case MODE:    paramAction(action, vfomode[vfosel],0x11,      "Mode",     mode_label,        0,     _N(mode_label)-1, triggerValueChange); break;
    case FILTER:  paramAction(action, filt,           0x12, "NR Filter",     filt_label,        0,     _N(filt_label)-1, triggerValueChange); break;
    case BAND:    paramAction(action, bandval,        0x13,      "Band",     band_label,        0,     _N(band_label)-1, triggerBandChange ); break;
    case STEP:    paramAction(action, stepsize,       0x14, "Tune Rate", stepsize_label,        0, _N(stepsize_label)-1, triggerValueChange); break;
    case VFOSEL:  paramAction(action, vfosel,         0x15,  "VFO Mode",   vfosel_label,        0,   _N(vfosel_label)-1, triggerValueChange); break;
    case RIT:     paramAction(action, rit,            0x16,       "RIT",    offon_label,        0,                    1, triggerValueChange); break;
    case RITFREQ: paramAction(action, ritFreq,        0x17,"RIT Offset",           NULL,    -1000,                 1000, triggerRITChange  ); break;
    case SIFXTAL: paramAction(action, xtalfreq,       0x81,  "Ref freq",           NULL, 14000000,             28000000, triggerValueChange); break;
    case IF_LSB:  paramAction(action, ifFreq[0],      0x82,    "IF-LSB",           NULL, 14000000,             28000000, triggerValueChange); break;
    case IF_USB:  paramAction(action, ifFreq[1],      0x83,    "IF-USB",           NULL, 14000000,             28000000, triggerValueChange); break;
    case IF_CW:   paramAction(action, ifFreq[2],      0x84,     "IF-CW",           NULL, 14000000,             28000000, triggerValueChange); break;
    case IF_AM:   paramAction(action, ifFreq[3],      0x85,     "IF-AM",           NULL, 14000000,             28000000, triggerValueChange); break;
    case IF_FM:   paramAction(action, ifFreq[4],      0x86,     "IF-FM",           NULL, 14000000,             28000000, triggerValueChange); break;
    case BFO_LSB: paramAction(action, BFOFreq[0],     0x87,   "BFO-LSB",           NULL, 14000000,             28000000, triggerValueChange); break;
    case BFO_USB: paramAction(action, BFOFreq[1],     0x88,   "BFO-USB",           NULL, 14000000,             28000000, triggerValueChange); break;
    case BFO_CW:  paramAction(action, BFOFreq[2],     0x89,    "BFO-CW",           NULL, 14000000,             28000000, triggerValueChange); break;

    // invisible parameters. These are here only for eeprom save/restore
    case FREQA:   paramAction(action, vfo[VFOA],         0,        NULL,           NULL,         0,                    0, triggerNoop       ); break;
    case FREQB:   paramAction(action, vfo[VFOB],         0,        NULL,           NULL,         0,                    0, triggerNoop       ); break;
    case MODEA:   paramAction(action, vfomode[VFOA],     0,        NULL,           NULL,         0,                    0, triggerNoop       ); break;
    case MODEB:   paramAction(action, vfomode[VFOB],     0,        NULL,           NULL,         0,                    0, triggerNoop       ); break;
    case VERS:    paramAction(action, eeprom_version,    0,        NULL,           NULL,         0,                    0, triggerNoop       ); break;
    
    // case NULL_:   menumode = NO_MENU; show_banner(); break;
    default:      if((action == NEXT_MENU) && (id != N_PARAMS)) 
                      id = paramAction(action, max(1 /*0*/, min(N_PARAMS, id + ((readEncoder(3) > 0) ? 1 : -1))) ); break;  // keep iterating util menu item found
  }
  return id;  // needed?
}

void processMenuKey() {
  Serial.println("processmenukey");
    if     (menumode == 0){ menumode = 1; paramAction(UPDATE_MENU, menu);}  // enter menu mode
    else if(menumode == 1){ menumode = 2; paramAction(UPDATE_MENU, menu);}                          // enter value selection screen
    else if(menumode >= 2){ Serial.println("test menu"); paramAction(SAVE, menu); menumode = 0; show_banner();}  // save value, and return to default screen
}

void processEnterKey() {
  if     (menumode == 1){ menumode = 0; show_banner();}  
  else if(menumode >= 2){ Serial.println("test enter"); menumode = 1; paramAction(UPDATE_MENU, menu); paramAction(SAVE, menu); } // save value, and return to menu mode

}

void processMenu() {
  if (menumode) {
    int8_t encoder_change = readEncoder(4);
    if((menumode == 1) && encoder_change){
      //Serial.println("menu - got encoder val "); Serial.println(encoder_change);
      menu += readEncoder(5);   // Navigate through menu
      menu = max(1 , min(menu, N_PARAMS));
      menu = paramAction(NEXT_MENU, menu);  // auto probe next menu item (gaps may exist)
      resetEncoder(5);
    }
    if(encoder_change)
      paramAction(UPDATE_MENU, menu);  // update param with encoder change and display.
  }
  
}

//------------------  Initialization -------------------------
 
void setup() {
#ifdef BLUEPILL
  while (!Serial)
    ;
  Serial.println("Starting setup");
  pinMode(LED, OUTPUT);
#endif

  initRotary();
  
  ucg.begin(UCG_FONT_MODE_SOLID);
  ucg.clearScreen();
  ucg.setRotate270();
  ucg.setColor(1, 0, 0, 0);       // set background color
  ucg.setColor(255,255,255);
  ucg.setFont(ucg_font_inb16_mr);   // arbitrary font for the moment; must be fixed-size


  b.add(SW_BAND, EVT_PA0_BTNUP, EVT_PA0_LONGPRESS);
  b.add(SW_STEP, EVT_PA1_BTNUP, EVT_PA1_LONGPRESS);
  b.add(SW_MODE, EVT_PC14_BTNUP, EVT_PC14_LONGPRESS);
  b.add(SW_RIT,  EVT_PC15_BTNUP, EVT_PC15_LONGPRESS);
  
  pinMode(SW_TX,INPUT_PULLUP);
  pinMode(OUT_LSB,OUTPUT);                    // LSB Mode 
  pinMode(OUT_USB,OUTPUT);                    // USB Mode
  pinMode(OUT_CW,OUTPUT);                     // CW Mode - G6LBQ added additional mode selection
  pinMode(OUT_AM,OUTPUT);                     // AM Mode - G6LBQ added additional mode selection

  pinMode(debugOut, OUTPUT);                  // temp for debugging
  pinMode(debugTriggered, OUTPUT);

  init_PCF8574();
/* test display, font sizes
  setCursor(0, 0); ucg.print("Line 0");
  setCursor(1, 1); ucg.print("offset x");
  setCursor(2, 2); ucg.print("Line 2, offset 2");
  setCursor(0, 3); ucg.print("Line 3");
  setCursor(0, 4);ucg.print(42);
*/

  eeprom_init();
  // Load parameters from EEPROM, reset to factory defaults when 
  // 1. stored values are from a different version
  // 2. SW_KEY pressed during power-up
  
  Serial.println("checking version");
  paramAction(LOAD, VERS);
  Serial.print("On initial load eeprom_version: "); Serial.println(eeprom_version);
  if((eeprom_version != get_version_id()) || !digitalRead(SW_STEP)){
    Serial.print("Reload - eeprom_version: "); Serial.println(eeprom_version);
    eeprom_version = get_version_id();
    paramAction(SAVE);  // save default parameter values
    setCursor(0, 1); ucg.print("Reset settings..");
    delay(1000);
  }
  paramAction(LOAD);  // load all parameters
  Serial.print("After Load-all eeprom_version: "); Serial.println(eeprom_version);
  
  /*
  Serial.print("eeprom_version before: "); Serial.println(eeprom_version);
  paramAction(LOAD, VERS);
  Serial.print("eeprom_version after read: "); Serial.println(eeprom_version);
  eeprom_version = 1234;
  paramAction(SAVE, VERS);
  paramAction(LOAD, VERS);
  Serial.print("eeprom_version after writing 1234: "); Serial.println(eeprom_version);
  
  
  Serial.print("xtalfreq before: "); Serial.println(xtalfreq);
  paramAction(LOAD, SIFXTAL);
  Serial.print("xtalfreq after read: "); Serial.println(xtalfreq);
  xtalfreq = 0x12345678;
  paramAction(SAVE, SIFXTAL);
  paramAction(LOAD, SIFXTAL);
  Serial.print("xtalfreq after writing 0x12345678: "); Serial.println(xtalfreq);
  */
  
  show_banner();
  triggerValueChange(0);
}

//----------  Main program  ------------------------------------

void loop() {
#ifdef BLUEPILL
  digitalWrite(LED, !digitalRead(LED));
#endif

#define DEBUG
#ifdef DEBUG
  if ( interruptCount != lastInterruptCount ) {
    if (millis() > interruptsTimeExpired) { 
      Serial.print("Interrupts: "); Serial.println(interruptCount);
      lastInterruptCount = interruptCount;
      interruptsTimeExpired = millis() + 1000;  // update max of one a second
      //delay(1000);
    }
  }
#endif

  
  int event = b.getButtonEvent();
  switch (event) {
     case EVT_PC15_BTNUP: // was the RIT button
      processMenuKey();
      break;
      
    case EVT_PC14_BTNUP:  // was the MODE button
      processEnterKey();
      break;
      
    case EVT_PA1_BTNUP:     // STEP button
      setstepDown();
      break;
    case EVT_PA1_LONGPRESS: // STEP button
      setstepUp();
      break;
      
    case EVT_PA0_BTNUP:     // add back the BAND button
      break;
    case EVT_PA0_LONGPRESS:
      break;
      
    case EVT_NOCHANGE:
      break; // nothing to do
  }

  if (menumode) {
    processMenu();
  } else {
    // only process the encoder when not in menu mode. This is because encoder counts could occur during the slow screen painting
    if (readEncoder(6)) {
      // update the frequency that's tuned
      vfo[vfosel] += readEncoder(7)*fstepRates[stepsize];
      triggerVFOChange();
      resetEncoder(8);
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
    setCursor(0, 8);
    long dt = micros();
    //displayRawText("Time this string1234", 20, 20, DISPLAY_RED, DISPLAY_DARKGREEN);
    ucg.print("Time this string1234");
    dt = micros() - dt;
    Serial.print("Time for 20 chars: "); Serial.println(dt);
    
    Serial.print("menumode: "); Serial.println(menumode);
    Serial.print("mode: "); Serial.println(vfomode[vfosel]);
    Serial.print("filt: "); Serial.println(filt);
    Serial.print("stepsize: "); Serial.println(stepsize);
    Serial.print("vfosel: "); Serial.println(vfosel);
    Serial.print("rit: "); Serial.println(rit);
    Serial.print("ritFreq: "); Serial.println(ritFreq);
    Serial.print("vfo[VFOA]: "); Serial.println(vfo[VFOA]);
    Serial.print("vfo[VFOB]: "); Serial.println(vfo[VFOB]);
    Serial.print("vfomode[VFOA]: "); Serial.println(vfomode[VFOA]);
    Serial.print("vfomode[VFOB]: "); Serial.println(vfomode[VFOB]);
    Serial.println(); Serial.println();
    
    
  }
}
  
