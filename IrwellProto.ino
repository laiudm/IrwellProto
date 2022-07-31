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

// state variables

uint8_t mode = MODE_USB;
uint8_t filt = 0;
uint8_t bandval = 3;
uint8_t stepsize = 3;  //todo revisit - uSDX uses an enum
uint32_t xtalfreq = 25000000;
unsigned long if_bfo[]= {11056570, 11059840, 11058400, 11058200};
enum vfo_t { VFOA=0, VFOB=1, SPLIT=2 };
uint8_t vfosel = VFOA;
int16_t rit = 0;
int16_t ritFreq = 0;
int32_t vfo[] = { 7074000, 14074000 };
uint8_t vfomode[] = { MODE_USB, MODE_USB };

static const uint32_t conversionOffsets[] { 
  56059000, // LSB
  33941000, // USB
  56059000, // CW
  56059000  // AM
};


//---------- Rotary Encoder Processing -----------------------

volatile int8_t encoder_val = 0;
long interruptCount = 0;
long lastInterruptCount = -1;
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

void select_BPF(long freq) {

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

long currentFrequency[] = { -1, -1, -1 };   // track output frequences for changes

void _setFrequency(uint8_t port, uint8_t channel, uint32_t frequency) {
#ifndef MOCKI2C
  tcaselect(port);
  si5351aSetFrequency(channel, frequency, xtalfreq);
#endif
#ifdef TRACEI2C
  Serial.print("Si5351 port: "); Serial.print(port); Serial.print(", chl: "); Serial.print(channel); Serial.print(", freq: "); Serial.print(frequency);
  Serial.print(", xtalFreq: "); Serial.println(xtalfreq);
#endif
}

void setFrequency(uint8_t port, uint8_t channel, uint32_t frequency) {
  // don't need to track port and channel; port is unique
  if (currentFrequency[port] == frequency)
    // it's already set to the correct frequency; nothing further to do here
    return;
  _setFrequency(port, channel, frequency);
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

void Fnc_eepINIT(){
  uint16 dummy;
  //Refer to ...\arduino-1.8.13\portable\packages\stm32duino\hardware\STM32F1\2021.5.31\libraries\EEPROM\EEPROM.cpp, EEPROM.h 
  //Serial.print("EEPROM_PAGE0_BASE = "); Serial.print(EEPROM_PAGE0_BASE, HEX); Serial.print(" "); Serial.println(EEPROM_PAGE0_BASE == 0x801F000);
  //Serial.print("EEPROM_PAGE1_BASE = "); Serial.print(EEPROM_PAGE1_BASE, HEX); Serial.print(" "); Serial.println(EEPROM_PAGE1_BASE == 0x801F800);
  //Serial.print("EEPROM_PAGE_SIZE = ");  Serial.print(EEPROM_PAGE_SIZE,  HEX); Serial.print(" "); Serial.println(EEPROM_PAGE_SIZE  == 0x400);
  //long addr = (long)&bandwrite;
  //Serial.print("Addr = "); Serial.println(addr, HEX);
  EEPROM.PageBase0 = 0x801F000;         // 0x801F800 default values. So these settings move the "eeprom" storage down a bit. Why? Not for the bootloader - it's at the start of memory
  EEPROM.PageBase1 = 0x801F800;         // 0x801FC00 
  EEPROM.PageSize  = 0x400;             // 1kB
  dummy = EEPROM.init();
}

long Fnc_eepRD(uint16 adr){
  long val = 0;
  uint16 dat,dummy;  

  dummy = EEPROM.read(adr,&dat);
  val = dat << 16;
  dummy = EEPROM.read(adr+1,&dat);
  return val | dat;
}

void Fnc_eepWT(long dat,uint16 adr){
  uint16 dummy,val;

  val = dat & 0xffff;
  dummy = EEPROM.write(adr+1,val);
  val = dat >> 16;
  val = val & 0xffff;
  dummy = EEPROM.write(adr,val);
}

// todo - moch interfaces currently
void eeprom_read_block (void *__dst, const void *__src, size_t __n) {
  Serial.print("eeprom_read_block: from "); Serial.print((int) __src); Serial.print(", length: "); Serial.println(__n);
};

void eeprom_write_block(const void *__src, void *__dst, size_t __n) {
  Serial.print("eeprom_write_block: to "); Serial.print((int) __dst); Serial.print(", length: "); Serial.println(__n);
};

//------------- temp vars, routines --------------------------------

uint8_t lastTXInput = 0;   // captures last state and current state of TX input line.
bool transmitting = false;  // status indication - used to calcute VFO, BFO freqs

long freq;
long freqrit;
long vfofreq;
long ifshift;
long firstIF;
bool flg_bfochg;
bool flagrit;
int fmode;
long fstep;
static const long fstepRates[] = {1, 10, 100, 1000, 10000, 100000, 1000000};
#define fstepRatesSize (sizeof(fstepRates)/sizeof(fstepRates[0]))

int eeprom_addr;
#define EEPROM_OFFSET 0x0
#define get_version_id() 1

uint8_t eeprom_version;



void lcdnoCursor() {};  // really is nothing to do here. Unless the display does have a cursor?


// -----------menu system labels ----------------------------------

const char* mode_label[]       = { "LSB", "USB", "CW ", "AM ", "FM " };
const char* filt_label[]       = { "Full", "7", "6", "5", "4", "3", "2", "1" };
const char* band_label[]       = { "160m", "80m", "60m", "40m", "30m", "20m", "17m", "15m", "12m", "10m", "6m" };
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

void printRIT(int rit) {
  setCursor(0, 6); ucg.print("RIT ");ucg.print(offon_label[rit]); ucg.print("  "); ucg.print(ritFreq); printBlanks();
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

void updateAllFrquencyOutputs(uint8_t mode, int32_t freq, int32_t ifshift, int32_t freqRIT, bool transmitting) {
  select_BPF(freq);
  updateModeOutputs(mode);
  int32_t vfofreq = freq + firstIF + freqRIT;
  if ((mode==MODE_CW) && transmitting)  // is in the original code. Is this correct? Isn't rx offset from the tuned freq to create the tone (or BFO adds)?
    vfofreq += CW_TONE;
  setFrequency(VFO_PORT, VFO_CHL, vfofreq);
  
  if (mode!=MODE_AM) {
    setFrequency(BFO_PORT, BFO_CHL, ifshift);
  } else {
    disableFrequency(BFO_PORT, BFO_CHL);
  }
  setFrequency(CONV_PORT, CONV_CHL, conversionOffsets[mode]);
}


//--------------- value change triggers --------------------------------------

void updateAllFreq() {
  updateAllFrquencyOutputs(vfomode[vfosel], vfo[vfosel], if_bfo[vfomode[vfosel]], rit ? ritFreq : 0, transmitting);
}

// Band information - last 4 are made up. TODO - provide proper values
const uint8_t bandMode[]  = { MODE_LSB, MODE_LSB,  MODE_CW, MODE_USB, MODE_USB, MODE_USB,  MODE_AM,  MODE_FM, MODE_USB, MODE_USB };
const uint32_t bandFreq[] = {  3500000,  7000000, 10100000, 14000000, 21000000, 28000000, 30000000, 31000000, 32000000, 33000000 };

void triggerVFOChange() {
  updateAllFreq();
  printPrimaryVFO(vfosel);
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
  printRIT(rit);
}

void triggerNoop(int menu) {}



//-------------- encoder tune rate  -----------

void setstepUp(){
  stepsize = (stepsize + 1) % fstepRatesSize;
  printStep(stepsize); 
}

void setstepDown() {
  if (stepsize==0) stepsize = fstepRatesSize;
  stepsize--;
  printStep(stepsize);
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
uint8_t menu = 1;  // current parameter id selected in menu


enum action_t { UPDATE, UPDATE_MENU, NEXT_MENU, LOAD, SAVE, SKIP, NEXT_CH };
enum params_t {NULL_, MODE, FILTER, BAND, STEP, VFOSEL, RIT, RITFREQ, SIFXTAL, IF_LSB, IF_USB, IF_CW, IF_AM, FREQA, FREQB, MODEA, MODEB, VERS, ALL=0xff};
#define N_PARAMS 12                 // number of (visible) parameters
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
      //if(menuid==0x18) Serial.print("In menu ritFreq ");Serial.println(delta);
      if (sizeof(T) == sizeof(uint32_t)) delta *= fstepRates[stepsize];  // large menu items use the tune-rate
      value = (int32_t)value + delta;
      if(     value < _min) value = _min;
      else if(value > _max) value = _max;
      resetEncoder(2);

      //lcdnoCursor();
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
      actionCommon(action, (uint8_t *)&value, sizeof(value));
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
    case FILTER:  paramAction(action, filt,           0x13, "NR Filter",     filt_label,        0,     _N(filt_label)-1, triggerValueChange); break;
    case BAND:    paramAction(action, bandval,        0x14,      "Band",     band_label,        0,     _N(band_label)-1, triggerBandChange ); break;
    case STEP:    paramAction(action, stepsize,       0x15, "Tune Rate", stepsize_label,        0, _N(stepsize_label)-1, triggerValueChange); break;
    case VFOSEL:  paramAction(action, vfosel,         0x16,  "VFO Mode",   vfosel_label,        0,   _N(vfosel_label)-1, triggerValueChange); break;
    case RIT:     paramAction(action, rit,            0x17,       "RIT",    offon_label,        0,                    1, triggerValueChange); break;
    case RITFREQ: paramAction(action, ritFreq,        0x18,"RIT Offset",           NULL,    -1000,                 1000, triggerValueChange); break;
    case SIFXTAL: paramAction(action, xtalfreq,       0x83,  "Ref freq",           NULL, 14000000,             28000000, triggerValueChange); break;
    case IF_LSB:  paramAction(action, if_bfo[0],      0x84,    "IF-LSB",           NULL, 14000000,             28000000, triggerValueChange); break;
    case IF_USB:  paramAction(action, if_bfo[1],      0x85,    "IF-USB",           NULL, 14000000,             28000000, triggerValueChange); break;
    case IF_CW:   paramAction(action, if_bfo[2],      0x86,     "IF-CW",           NULL, 14000000,             28000000, triggerValueChange); break;
    case IF_AM:   paramAction(action, if_bfo[3],      0x87,     "IF-AM",           NULL, 14000000,             28000000, triggerValueChange); break;

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
  /*
  // Load parameters from EEPROM, reset to factory defaults when stored values are from a different version
  paramAction(LOAD, VERS);
  if((eeprom_version != get_version_id()) || !digitalRead(SW_BAND)){  // EEPROM clean: if key pressed or version signature in EEPROM does NOT corresponds with this firmware
    eeprom_version = get_version_id();
    paramAction(SAVE);  // save default parameter values
    setCursor(0, 1); ucg.print("Reset settings..");
    delay(500); // wdt_reset();
  } else {
    paramAction(LOAD);  // load all parameters
  }
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
    // only process the encoder if not in menu mode. This is because encoder counts could occur during the slow screen painting
    if (readEncoder(6)) {
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
  
  // debug output
  if (Serial.available()) {
    int ch = Serial.read();
    Serial.print("menumode: "); Serial.println(menumode);
    Serial.print("mode: "); Serial.println(mode);
    Serial.print("filt: "); Serial.println(filt);
    Serial.print("stepsize: "); Serial.println(stepsize);
    Serial.print("vfosel: "); Serial.println(vfosel);
    Serial.print("rit: "); Serial.println(rit);
    Serial.print("vfo[VFOA]: "); Serial.println(vfo[VFOA]);
    Serial.print("vfo[VFOB]: "); Serial.println(vfo[VFOB]);
    Serial.print("vfomode[VFOA]: "); Serial.println(vfomode[VFOA]);
    Serial.print("vfomode[VFOB]: "); Serial.println(vfomode[VFOB]);
    Serial.println(); Serial.println();
    
  }
}
  
