///////////////////////////////////////////////////////////////////////////////////
//   G6LBQ Irwell HF Transceiver VFO - Version 1.0
//   stm32 + si5351a VFO With BFO & Conversion Oscilator
//   
//   (I 'JAN2KD 2016.10.19 Multi Band DDS VFO Ver3.1')     
//   Expanded with Multiple SI5351 & I/O Expanders by G6LBQ  
//
//   Created by G6LBQ on 15/09/2020 
///////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////
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


///////////////////////////////////////////////////////////////////////////////////
//  Main Bandpasss Filters For HF Ham Bands - Added by G6LBQ
//
//  1) 0.15 MHz to 1.599999 MHz
//  2) 1.600001 MHz to 1.999999 MHz
//  3) 2.000001 MHz to 2.999999 MHz
//  4) 3.000001 MHz to 3.999999 MHz
//  5) 4.000001 MHz to 5.999999 MHz
//  6) 6.000001 MHz to 7.999999 MHz
//  7) 8.000001 MHz to 10.999999 MHz
//  8) 11.000001 MHz to 14.999999 MHz
//  9) 15.000001 MHz to 21.999999 MHz
//  10)22.000001 MHz to 29.999999 MHz
//
///////////////////////////////////////////////////////////////////////////////////

#define BLUEPILL    // uncomment to tweak some i/o ports for the blue pill, and enable Serial
#define MOCKI2C     // uncomment this to mock transmission to the 2x pcf8574 , Si5351s
#define TRACEI2C    // uncomment to generate debug traces on I2C outputs. BLUEPILL must be defined for Serial to work
#define DEBUG

//---------- Library include ----------

//#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>

#include "si5351a2.h" 
#include "Rotary.h"                     
#include "src/Ucglib.h"
#include "ButtonEvents.h"

//----------   Encoder setting  ---------------

#define ENC_A     PB12                    // Rotary encoder A
#define ENC_B     PB13                    // Rotary encoder B

Rotary r = Rotary(ENC_A,ENC_B);

//----------   TFT setting  ------------------- 

#define   __CS    PB10                    // G6LBQ changed from PB10 to PB3 to free up PB10 for 2nd I2C bus    
#define   __DC    PB0                     // D/C
#define   __RST   PB1                     // RESET   

Ucglib_ILI9341_18x240x320_HWSPI ucg(__DC, __CS, __RST);

//----------  Button Setting -----------------

// define event names for the key events
typedef enum {EVT_NOCHANGE, EVT_STEPUP, EVT_STEPDOWN, EVT_MENU, EVT_RIT, EVT_FREQ_ADJ, EVT_ENTER, EVT_MODE, EVT_BFO_ADJ} keyEvents;

ButtonEvents b = ButtonEvents(EVT_NOCHANGE);

//----------   CW Tone  ------------------- 

#define   CW_TONE     700                 // 700Hz


//----------   I/O Assign  ------------------- 

#define   OUT_LSB      PB15               // Data line for controlling modes of operation                  
#define   OUT_USB      PA8                // Data line for controlling modes of operation                  
#define   OUT_CW       PA9                // G6LBQ added extra mode selection output                  
#define   OUT_AM       PA10               // G6LBQ added extra mode selection output
#define   SW_MENU      PA0
#define   SW_ENTER     PA1                 
#define   SW_MODE      PC14                 
            
#define   SW_RIT       PC15

#ifdef BLUEPILL
#define   LED          PC13
#define   SW_TX        PA3                // need to reassign to move away from led
#else                  
#define   SW_TX        PC13               // G6LBQ> PTT - connect to Gnd for TX
#endif

#define   METER        PA2                // G6LBQ changed from PA1 to PA2    

//---------  Si5351 Assignments ---------------------------
#define   VFO_PORT     0
#define   VFO_CHL      0
#define   BFO_PORT     1
#define   BFO_CHL      2
#define   CONV_PORT    2
#define   CONV_CHL     1

//---------- Modes ---------------------

typedef enum {MODE_LSB, MODE_USB, MODE_CW, MODE_AM, MODE_FM} modes;

//---------- Variable setting ----------

volatile int8_t encoder_val = 0;

// temporary
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

long interruptCount = 0;
long lastInterruptCount = -1;

int_fast32_t interruptsTimeExpired = 0; 




//--------- PCF8574 Interfacing ----------------------------------

void write_PCF8574(uint8_t address, uint8_t data) {
#ifndef MOCKI2C
  Wire.begin();
  Wire.beginTransmission(address);
  Wire.write(0b00000000);
  Wire.endTransmission();
#endif
#ifdef TRACEI2C
  Serial.print("Write to "); Serial.print(address); Serial.print(": "); Serial.println(data);
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

void select_BPF(long f) {

  // HF Band Pass Filter logic added by G6LBQ
  if (freq >=150000 && freq<=1599999){
    select_bank(0);
  } else if(freq >=1600001 && freq<=1999999){
    select_bank(1);
  } else if(freq >=2000001 && freq<=2999999){
    select_bank(2);
  } else if(freq >=3000001 && freq<=3999999){
    select_bank(3);
  } else if(freq >=4000001 && freq<=5999999){
    select_bank(4);
  } else if (freq >=6000001 && freq<=7999999){
    select_bank(5);
  } else if(freq >=8000001 && freq<=10999999){
    select_bank(6);
  } else if(freq >=11000001 && freq<=14999999){
    select_bank(7);
  } else if(freq >=15000001 && freq<=21999999){
    select_bank(8);
  } else if(freq >=22000001 && freq<=29999999){
    select_bank(9);
  } else {
    
  }
}



//---------- PLL write ---------------------------
//
// Original code was for single conversion IF at 11.059MHz so 11.059MHz + VFO Frequency
// 01/07/2022 Changes are for dual conversion so 45MHz firstIF + VFO Frequency 

void PLL_write(){
  if(flg_bfochg == 0){
    if (flagrit==0)
      vfofreq=freq+firstIF;               // G6LBQ 01/07/2022 changed from vfofreq=freq+ifshift; to vfofreq=freq+firstIF;
    else
      vfofreq=freq+firstIF+freqrit;       // G6LBQ 01/07/2022 changed from vfofreq=freq+ifshift+freqrit; to vfofreq=freq+firstIF+freqrit;

    //Vfo_out(vfofreq);                     // VFO output
    //Bfo_out(ifshift);                     // BFO
  }
  else{
    ifshift = freq;
    //Bfo_out(ifshift);                     // BFO
    freq = ifshift;
  }
}

// -----     Routines to interface to the Si5351s-----

long currentFrequency[] = { -1, -1, -1 };   // track output frequences for changes

void _setFrequency(uint8_t port, uint8_t channel, uint32_t frequency) {
#ifndef MOCKI2C
  si5351aSetFrequency(port, channel, frequency);
#endif
#ifdef TRACEI2C
  Serial.print("Si5351 port: "); Serial.print(port); Serial.print(", chl: "); Serial.print(channel); Serial.print(", freq: "); Serial.println(frequency);
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



//---------- Encoder Interrupt -----------------------

void Rotary_enc(){  // investigate very high interupt rate
  static long freq = 0;
  interruptCount++;
  unsigned char result = r.process();
  if (result == DIR_CW) {
    encoder_val++;
  } else if (result == DIR_CCW) {
    encoder_val--;
  }
}

//------------ On Air -----------------------------



void modeset(){

  switch(fmode){
    case MODE_LSB:
      //ifshift = eep_bfo[0];
      setFrequency(CONV_PORT, CONV_CHL,  56059000);
      break;
    case MODE_USB:                                      
      //ifshift = eep_bfo[1];
      setFrequency(CONV_PORT, CONV_CHL,  33941000);
      break;
    case MODE_CW:
      //ifshift = eep_bfo[2];
      setFrequency(CONV_PORT, CONV_CHL,  56059000);
      break;
    case MODE_AM:
      //ifshift = eep_bfo[3];
      setFrequency(CONV_PORT, CONV_CHL,  56059000);
      break;
  }

  digitalWrite(OUT_LSB, fmode==MODE_LSB);
  digitalWrite(OUT_USB, fmode==MODE_USB);
  digitalWrite(OUT_CW,  fmode==MODE_CW);       // G6LBQ added 1/11/20
  digitalWrite(OUT_AM,  fmode==MODE_AM);       // G6LBQ added 1/11/20
}

//------------- Mode set SW ------------

void modesw() {
  fmode = (fmode + 1) % 4;  // wrap around
  modeset();
  PLL_write();
}





//-------------- encoder frequency step set -----------

void setstep(){
  fstep = (fstep + 1) % fstepRatesSize;
  //steplcd(); 
}

void setstepDown() {
  if (fstep==0) fstep = fstepRatesSize;
  fstep--;
  //steplcd();
}

//------------- Step Screen ---------------------------
static const String stepText[] = {"     1Hz", "    10Hz", "   100Hz", "    1KHz", "   10KHz", " 100KHz", "    1MHz"};




//----------  Function EEPROM Initialize  ---------

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

//----------  Function EEPROM Read(4byte)  ---------

long Fnc_eepRD(uint16 adr){
  long val = 0;
  uint16 dat,dummy;  

  dummy = EEPROM.read(adr,&dat);
  val = dat << 16;
  dummy = EEPROM.read(adr+1,&dat);
  return val | dat;
}

//----------  Function EEPROM Write(4byte)  ---------

void Fnc_eepWT(long dat,uint16 adr){
  uint16 dummy,val;

  val = dat & 0xffff;
  dummy = EEPROM.write(adr+1,val);
  val = dat >> 16;
  val = val & 0xffff;
  dummy = EEPROM.write(adr,val);
}

//------------- temp vars, routines --------------------------------

// temp vars
int eeprom_addr;
#define EEPROM_OFFSET 0x0
uint8_t eeprom_version;

// state variables
enum menu_t { NO_MENU, MENU_SELECTED, MENU_VALUE };
volatile uint8_t menumode = 0;  // 0=not in menu, 1=selects menu item, 2=selects parameter value //todo - use enums
volatile uint8_t prev_menumode = 0;
volatile int8_t menu = 0;  // current parameter id selected in menu
volatile bool change = true;
volatile uint8_t mode = MODE_USB;  //todo need volatile?
volatile uint8_t filt = 0;
uint8_t bandval = 3;
volatile uint8_t stepsize = 3;  //todo revisit - uSDX uses an enum
volatile uint32_t xtalfreq = 25000000;
unsigned long if_bfo[]= {11056570, 11059840, 11058400, 11058200};
enum vfo_t { VFOA=0, VFOB=1, SPLIT=2 };
volatile uint8_t vfosel = VFOA;
volatile int16_t rit = 0;
int32_t vfo[] = { 7074000, 14074000 };
uint8_t vfomode[] = { MODE_USB, MODE_USB };

void lcdnoCursor() {};  // really is nothing to do here. Unless the display does have a cursor?

#define get_version_id() 1

void eeprom_read_block (void *__dst, const void *__src, size_t __n) {
  Serial.print("eeprom_read_block: from "); Serial.print((int) __src); Serial.print(", length: "); Serial.println(__n);
};

void eeprom_write_block(const void *__src, void *__dst, size_t __n) {
  Serial.print("eeprom_write_block: to "); Serial.print((int) __dst); Serial.print(", length: "); Serial.println(__n);
};

//------------- menu system --------------------------------


enum action_t { UPDATE, UPDATE_MENU, NEXT_MENU, LOAD, SAVE, SKIP, NEXT_CH };
enum params_t {NULL_, MODE, FILTER, BAND, STEP, VFOSEL, RIT, SIFXTAL, IF_LSB, IF_USB, IF_CW, IF_AM, FREQA, FREQB, MODEA, MODEB, VERS, ALL=0xff};
#define N_PARAMS 7  // number of (visible) parameters
#define N_ALL_PARAMS (N_PARAMS+5)  // number of parameters

#define N_BANDS 11

const char* mode_label[5] = { "LSB", "USB", "CW ", "AM ", "FM " };
const char* filt_label[8] = { "Full", "7", "6", "5", "4", "3", "2", "1" };  // todo - use numeric input instead?
const char* band_label[N_BANDS] = { "160m", "80m", "60m", "40m", "30m", "20m", "17m", "15m", "12m", "10m", "6m" };
const char* stepsize_label[] = { "1Hz", "10Hz", "100Hz", "1KHz", "10KHz", "100KHz", "1MHz" };
const char* vfosel_label[] = { "A", "B"/*, "Split"*/ };
const char* offon_label[2] = {"OFF", "ON"};

#define _N(a) sizeof(a)/sizeof(a[0])

void lcd_blanks(){ 
  ucg.print("        ");
}

void show_banner(){
  setCursor(0, 0);
  ucg.print(F("Irwell TXCVR V0.1"));
  //ucg.print('\x01'); 
  lcd_blanks(); lcd_blanks();
  setCursor(0, 1);
  lcd_blanks(); lcd_blanks();
}

void setCursor(int x, int y) {
  ucg.setPrintPos(x*22, y*22+22);
}

// output menuid in x.y format
void printmenuid(uint8_t menuid){
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
    ucg.print(label); lcd_blanks(); lcd_blanks();
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
    case SKIP:
      break;
  }
  eeprom_addr += size;
}

template<typename T> void paramAction(uint8_t action, volatile T& value, uint8_t menuid, const char* label, const char* enumArray[], int32_t _min, int32_t _max, void (*trigger)(int m)){
  switch(action){
    case UPDATE:
    case UPDATE_MENU:
      value = (int32_t)value + encoder_val;
      if(     value < _min) value = _min;
      else if(value > _max) value = _max;
      trigger(menuid);     
      encoder_val = 0;

      lcdnoCursor();
      printlabel(action, menuid, label);  // print normal/menu label
      if(enumArray == NULL){  // print value
        if((_min < 0) && (value >= 0)) ucg.print('+');  // add + sign for positive values, in case negative values are supported
        ucg.print(value);
      } else {
        ucg.print(enumArray[value]);
      }
      lcd_blanks(); lcd_blanks();
      break;
      
    default:
      actionCommon(action, (uint8_t *)&value, sizeof(value));
      break;
  }
}

void triggerValueChange(int menu) {
  Serial.print("Trigger on "); Serial.println(menu);
}

int8_t paramAction(uint8_t action, uint8_t id = ALL) { // list of parameters
  if((action == SAVE) || (action == LOAD)){
    eeprom_addr = EEPROM_OFFSET;
    for(uint8_t _id = 1; _id < id; _id++) 
      paramAction(SKIP, _id);
  }
 
  switch(id){    
    case ALL:     for(id = 1; id != N_ALL_PARAMS+1; id++) paramAction(action, id);  // for all parameters
    // Visible parameters
    case MODE:    paramAction(action, mode,     0x11, "Mode",      mode_label,     0, _N(mode_label) - 1,     triggerValueChange); break;
    case FILTER:  paramAction(action, filt,     0x13, "NR Filter", filt_label,     0, _N(filt_label) - 1,     triggerValueChange); break;
    case BAND:    paramAction(action, bandval,  0x14, "Band",      band_label,     0, _N(band_label) - 1,     triggerValueChange); break;
    case STEP:    paramAction(action, stepsize, 0x15, "Tune Rate", stepsize_label, 0, _N(stepsize_label) - 1, triggerValueChange); break;
    case VFOSEL:  paramAction(action, vfosel,   0x16, "VFO Mode",  vfosel_label,   0, _N(vfosel_label) - 1,   triggerValueChange); break;
    case RIT:     paramAction(action, rit,      0x17, "RIT",       offon_label,    0, 1,                      triggerValueChange); break;
    case SIFXTAL: paramAction(action, xtalfreq, 0x83, "Ref freq",  NULL,    14000000, 28000000,               triggerValueChange); break;
    case IF_LSB:  paramAction(action, if_bfo[0],0x84, "IF-LSB",    NULL,    14000000, 28000000,               triggerValueChange); break;
    case IF_USB:  paramAction(action, if_bfo[1],0x85, "IF-USB",    NULL,    14000000, 28000000,               triggerValueChange); break;
    case IF_CW:   paramAction(action, if_bfo[2],0x86, "IF-CW",     NULL,    14000000, 28000000,               triggerValueChange); break;
    case IF_AM:   paramAction(action, if_bfo[3],0x87, "IF-AM",     NULL,    14000000, 28000000,               triggerValueChange); break;

    // invisible parameters
    case FREQA:   paramAction(action, vfo[VFOA],      0, NULL,     NULL,           0,        0,               triggerValueChange); break;
    case FREQB:   paramAction(action, vfo[VFOB],      0, NULL,     NULL,           0,        0,               triggerValueChange); break;
    case MODEA:   paramAction(action, vfomode[VFOA],  0, NULL,     NULL,           0,        0,               triggerValueChange); break;
    case MODEB:   paramAction(action, vfomode[VFOB],  0, NULL,     NULL,           0,        0,               triggerValueChange); break;
    case VERS:    paramAction(action, eeprom_version, 0, NULL,     NULL,           0,        0,               triggerValueChange); break;
    
    case NULL_:   menumode = NO_MENU; show_banner(); change = true; break;
    default:      if((action == NEXT_MENU) && (id != N_PARAMS)) 
                      id = paramAction(action, max(1 /*0*/, min(N_PARAMS, id + ((encoder_val > 0) ? 1 : -1))) ); break;  // keep iterating util menu item found
  }
  return id;  // needed?
}

void processMenuKey() {
    if     (menumode == 0){ menumode = 1; if(menu == 0) menu = 1; }  // enter menu mode
    else if(menumode == 1){ menumode = 2; }                          // enter value selection screen
    else if(menumode >= 2){ paramAction(SAVE, menu); menumode = 0;}  // save value, and return to default screen
}

void processEnterKey() {
  if     (menumode == 1){ menumode = 0; }  
  else if(menumode >= 2){ menumode = 1; change = true; paramAction(SAVE, menu); } // save value, and return to menu mode

}

bool menuActive() {
  return menumode > 0;
}

void processMenu() {
  
  if((menumode) || (prev_menumode != menumode)){  // Show parameter and value
    int8_t encoder_change = encoder_val;
    if((menumode == 1) && encoder_change){
      menu += encoder_val;   // Navigate through menu
      menu = max(1 , min(menu, N_PARAMS));
      menu = paramAction(NEXT_MENU, menu);  // auto probe next menu item (gaps may exist)
      encoder_val = 0;
    }
    if(encoder_change || (prev_menumode != menumode))
      paramAction(UPDATE_MENU, (menumode) ? menu : 0);  // update param with encoder change and display
    
    prev_menumode = menumode;
  }  
}

//------------------  Initialization  Program  -------------------------
 
void setup() {
#ifdef BLUEPILL
  while (!Serial)
    ;
  Serial.println("Starting setup");
  pinMode(LED, OUTPUT);
#endif
  
  pinMode( ENC_A, INPUT_PULLUP);                  // PC13 pull up
  pinMode( ENC_B, INPUT_PULLUP);                  // PC14
  attachInterrupt( ENC_A, Rotary_enc, CHANGE);    // Encoder A
  attachInterrupt( ENC_B, Rotary_enc, CHANGE);    //         B

  delay(100);
  
  ucg.begin(UCG_FONT_MODE_SOLID);
  ucg.clearScreen();
  ucg.setRotate270();
  ucg.setColor(1, 0, 0, 0);       // set background color
  ucg.setColor(255,255,255);
  ucg.setFont(ucg_font_inb16_mr);   // set an arbitrary font for the moment


  b.add(SW_MENU, EVT_MENU, EVT_NOCHANGE);
  b.add(SW_ENTER, EVT_ENTER, EVT_NOCHANGE);
  b.add(SW_MODE, EVT_MODE, EVT_NOCHANGE);
  b.add(SW_RIT, EVT_RIT, EVT_NOCHANGE);
  
  pinMode(SW_TX,INPUT_PULLUP);
  pinMode(OUT_LSB,OUTPUT);                    // LSB Mode 
  pinMode(OUT_USB,OUTPUT);                    // USB Mode
  pinMode(OUT_CW,OUTPUT);                     // CW Mode - G6LBQ added additional mode selection
  pinMode(OUT_AM,OUTPUT);                     // AM Mode - G6LBQ added additional mode selection
  
  init_PCF8574();
/* test display
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
    setCursor(0, 1); ucg.print(F("Reset settings.."));
    delay(500); // wdt_reset();
  } else {
    paramAction(LOAD);  // load all parameters
  }
  */
  show_banner();
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
     case EVT_MENU:
      processMenuKey();
      break;
    case EVT_ENTER:
      processEnterKey();
      break;
    case EVT_STEPUP:
      //setstep();
      break;
    case EVT_STEPDOWN:
      //setstepDown();
      break;
    case EVT_RIT:
      //setrit();
      break;
    case EVT_NOCHANGE:
      break; // nothing to do
  }

  processMenu();

  

  // debug output
  if (Serial.available()) {
    int ch = Serial.read();
    Serial.print("mode: "); Serial.println(mode);
    Serial.print("filt: "); Serial.println(filt);
    Serial.print("stepsize: "); Serial.println(stepsize);
    Serial.print("vfosel: "); Serial.println(vfosel);
    Serial.print("rit: "); Serial.println(rit);
    Serial.print("vfo[VFOA]: "); Serial.println(vfo[VFOA]);
    Serial.print("vfo[VFOB]: "); Serial.println(vfo[VFOB]);
    Serial.print("vfomode[VFOA]: "); Serial.println(vfomode[VFOA]);
    Serial.print("vfomode[VFOB]: "); Serial.println(vfomode[VFOB]);
    Serial.println();
    
  }


}
  
