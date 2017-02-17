/*
 * AD9102.cpp
 * Library for the Analog Devices
 * Waveform Generation IC AD9102
 * for the Arduino Platform
 * Created by Evangelos Petrongonas,
 * Email: vpetrog@hotmail.com
 * Athens 2017
 * Part of the ArduGen Project
 * Licensed under: MIT
 * Copyright (c) 2017 Evangelos Petrongonas
 */

#include <AD9102.h>

//Constructor
AD9102::AD9102(byte chip_select){
  CS=chip_select;
}
//Function Implementations

//Register Write function
void AD9102::register_write(uint16_t register_value,uint16_t register_Address, byte CS) { //write to register_Address the register_Value
  digitalWrite(CS,LOW); //to be changed
  SPI.transfer16(register_Address); //RW bit +Register Address
  SPI.transfer16(register_value);   //Register Value
  digitalWrite(CS,HIGH);
}

//Register Read function
void AD9102::register_read(uint16_t register_Address,byte CS,uint16_t* Value) {
uint16_t register_Address_calculation= register_Address+0x8000; // RW->1 Read,RW->0 Write
  digitalWrite(CS,LOW); //to be changed
  SPI.transfer16(register_Address_calculation);
  *Value=SPI.transfer16(0xFFFF);   //dummy 16 bit word, the actual value of the regster wil be stored in *value
  digitalWrite(CS,HIGH);
}

//each of the followings function is used to determine the value to be written to each one of the 32 registers of the AD9102.
//Register 0x0060 CFG_ERROR, cannot be written, so no value fuction is availabe.

//Register 0x00 SPICONFIG
uint16_t AD9102::spiconfig(byte RESET) { //SPI congig register 0x00
   uint16_t spiconfig_value=0x0000;
   byte LSB_FIRST=0; //BIT 15:LSB first selection. 0->MSB (default), 1->LSB
   byte SPI3WIRE=0;  //BIT 14: SPI 3/4 wire config. 0-> 4 wire SPI (default), 1->3wire SPI deffault
                     //BIT 13: 0->DO NOTHING, 1->Reset all registers to default value
   byte DOUBLESPI=0; //BIT 12: Double drive data line. 0->1 data line, 1->2 data lines.
   byte SPI_DRV=0;   //BIT 11: Double Drive out. 0-> Single. 1-> Double.
   byte DOUT_EN=1;   //BIT 10: DOUT Signal enable. 0-> SDO/SDI2. 1->DOUT
                     //BIT 9-6: RESERVED
                     //BIT 5-0: Mirror of BITS [15-10]
   //Create Register Value
   bitWrite(spiconfig_value,0,LSB_FIRST);
   bitWrite(spiconfig_value,1,SPI3WIRE);
   bitWrite(spiconfig_value,2,RESET);
   bitWrite(spiconfig_value,3,DOUBLESPI);
   bitWrite(spiconfig_value,4,SPI_DRV);
   bitWrite(spiconfig_value,5,DOUT_EN);
   bitWrite(spiconfig_value,10,DOUT_EN);
   bitWrite(spiconfig_value,11,SPI_DRV);
   bitWrite(spiconfig_value,12,DOUBLESPI);
   bitWrite(spiconfig_value,13,RESET);
   bitWrite(spiconfig_value,14,SPI3WIRE);
   bitWrite(spiconfig_value,15,LSB_FIRST);

  return spiconfig_value;
}

//Register 0x01 POWRCONFIG
uint16_t AD9102::powerconfig() {
  uint16_t powerconfig_value=0x0000;
                                     //BIT 15-12: Reserved
                                     //BIT 11: CLKVDD LDO status
                                     //BIT 10: DVDD1 LDO status
                                     //BIT 9:  DVDD2 LDO status
  byte PDN_LDO_CLK=0;  //BIT 8: Disable CLVDD LDO
  byte PDN_LDO_DIG1=0; //BIT 7: Disable DVDD1 LDO
  byte PDN_LDO_DIG2=0; //BIT 6: Disable DVD22 LDO
  byte REF_PDN=1;      //BIT 5: Power Down on Chip REFIO
  byte REF_EXT=0;      //BIT 4: Must be set to 0
  byte DAC_SLEEP=0;    //BIT 3: Disable Dac Output
                                     //BIT 2-0: Reserved
  //Create Register Value
  bitWrite(powerconfig_value,3,DAC_SLEEP);
  bitWrite(powerconfig_value,4,REF_EXT);
  bitWrite(powerconfig_value,5,REF_PDN);
  bitWrite(powerconfig_value,6,PDN_LDO_DIG2);
  bitWrite(powerconfig_value,7,PDN_LDO_DIG1);
  bitWrite(powerconfig_value,8,PDN_LDO_CLK);

 return powerconfig_value;
}

//Register 0x02 CLOCKCONFIG
uint16_t AD9102::clockconfig() {
  uint16_t clockconfig_value=0x0000;
                                    //BIT 15-12: Reserved
  byte DIS_CLK=0;     //BIT 11: Disable Analog clock to DAC_SLEEP
                                    //BIT 10-8: Reserved
  byte DIS_DCLK=0;    //BIT 7: Disable clock to digital block
  byte CLK_SLEEP=0;   //BIT 6: Very Low Power clock Mode
  byte CLK_PDN=0;     //BIT 5: Disables Main Clock Receiver
  byte EPS=0;         //BIT 4: Enable Power Save
  byte DAC_INV_CLK=0; //BIT 3: Inverts Inside Clock 180 phase shift DAC update Timing
                                    //BIT 2-0: Reserved
  //Create Register Value
  bitWrite(clockconfig_value,3,DAC_INV_CLK);
  bitWrite(clockconfig_value,4,EPS);
  bitWrite(clockconfig_value,5,CLK_PDN);
  bitWrite(clockconfig_value,6,CLK_SLEEP);
  bitWrite(clockconfig_value,7,DIS_DCLK);
  bitWrite(clockconfig_value,11,DIS_CLK);

  return clockconfig_value;
}

//Register 0x0003 REFADJ
uint16_t AD9102::refadj(uint16_t BGDR_value) { //for the ArduGen Project this is not Important
  uint16_t refadj_value=0x0000;
                                      //BIT 15-6: Reserved
                                      //BIT 5-0 On Chip Reference Adjustment
  //create register value
  return refadj_value+=BGDR_value;
}

//Register 0x0007 DACAGAIN
uint16_t AD9102::dacagain(uint16_t DAC_AGain) { //DAC_Again is in binary.
  uint16_t dacagain_value=0;
                        //BIT 15: Reserved
                        //BIT 14:8 DAC Analog Gain Calibration Output
                        //BIT 7: Resserved
                        //BIT 6-0: DAC Analog Gain Control, 2's complement
  //Register Value
  return dacagain_value+=DAC_AGain;
}

uint16_t AD9102::dacrange(byte DAC_GAIN_RNG) { //DAC_GAIN_RNG is in binary
  uint16_t dacrange_value=0x0000;
                        //BIT: 15-2 Reserved
                        //BIT:2-0 DAC gain range controll
  //Register Value
  return dacrange_value+=DAC_GAIN_RNG;
}

uint16_t AD9102::dacrset(byte RW, byte DAC_RSET) { //RW RW=0->READ, RW=1-> WRITE
  uint16_t dacrset_value=0x0000;
                        //BIT 15: DAC rset Enable. 0-> Disable (calibration), 1-> Enable (post calibration)
                        //BIT 14-13: Reserved
                        //BIT 12:8: DAC_RSET_CAL Read digital control for the value of Rset after calibration
                        //BIT 7-5: Reserved
                        //BIT 4:0: DAC_RSET digital control sto set the value of th rset resistor
  //Register Value
  return dacrset_value+=0x8000*RW+DAC_RSET*RW; //DURING Read, DAC_RSET is null, bit 15 is equal to 0x8000
}

//Register 0x000D CALCONFIG
uint16_t AD9102::calconfig(byte CAL_RESET, byte CAL_MODE_EN,byte COMP_CAL_RNG, byte CAL_CLK_EN,byte CAL_CLK_DIV) {
  uint16_t calconfig_value;
                        //BIT 15: Reserved
                        //BIT 14: Compensation offset overflow
                        //BIT 13: Compensation offset underflow
                        //BIT 12: Rset calibration overflow
                        //BIT 11: Rset Calibration underflow
                        //BIT 10: Gain Calibration overflow
                        //BIT 9:  Gain Calibration underflow
                        //BIT 8:  Calibration Reset
                        //BIT 7:  Calibration is being Used
                        //BIT 6:  Calibration Enable
                        //BIT 5-4: Offset Callibration range
                        //BIT 3:  Calibration Clock Enable
                        //BIT 2-0: Calibration Clock Divider
  //Register Value
  calconfig_value= 0x0000;
  bitWrite(calconfig_value,8,CAL_RESET);
  bitWrite(calconfig_value,6,CAL_MODE_EN);
  byte rng=COMP_CAL_RNG;
  rng<<=4; //shift COMP_CAL_RNG to the designated bit position
  bitWrite(calconfig_value,3,CAL_CLK_EN);

  return calconfig_value+=rng+CAL_CLK_DIV;
}

//Overload cal_config for writing only offset calibration range
void AD9102::calconfig(byte cal_range,byte CS) {
  uint16_t *register_value=0;
  byte bit0,bit1;
  bit1=bitRead(cal_range,1);
  bit0=bitRead(cal_range,0);
  register_read(calconfig_Address,CS,register_value);
  bitWrite(*register_value,5,bit1);
  bitWrite(*register_value,4,bit0);
  register_write(*register_value,calconfig_Address,CS);
}

//Register 0x000E COMPOFFSET
uint16_t AD9102::compoffset(byte START_CAL) {
  uint16_t compoffset_value=0x0000;
                          //BIT 15:  Reserved
                          //BIT 14:- 8 Result of the Offset calibration for CMP
                          //BIT 7-2: Reserved
                          //BIT 1:   Calibration is Finished
                          //BIT 0:   Start a Callibration cycle
  //Register Value
  return compoffset_value+=START_CAL;
}

//Register RAMUPDATE
uint16_t AD9102::ramupdate(byte UPDATE) {
                          //BIT 15-1: Reserved
                          //BIT 0: update all spi settings with a new config self-cleaning
  //Register value
  short unsigned ramupdate_value=0x0000;
  return ramupdate_value+=UPDATE;
}

//Register 0x001E
uint16_t AD9102::pat_status(byte MEM_ACCESS,byte RUN) {
  uint16_t pat_status_value=0x0000;
                          //BIT 15-3: Reserved
  byte BUF_READ=B0000;    //BIT 3: Read Back from updated Buffer
                          //BIT 2: Enable Spi Memmory acces
                          //BIT 1: Run
  //Register Value
  byte mem=MEM_ACCESS;
  mem<<=2; //shift MEM_ACCESS to the designated bit position
  return pat_status_value+=BUF_READ+mem+RUN;
}

//Register 0x001F PAT_TYPE
uint16_t AD9102::pat_type(byte PATTERN_RPT) {
                        //BIT 15-1: RESERVED
                        //BIT 0: pattern repeat. 0-> continiously. 1->Repeats for times defined in 0X002A-B
  //Register Value
  uint16_t pat_type_value=0x0000;
  return pat_type_value+=PATTERN_RPT;
}

//Register 0x0020 PATTERN_DLY
uint16_t AD9102::pattern_dly(uint16_t PATTERN_DELAY) {
                        //BIT 15-0: time between when the TRIGGER PIN IS LOW and the pattern starts in number of DAC cycle +1
  //Register Value
  uint16_t pattern_dly=PATTERN_DELAY;
  return pattern_dly;
}

//Register 0x0025 DACDOF
uint16_t AD9102::dacdof(uint16_t DAC_DIG_OFFSET) {
                      //BIT 15-4: DAC_DIG_OFFSET
                      //BIT 3-0:  Reserved
  //Register Value
  uint16_t dacdof_value=0x0000;
  uint16_t doff=DAC_DIG_OFFSET;
  doff<<=4;
  return dacdof_value+=doff;
}

//Register 0x0027 WAV_CONFIG
uint16_t AD9102::wav_config(byte PRESTORE_SEL, byte WAVE_SEL) {
                      //BIT 15-6: Reserved
                      //BIT 5-4: Prestored Waveform selection. 00->DC, 01->Sawtooth, 10-> Noise, 11-> DDS
                      //BIT 3: Reserved
  byte CH_ADD=0;      //BIT 2: CH_ADD normal operation for DAC
                      //BIT 1-0: Waveform selection. 00-> RAM, 01->Prestored, 10->prestored with start delay and pattern period, 11-> presoterd with SRAM Modulation
  //Caution Reserved BITS 9-8 must be set to 0x0001 instead of null

  //Register value
  uint16_t wav_config_value=0x0300; //equals to 0000001100000000
  byte prestore=PRESTORE_SEL;
  prestore<<=5;
  bitWrite(wav_config_value,2,CH_ADD);
  return wav_config_value+=prestore+WAVE_SEL;
}

//Register 0x28 PAT_TIMEBASE
uint16_t AD9102::pat_timebase(byte HOLD,byte PAT_PERIOD_BASE,byte START_DELAY_BASE) {
                      //BIT 15-12: Reserved
                      //BIT 11-8: Number of time the DAC holds the sample 0-> 1 sample
                      //BIT 7-4:  Number of DAC clock periods per patern period LSB 0-> 1 DAC clock
                      //BIT 3-0:  Number of DAC Periods
  //Register Value
  short pat_timebase_value= 0x0000;
  uint16_t hd=HOLD;
  hd<<=8;
  byte period=PAT_PERIOD_BASE;
  period<<=4;

  return pat_timebase_value+=hd+period+START_DELAY_BASE;
}

//Register 0x0029 PAT_PERIOD
uint16_t AD9102::pat_period(uint16_t PATTERN_PERIOD) {
                      //BIT 15-0: Pattern period= pat_timebase*pat_period
  //Register value
  uint16_t pat_period_value=0x8000;
  return pat_period_value+=(PATTERN_PERIOD*(PATTERN_PERIOD !=0)-((PATTERN_PERIOD!=0)*0x8000)); //if PATTERN_PERIOD <>0 then load the value, else load 0x8000
}

//Register 0x002B DAC_PAT
uint16_t AD9102::dac_pat(byte DAC_REPEAT_CYCLE) {
                      //BIT 15-8: Reserved
                      //BIT 7-0: number of DAC pattern repeat cycles
  //Register Value
  uint16_t dac_pat_value=0xFFFF; //deafault  value is 0
  return dac_pat_value+= -((DAC_REPEAT_CYCLE!=0)*0x00FF)+DAC_REPEAT_CYCLE*(DAC_REPEAT_CYCLE!=0); //BITS 15-8 stay intact while the rest ewual to the DAC_REPEAT_CYCLE
}

//Register 0x002C DOUT_START
uint16_t AD9102::dout_start(uint16_t DOUT_STARTV) { //Not Used
                      //BIT 15-0: time between when the trigger is low and Dout is high in DAC clock cycles
  //Register Value
  uint16_t dout_start_value=0x0003;
  return dout_start_value+= -((DOUT_STARTV!=0)*0x0003)+DOUT_STARTV*(DOUT_STARTV!=0);
}

//Register 0x002D DOUT_CONFIG
uint16_t AD9102::dout_config(byte DOUT_STOP) { //Not Used
                      //BIT 15-6: Reserved
  byte DOUT_VAL=0;    //BIT 5: Manually set Dout Signal Value (DOUT_MODE must be 0)
  byte DOUT_MODE=0;   //BIT 4: DOUT enable signal differnt modes.
                      //BIT 3-0: DOUT_STOP time between pattern end and Dout signal is low
  //Register Value
  uint16_t dout_config_value=0x0000;
  bitWrite(dout_config_value,5,DOUT_VAL);
  bitWrite(dout_config_value,4,DOUT_MODE);
  return dout_config_value+= DOUT_STOP;
}

//Regsiter 0x0031 DAC_CST
uint16_t AD9102::dac_cst(uint16_t DAC_CONST) {
                      //BIT 15-4: MSB DAC constant value
                      //BIT 3-0:  Reserved
  //Register Value
  uint16_t dac_cst_value=0x0000;
  uint16_t a=DAC_CONST;
  a<<=4;
  return dac_cst_value+=a;
}

//Register 0x0035 DAC_DGAIN
uint16_t  AD9102::dac_dgain(uint16_t DAC_DIG_GAIN) {
                      //BIT 15-4: DAC Digital gain form -2-2 in steps of 0.001
                      //BIT 3-0:  Reserved
  //Register Value
  uint16_t dac_dgain_value=0x0000;
  uint16_t temp=DAC_DIG_GAIN;
  temp<<=4;
  return dac_dgain_value+=temp;
}

//Register 0x0037 SAW_CONFIG
uint16_t AD9102::saw_config(byte SAW_STEP, byte SAW_TYPE) {
                      //BIT 15-8: Reserved
                      //BIT 7-2: Saw_Step number of samples per step for the DAC
                      //BIT 1-0: saw_type 0->Ramp up,1->Ramp down,2->Triangle,3 No wave
  //Register Value
  uint16_t saw_config_value=0xFFFC;
  byte temp=SAW_STEP;
  temp<<=2;
  return saw_config_value+= temp-(temp!=0)*0x00FC+SAW_TYPE;
}

//Resgister 0x003E DDS_TW32
uint16_t AD9102::dds_tw32(uint16_t DDSTW_MSB) {
                      //BIT 15-0: DDS TW MSB
  //Register Value
  uint16_t dds_tw32_value=0x0000;
  return dds_tw32_value+=DDSTW_MSB;
}

//Register 0x003F DDS_TW1
uint16_t AD9102::dds_tw1(byte DDSTW_LSB) {
                      //BIT 15-8: DDS TW LSB
                      //BIT 7-0: Reserved
  //Register Value
  uint16_t dds_tw1_value=0x0000;
  uint16_t temp=DDSTW_LSB;
  temp<<=8;
  return dds_tw1_value+=temp;
}

//Register 0x0043 DDS_PW
uint16_t AD9102::dds_pw(uint16_t DDS_PHASE) {
                      //BIT 15-0: DDS phase offset
  //Register Value
  uint16_t dds_pw_value=0x0000;
  return dds_pw_value+= DDS_PHASE;
}

//Register 0x0044 TRIG_TW_SEL
uint16_t AD9102::trig_tw_sel(byte TRIG_DELAY_EN) {
                      //BIT 15-2: Reserved
                      //BIT 1 Enable Start Delay for all patterns (0) or only in the beginning (1)
                      //BIT 0 Reserved
  //Register Value
  uint16_t trig_tw_sel_value=0x0000;
  bitWrite(trig_tw_sel_value,1,TRIG_DELAY_EN);
  return trig_tw_sel_value;
}

//Register 0x0045 DDS_CONFIG
uint16_t AD9102::dds_config(byte DDS_COS_EN,byte PHASE_MEM_EN, byte TW_MEM_EN){
                      //BIT 15-4: Reserved
                      //BIT 3: DDS_COS enable cos instead of sine
  byte DDS_MSB_EN=0;  //BIT 2: DDS_MSB. 0->selects SRam address counter clock as CLK.1-> DDS MSB
                      //BIT 1: PHASE_MEM_EN. 0-> DDS_PW as source of DDS offset. 1-> SRAM as source
                      //BIT 0: TW_MEM_EN.
  //Register Value
  uint16_t dds_config_value=0x0000;
  bitWrite(dds_config_value,0,TW_MEM_EN);
  bitWrite(dds_config_value,1,PHASE_MEM_EN);
  bitWrite(dds_config_value,2,DDS_MSB_EN);
  bitWrite(dds_config_value,3,DDS_COS_EN);
  return dds_config_value;
}

//Register 0x0047 TW_RAM_CONFIG
uint16_t AD9102::tw_ram_config(byte TW_MEM_SHIFT) {
                      //BIT 15-5: Reserved
                      //BIT 4-0: TW_MEM_SHIFT. The register controls the right shift when memory data merge to DDS1TW
  //Register Value
  uint16_t tw_ram_config_value=0x0000;
  return tw_ram_config_value+= TW_MEM_SHIFT;
}

//Register 0x005C START_DLY
uint16_t AD9102::start_dly(uint16_t START_DELAY) {
                      //BIT 15-0: Start Delau of DAC
  //Register Value
  uint16_t start_dly_value=0x0000;
  return start_dly_value+=START_DELAY;
}

//Register 0x005D start_addr_Address
uint16_t AD9102::start_addr(uint16_t START_ADDR){
                      //BIT 15-4: START_ADDR
                      //BIT 3-0: Reserved
  //Register Value
  uint16_t start_addr_value=0x0000;
  uint16_t temp=START_ADDR;
  temp<<=4;
  return start_addr_value+=temp;
}

//Register 0x005E STOP_ADDR
uint16_t AD9102::stop_addr(uint16_t STOP_ADDR){
                      //BIT 15-4: STOP_ADDR
                      //BIT 3-0: Reserved
  //Register Value
  uint16_t stop_addr_value=0x0000;
  uint16_t temp=STOP_ADDR;
  temp<<=4;
  return stop_addr_value+=temp;
}

//Register 0x005F DDS_CYC
uint16_t AD9102::dds_cyc(uint16_t DDS_CYCLE) {
                      //BIT 15-0: Number of wine wave cycles when a DDS prestored waveform with start_addr
                      //and stop delays is selected for the DAC output
  //Register Value
  uint16_t dds_cyc_value=0xFFFF;
  return dds_cyc_value=dds_cyc_value*(DDS_CYCLE==0)+DDS_CYCLE;
}

//The following functions are used to interprent the value of the registers

//Power Config Register Signal Print
void AD9102::power_config_read(uint16_t register_value) {
  byte bit11=bitRead(register_value,11);
  byte bit10=bitRead(register_value,10);
  byte bit9=bitRead(register_value,9);
  (bit11) ? (Serial.println(F("CLVDD LDO is ON"))) : (Serial.print(F("CLVDD LDO is OFF"))); //CLKVDD LDO STATUS.
  (bit10) ? (Serial.println(F("DVDD1 LDO is ON"))) : (Serial.println(F("DVDD1 LDO is OFF"))); //DVDD1 LDO STATUS.
  (bit9) ? (Serial.println(F("DVDD2 LDO is ON"))) : (Serial.println(F("DVDD2 LDO is OFF")));  //DVDD2 LDO STATUS.
}

//CAL CONFIG Register Signal print
void AD9102::cal_config_read(uint16_t register_value) {
  byte bit14=bitRead(register_value,14);
  byte bit13=bitRead(register_value,13);
  byte bit12=bitRead(register_value,12);
  byte bit11=bitRead(register_value,11);
  byte bit10=bitRead(register_value,10);
  byte bit9=bitRead(register_value,9);
  byte bit7=bitRead(register_value,7);
  if(bit14) (Serial.println(F("Error 010: Compensation Offset Calibration Overflow"))); //Compensation Cal Overflow
  if(bit13) (Serial.println(F("Error 011: Compensation Offset Calibration Underflow"))); //Compensation Cal underflow
  if(bit14==0 && bit13==0) Serial.println(F("CompensationOffset Calibration Success")); //Compensation Cal Success
  if(bit12) (Serial.println(F("Error 020:Rset value Calibration Overflow"))); //Rset Calibration Overflow
  if(bit11) (Serial.println(F("Error 021: Rset value Calibration Underflow"))); //Rset Calibration Underflow
  if(bit12==0 && bit11==0) (Serial.println(F("Rset value Calibration Success"))); //Rset Calibration Success
  if(bit10) (Serial.println(F("Error 030: Gain Calibration Overflow"))); //Gain Callibration overflow
  if(bit9) (Serial.println(F("Error 031: Gain Calibration Underflow"))); //Gain Callibration underflow
  if(bit10==0 && bit9==0) (Serial.println(F("Gain Calibration Success"))); //Gain Calibration Success
  (bit7) ? (Serial.println(F("Callibration is in Progress"))) : (Serial.println(F("Callibration not in Use")));  //Callibration Status.
  if(bit10==0 && bit9==0 && bit12==0 && bit11==0 && bit14==0 && bit13==0) (Serial.println(F("Auto Calibration finished succesfully")));
}

//CFG_ERROR REgister signal print
void AD9102::cfg_error_read(uint16_t register_value) {
  byte bit5=bitRead(register_value,5);
  byte bit4=bitRead(register_value,4);
  byte bit3=bitRead(register_value,3);
  byte bit2=bitRead(register_value,2);
  byte bit1=bitRead(register_value,1);
  byte bit0=bitRead(register_value,0);
  if(bit5) (Serial.println(F("Error 101: DOUT_Start falue is larger than the patern delay")));
  if(bit4) (Serial.println(F("Error 102: Pattern Delay value is smaller than the default value")));
  if(bit3) (Serial.println(F("Error 103: DOUT_START value is smaller than the default value")));
  if(bit2) (Serial.println(F("Error 104: Period Setting Register is smaller than the pattern play cycle")));
  if(bit1) (Serial.println(F("Error 105: Memmory Pattern Play is not of even length in trigger delay mode")));
  if(bit0) (Serial.println(F("Error 105: Memmory Read Conflict")));
}

//overload cfg_error_read function to use for a specific bit
void AD9102::cfg_error_read(uint16_t register_value,byte bit) {
  byte bit_value=bitRead(register_value,bit);
  if (bit_value) {
    switch(bit) {
      case 5: (Serial.println(F("Error 101: DOUT_Start falue is larger than the patern delay")));
      case 4: (Serial.println(F("Error 102: Pattern Delay value is smaller than the default value")));
      case 3: (Serial.println(F("Error 103: DOUT_START value is smaller than the default value")));
      case 2: (Serial.println(F("Error 104: Period Setting Register is smaller than the pattern play cycle")));
      case 1: (Serial.println(F("Error 105: Memmory Pattern Play is not of even length in trigger delay mode")));
      case 0: (Serial.println(F("Error 105: Memmory Read Conflict")));
    }
  }
}

//SELF CALIBRATION PROCEDURE
int AD9102::self_cal(byte CS) {
  //Set Initial Range for the DACRANGE and COMP_CAL_RANGE
  byte dac_gain_range=B00; //Dac gain =0
  byte comp_cal_range=B00; //Comp Call=0
  register_write(dacrange(dac_gain_range),dacrange_Address,CS);
  //calibration clock frequency Adjustment based on a (2^24)*10~=167,77 Mhz DAC clock
  byte cal_clock_div=B000;
  register_write(calconfig(0,0,0,comp_cal_range,cal_clock_div),calconfig_Address,CS); //set Calibration Clock to 327.680 KHz
  //Enable Callibration clock
  byte cal_clk_enable=B1;
  register_write(calconfig(0,0,comp_cal_range,cal_clk_enable,cal_clock_div),calconfig_Address,CS);
  //Start Calibration cycles
  byte cal_en=B1;
  register_write(calconfig(0,cal_en,comp_cal_range,cal_clk_enable,cal_clock_div),calconfig_Address,CS);
  uint16_t *temp=0,*temp2=0;
  byte bit_temp=0,bit_temp2=0;
  byte comp_ok=0,rset_ok=0,gain_ok=0; //Flags for Offset, Gain and Rset (0 if underflow or overflow)

  //Calibration cycle
  while(!(comp_ok && rset_ok && gain_ok)) {

    //Wait for calibration to finish
    register_write(compoffset(B1),compoffset_Address,CS);
    byte cal_status=B00; //set cal status
    while(cal_status==B00) { //Wait until calibration cycle is completed
      register_read(calconfig_Address,CS,temp);
      register_read(compoffset_Address,CS,temp2);
      cal_status=(bitRead(*temp,7)<<1) + bitRead(*temp2,1); //read Calibration status
    }

    //Fail Abort
    if(!bitRead(*temp2,1)) return -1;

    //set START_CAL to 0;
    bitWrite(*temp2,0,0);
    register_write(*temp2,compoffset_Address,CS);

    //Read Calibration Results
    register_read(calconfig_Address,CS,temp);

    //Check Compensation Offset
    if(!comp_ok) {
      bit_temp=bitRead(*temp,14);
      bit_temp2=bitRead(*temp,13);
      if (bit_temp2==0 && bit_temp==0) comp_ok=1; //if no over/underflow then compensation offset calibration is completed;
      else if (bit_temp==1 && comp_cal_range<B11) comp_cal_range++;  //overflow -> increase range
      else if (bit_temp2==1 && comp_cal_range>B00) comp_cal_range--; //underflow -> decrease range
      else if ((bit_temp2==1 && comp_cal_range==B00) || (bit_temp==0 && comp_cal_range==B11)) return -2;  //range could not be adjusted
      if(!comp_ok) calconfig(comp_cal_range,CS); //write back the new range
    }
    //Check Dac Gain
    if(!gain_ok) {
      bit_temp=bitRead(*temp,10);
      bit_temp2=bitRead(*temp,9);
      if (bit_temp2==0 && bit_temp==0) gain_ok=1; //if no over/underflow then dac gain calibrations is completed.
      else if (bit_temp==1 && dac_gain_range<B11) dac_gain_range++; //overflow -> increase range
      else if (bit_temp2==1 && dac_gain_range>B00) dac_gain_range--; //underflow -> decrease range
      else if ((bit_temp2==1 && dac_gain_range==B00) || (bit_temp==0 && dac_gain_range==B11)) return -3;
      if(!gain_ok) register_write(dacrange(dac_gain_range),dacrange_Address, CS); //write back new range
    }

    //Check Rset
    if(!rset_ok ) {
      bit_temp=bitRead(*temp,12);
      bit_temp2=bitRead(*temp,11);
      if (bit_temp2==0 && bit_temp==0) rset_ok=1; //if no over/underflow then dac gain calibrations is completed.
      else if ((bit_temp || bit_temp2) && (gain_ok && comp_ok)) return -4;
      }
  }

  //Calibration Values Adjustment

  //RSET adjustment
  register_read(dacrset_Address,CS,temp);
  *temp2= (*temp/0x100)%0x20; //isolate bits 12-8
  bitWrite(*temp,15,1);
  *temp>>=5;
  *temp<<=5; //fil thee first 4 bits with zeros
  *temp+= *temp2;
  register_write(*temp,dacrset_Address,CS);

  //DACAGAIN Adjustment
  register_read(dacagain_Address,CS,temp);
  *temp2= (*temp/0x100)%0x80; //isolate bits 14-8
  *temp>>=7;
  *temp<<=7; //fill the first 7 bits with zeros
  *temp+= *temp2;
  register_write(*temp,dacrset_Address,CS);

  //Finalise calibration
  register_read(calconfig_Address,CS,temp);
  bitWrite(*temp,6,0);
  bitWrite(*temp,1,0);
  register_write(*temp,calconfig_Address,CS);

  return 0;
}

void AD9102::calibration(byte CS) {
  Serial.println("Starting Calibration");
  uint16_t temp=self_cal(CS);
  switch(temp) {
    case -1 : Serial.println(F("Error 201: Calibration Cycle Terminated Unexpectadly "));
    case -2 : Serial.println(F("Error 202: Compensation Offset failed to calibrate"));
    case -3 : Serial.println(F("Error 203: Dac Analog Gain fail to Calibrate"));
    case -4 : Serial.println(F("Error 204: Rset Resistor failed to Calibrate"));
  }
  cal_config_read(CS);
}

void AD9102::initialise() { //subject to change only for test purposes
  register_write(spiconfig(0),spiconfig_Address,CS);
  register_write(powerconfig(),powerconfig_Address,CS);
  register_write(clockconfig(),clockconfig_Address,CS);
  register_write(refadj(0),refadj_Address,CS);
  register_write(dacagain(0),dacagain_Address,CS);
  register_write(dacrange(0),dacrange_Address,CS);
  register_write(dacrset(1,0),dacrset_Address,CS);
  register_write(calconfig(0,1,B01,1,0),calconfig_Address,CS);
  register_write(compoffset(1),compoffset_Address,CS);
  register_write(ramupdate(0),ramupdate_Address,CS);
  register_write(pat_status(0,1),pat_status_Address,CS);
  register_write(pat_type(0),pat_type_Address,CS);
  register_write(pattern_dly(0),pattern_dly_Address,CS);
  register_write(dacdof(0),dacdof_Address,CS);
  register_write(wav_config(0,0),wav_config_Address,CS);
  register_write(pat_timebase(0,0,0),pat_timebase_Address,CS);
  register_write(pat_period(0),pat_period_Address,CS);
  register_write(dac_pat(0),dac_pat_Address,CS);
  register_write(dout_start(0),dout_start_Address,CS);
  register_write(dout_config(0),dout_config_Address,CS);
  register_write(dac_cst(0),dac_cst_Address,CS);
  register_write(dac_dgain(0x0000),dac_dgain_Address,CS);
  register_write(saw_config(0,0),saw_config_Address,CS);
  register_write(dds_tw32(0),dds_tw32_Address,CS);
  register_write(dds_tw1(0),dds_tw1_Address,CS);
  register_write(dds_pw(0),dds_pw_Address,CS);
  register_write(trig_tw_sel(0),trig_tw_sel_Address,CS);
  register_write(dds_config(0,0,0),dds_config_Address,CS);
  register_write(tw_ram_config(0),tw_ram_config_Address,CS);
  register_write(start_dly(0x0000),start_dly_Address,CS);
  register_write(start_addr(0x0000),start_addr_Address,CS);
  register_write(stop_addr(0x0000),stop_adr_Address,CS);
  register_write(dds_cyc(0x0000),dds_cyc_Address,CS);
}
