#include <Arduino.h>
/*
 * AD9102.h
 * Library for the Analog Devices
 * Waveform Generation IC AD9102
 * for the Arduino Platform
 * Created by Evangelos Petrongonas,
 * Email: vpetrog@hotmail.com
 * Athens 2017
 * Part of the ArduGen Project
 * Licensed under :MIT
 * Copyright (c) 2017 Evangelos Petrongonas
 */

 #ifndef AD9102_h
 #define AD9102_h

 #include "Arduino.h"
 #include "SPI.h"

class AD9102
{
public:
  //Costructor
  AD9102(byte chip_select);
  //Destructor
  ~AD9102(){};
  //Register Write Functions
  uint16_t spiconfig(byte RESET); //SPI congig register 0x00
  uint16_t powerconfig();
  uint16_t clockconfig();
  uint16_t refadj(uint16_t BGDR_value);
  uint16_t dacagain(uint16_t DAC_AGain);
  uint16_t dacrange(byte DAC_GAIN_RNG);
  uint16_t dacrset(byte RW, byte DAC_RSET);
  uint16_t calconfig(byte CAL_RESET, byte CAL_MODE_EN,byte COMP_CAL_RNG, byte CAL_CLK_EN,byte CAL_CLK_DIV);
  void calconfig(byte cal_range,byte CS);
  uint16_t compoffset(byte START_CAL);
  uint16_t ramupdate(byte UPDATE);
  uint16_t pat_status(byte MEM_ACCESS,byte RUN);
  uint16_t pat_type(byte PATTERN_RPT);
  uint16_t pattern_dly(uint16_t PATTERN_DELAY);
  uint16_t dacdof(uint16_t DAC_DIG_OFFSET);
  uint16_t wav_config(byte PRESTORE_SEL, byte WAVE_SEL);
  uint16_t pat_timebase(byte HOLD,byte PAT_PERIOD_BASE,byte START_DELAY_BASE) ;
  uint16_t pat_period(uint16_t PATTERN_PERIOD);
  uint16_t dac_pat(byte DAC_REPEAT_CYCLE);
  uint16_t dout_start(uint16_t DOUT_STARTV);
  uint16_t dout_config(byte DOUT_STOP);
  uint16_t dac_cst(uint16_t DAC_CONST);
  uint16_t  dac_dgain(uint16_t DAC_DIG_GAIN);
  uint16_t saw_config(byte SAW_STEP, byte SAW_TYPE);
  uint16_t dds_tw32(uint16_t DDSTW_MSB);
  uint16_t dds_tw1(byte DDSTW_LSB);
  uint16_t dds_pw(uint16_t DDS_PHASE);
  uint16_t trig_tw_sel(byte TRIG_DELAY_EN);
  uint16_t dds_config(byte DDS_COS_EN,byte PHASE_MEM_EN, byte TW_MEM_EN);
  uint16_t tw_ram_config(byte TW_MEM_SHIFT);
  uint16_t start_dly(uint16_t START_DELAY);
  uint16_t start_addr(uint16_t START_ADDR);
  uint16_t stop_addr(uint16_t STOP_ADDR);
  uint16_t dds_cyc(uint16_t DDS_CYCLE);
  void power_config_read(uint16_t register_value);
  void cal_config_read(uint16_t register_value);
  void cfg_error_read(uint16_t register_value);
  void cfg_error_read(uint16_t register_value,byte bit);
  void calibration(byte CS);
  void initialise();
private:
  byte CS;
  //private functions
  int self_cal(byte CS);
  //Register read and write functions
  void register_write(uint16_t register_value,uint16_t register_Address, byte CS);
  void register_read(uint16_t register_Address,byte CS,uint16_t* Value);
  //Register Addresses
  const uint16_t spiconfig_Address=0x0000;
  const uint16_t powerconfig_Address=0x0001;
  const uint16_t clockconfig_Address=0x0002;
  const uint16_t refadj_Address=0x0003;
  const uint16_t dacagain_Address=0x0007;
  const uint16_t dacrange_Address=0x0008;
  const uint16_t dacrset_Address=0x000C;
  const uint16_t calconfig_Address=0x000D;
  const uint16_t compoffset_Address=0x000E;
  const uint16_t ramupdate_Address=0x001D;
  const uint16_t pat_status_Address=0x001E;
  const uint16_t pat_type_Address=0x001F;
  const uint16_t pattern_dly_Address=0x0020;
  const uint16_t dacdof_Address=0x0025;
  const uint16_t wav_config_Address=0x0027;
  const uint16_t pat_timebase_Address=0x0028;
  const uint16_t pat_period_Address=0x0029;
  const uint16_t dac_pat_Address=0x002B;
  const uint16_t dout_start_Address=0x002C;
  const uint16_t dout_config_Address=0x002D;
  const uint16_t dac_cst_Address=0x0031;
  const uint16_t dac_dgain_Address=0x0035;
  const uint16_t saw_config_Address=0x0037;
  const uint16_t dds_tw32_Address=0x003E;
  const uint16_t dds_tw1_Address=0x003F;
  const uint16_t dds_pw_Address=0x0043;
  const uint16_t trig_tw_sel_Address=0x0044;
  const uint16_t dds_config_Address=0x0045;
  const uint16_t tw_ram_config_Address=0x0047;
  const uint16_t start_dly_Address=0x005C;
  const uint16_t start_addr_Address=0x005D;
  const uint16_t stop_adr_Address=0x005E;
  const uint16_t dds_cyc_Address=0x005F;
  const uint16_t cgf_error_Address=0x0060;
};


#endif
