{\rtf1\ansi\ansicpg1252\cocoartf1404\cocoasubrtf470
{\fonttbl\f0\fnil\fcharset0 Monaco;}
{\colortbl;\red255\green255\blue255;\red63\green127\blue95;\red127\green0\blue85;\red42\green0\blue255;
\red0\green80\blue50;\red0\green0\blue192;\red100\green40\blue128;}
\margl1440\margr1440\vieww10800\viewh8400\viewkind0
\pard\tx720\tx1440\tx2160\tx2880\tx3600\tx4320\tx5040\tx5760\tx6480\tx7200\tx7920\tx8640\pardirnatural\partightenfactor0

\f0\fs22 \cf0 \
\pard\pardeftab720\partightenfactor0
\cf0 \
\
\cf2 /**************************************************************************//**\cf0 \
\cf2  * @file\cf0 \
\cf2  * @brief Empty Project\cf0 \
\cf2  * @author Energy \ul Micro\ulnone  AS\cf0 \
\cf2  * @version 3.20.2\cf0 \
\cf2  ******************************************************************************\cf0 \
\cf2  * @section License\cf0 \
\cf2  * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>\cf0 \
\cf2  *******************************************************************************\cf0 \
\cf2  *\cf0 \
\cf2  * This file is licensed under the Silicon Labs Software License Agreement. See\cf0 \
\cf2  * "http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt"\cf0 \
\cf2  * for details. Before using this software for any purpose, you must agree to the\cf0 \
\cf2  * terms of that agreement.\cf0 \
\cf2  *\cf0 \
\cf2  ******************************************************************************/\cf0 \
\cf2 //#include "em_device.h"\cf0 \
\cf2 //#include "em_chip.h"\cf0 \
\
\cf2 /**************************************************************************//**\cf0 \
\cf2  * @brief  Main function\cf0 \
\cf2  *****************************************************************************/\cf0 \
\
\
\cf3 #include\cf0  \cf4 <stdint.h>\cf0 \
\cf3 #include\cf0  \cf4 <stdbool.h>\cf0 \
\cf3 #include\cf0  \cf4 "em_device.h"\cf0 \
\cf3 #include\cf0  \cf4 "em_chip.h"\cf0 \
\cf3 #include\cf0  \cf4 "em_int.h"\cf0 \
\cf3 #include\cf0  \cf4 "em_cmu.h"\cf0 \
\cf3 #include\cf0  \cf4 "em_emu.h"\cf0 \
\cf3 #include\cf0  \cf4 "bsp.h"\cf0 \
\cf3 #include\cf0  \cf4 "em_letimer.h"\cf0 \
\cf3 #include\cf0  \cf4 "em_gpio.h"\cf0 \
\cf3 #include\cf0  \cf4 "bsp_trace.h"\cf0 \
\cf3 #include\cf0  \cf4 "em_acmp.h"\cf0 \
\cf3 #include\cf0  \cf4 "em_timer.h"\cf0 \
\cf3 #include\cf0  \cf4 "em_dma.h"\cf0 \
\cf3 #include\cf0  \cf4 "em_adc.h"\cf0 \
\cf3 #include\cf0  \cf4 "dmactrl.h"\cf0 \
\cf3 #include\cf0  \cf4 "em_i2c.h"\cf0 \
\cf3 #include\cf0  \cf4 "em_leuart.h"\cf0 \
\cf3 #include\cf0  \cf4 "myProject_1.h"\cf0 \
\cf3 #include\cf0  \cf4 "malloc.h"\cf0 \
\
\cf3 #define\cf0  size_circularbuffer 20\
\
\
\cf5 DMA_CB_TypeDef\cf0  cb;\
\
\cf2 /* Transfer Flag */\cf0 \
\cf3 volatile\cf0  bool transferActive;\
\
\cf2 /* ADC Transfer Data */\cf0 \
\
\cf3 volatile\cf0  \cf5 uint16_t\cf0  DstPtr[ADCSAMPLES];               \cf2 //buffer for storing temperature readings with DMA\cf0 \
\cf3 volatile\cf0  \cf5 uint16_t\cf0  Dstbuffer[ADCSAMPLES];			\cf2 //buffer for storing temperature readings without DMA\cf0 \
\
\cf5 uint32_t\cf0  i2c_txBuffer1;								\cf2 // variables for storing ADC values of two ADCs from TSL2561 device\cf0 \
\cf5 uint32_t\cf0  i2c_txBuffer2;								\cf2 // variables for storing ADC values of two ADCs from TSL2561 device\cf0 \
\cf5 uint32_t\cf0  i2c_txBuffer3;\
\cf5 uint32_t\cf0  i2c_txBuffer4;\
\cf5 uint32_t\cf0  i2c_txBuffer5;\
\cf5 uint32_t\cf0  i2c_txBuffer6;								\cf2 // variables for storing ADC values of two ADCs from TSL2561 device\cf0 \
\
\
\cf5 uint32_t\cf0  tsl_txBuffer2;\
\cf5 uint32_t\cf0  tsl_txBuffer3;\
\cf5 uint32_t\cf0  tsl_txBuffer4;\
\cf5 uint32_t\cf0  tsl_txBuffer5;\
\
\cf5 uint8_t\cf0  emptycheck(\cf5 uint8_t\cf0  n);\
\cf5 uint8_t\cf0  fullcheck(\cf5 uint8_t\cf0  n);\
\cf5 uint8_t\cf0  ringlocate(\cf5 uint8_t\cf0  i);\
\cf5 uint8_t\cf0  getelement(\cf5 uint8_t\cf0  n,\cf5 uint8_t\cf0  current_loc, \cf5 uint8_t\cf0  *str);\
\cf5 uint8_t\cf0  putelement(\cf5 uint8_t\cf0  *zz, \cf5 uint8_t\cf0  n, \cf5 uint8_t\cf0  current_loc,\cf5 uint8_t\cf0  *str);\
\cf5 uint8_t\cf0  circularbuffer[size_circularbuffer];\
\
\
\
\cf3 char\cf0  warning1[] = \{\cf4 "The buffer is full,ERROR!\\0"\cf0 \};\
\cf3 char\cf0  warning2[] = \{\cf4 "The buffer is empty,ERROR!\\0"\cf0 \};\
\cf3 char\cf0  warning3[] = \{\cf4 "\cf4 \ul \ulc4 Overfill\cf4 \ulnone ,ERROR!"\cf0 \};\
\cf3 char\cf0  warning4[] = \{\cf4 "\cf4 \ul \ulc4 Overempty\cf4 \ulnone ,ERROR!"\cf0 \};\
\cf3 char\cf0  pass[] = \{\cf4 "UNIT TEST PASS!\\0"\cf0 \};\
\cf3 char\cf0  fail[] = \{\cf4 "UNIT TEST FAIL!\\0"\cf0 \};\
\
\cf3 char\cf0  des[size_circularbuffer];\
\cf3 int\cf0  st[size_circularbuffer]=\{0,0,0,0,0,0,0,0\};\
\cf5 uint8_t\cf0  tf[size_circularbuffer];\
\cf5 uint8_t\cf0  heartbeat_count,average;\
\cf5 uint8_t\cf0  *str1;\
\cf5 uint8_t\cf0  *str2;\
\cf3 int\cf0  userflag = 0;\
\
\cf3 struct\cf0  ringbuf\
   \{   \cf5 uint8_t\cf0  \cf6 iput\cf0 ;\
       \cf5 uint8_t\cf0  \cf6 iget\cf0 ;\
       \cf5 uint8_t\cf0  \cf6 n\cf0 ;\
   \}rb=\{0,0,0\};\
\
	\cf3 char\cf0  *str;\
\cf5 uint32_t\cf0  sleep_block_counter[EM4+1];\
\
\cf3 int\cf0  Interval,a,b,c,d,h,p,fl,q,kk,count,timer_count;\
\cf3 int\cf0  t= 0;\
\
\cf3 int\cf0  sumI, sign;\
\cf3 int\cf0  sumF;\
\
\cf3 float\cf0  osc_ratio;\
\cf3 int\cf0  ULFRCO_Calibrated;\
\cf5 int8_t\cf0  string4[40];\
\
\
\
\cf3 void\cf0  LETIMER0_Calibration(\cf3 void\cf0 );\
\cf3 void\cf0  Sleep(\cf3 void\cf0 );\
\cf3 void\cf0  blockSleepMode(\cf5 uint32_t\cf0  Minimummode);\
\cf3 void\cf0  unblockSleepMode(\cf5 uint32_t\cf0  Minimummode);\
\cf3 void\cf0  CMU_Setup(\cf3 void\cf0 );\
\cf3 void\cf0  GPIO_Setup(\cf3 void\cf0 );\
\cf3 void\cf0  LETIMER0_Setup(\cf3 void\cf0 );\
\cf3 static\cf0  \cf3 void\cf0  ACMPInit(\cf3 void\cf0 );\
\cf3 void\cf0  DMA_ACTIVATE(\cf3 void\cf0 );\
\cf3 void\cf0  setupAdc(\cf3 void\cf0 );\
\cf3 float\cf0  convertToCelsius(\cf5 int32_t\cf0  adcSample);\
\
\cf3 void\cf0  ClearTSLInterrupt_TSL(\cf3 void\cf0 );\
\cf3 void\cf0  I2CSetup_TSL(\cf3 void\cf0 );\
\cf3 void\cf0  ClearACK();\
\cf3 void\cf0  writetoTXDATA ( \cf3 int\cf0  n );\
\
\
\cf3 void\cf0  I2CSetup_MMA(\cf3 void\cf0 );\
\cf3 void\cf0  performi2ctransfer_MMA(\cf3 void\cf0 );\
\cf3 void\cf0  writetoI2C(\cf3 int\cf0  reg_address, \cf3 int\cf0  data);\
\cf3 void\cf0  readfromI2C_MMA(\cf5 uint32_t\cf0  read_regaddress);\
\cf3 void\cf0  performI2CTransferAndMonitor_TSL(\cf3 void\cf0 );\
\
\cf3 void\cf0  powerup(\cf3 void\cf0 );\
\cf3 void\cf0  powerdown(\cf3 void\cf0 );\
\
\
\cf3 void\cf0  initLeuart(\cf3 void\cf0 );\
\cf2 //void LED_state(\ul int\ulnone  led, \ul bool\ulnone  state);\cf0 \
\cf2 /***************************************************************************/\cf0  \cf2 /**\cf0 \
\cf2   * Sleep routines\cf0 \
\cf2  ******************************************************************************* */\cf0 \
\
\
\
\
\cf2 /* This code is originally Silicon Labs and copyrighted by Silicon Labs in 2015 and Silicon Labs grants\cf0 \
\cf2  * permission to anyone to use the software for any purpose, including commercial applications, and to alter it,\cf0 \
\cf2  * and to redistribute it freely subject that the origin is not miss represented, altered source version must be\cf0 \
\cf2  * plainly marked, and this notice cannot be altered or removed from any source distribution.\cf0 \
\cf2  *\cf0 \
\cf2  * Routine include :\cf0 \
\cf2  *\cf0 \
\cf2  * void blockSleepMode(uint32_t \ul Minimummode\ulnone )\cf0 \
\cf2  * void unblockSleepMode(uint32_t \ul Minimummode\ulnone )\cf0 \
\cf2  * void(Sleep)void\cf0 \
\cf2 */\cf0 \
\
\cf3 void\cf0  blockSleepMode(\cf5 uint32_t\cf0  Minimummode)\{              \cf2 // block an energy mode so that it does not go lower\cf0 \
	\cf2 //INT_Disable();\cf0 \
	sleep_block_counter[Minimummode]++;\
	\cf2 //INT_Enable();\cf0 \
\}\
\
\cf3 void\cf0  unblockSleepMode(\cf5 uint32_t\cf0  Minimummode)\{            \cf2 // unblock an energy mode after operation completes\cf0 \
	\cf2 //INT_Disable();\cf0 \
	\cf3 if\cf0  (sleep_block_counter[Minimummode]>0)\{\
		sleep_block_counter[Minimummode]--;\
	\}\
	\cf3 else\cf0  sleep_block_counter[Minimummode] = 0;\
	\cf2 //INT_Enable();\cf0 \
\}\
\
\
\
\cf3 void\cf0  Sleep(\cf3 void\cf0 )\{                                               \cf2 // enter EMX sleep routine\cf0 \
	\cf3 if\cf0  (sleep_block_counter[EM0] > 0)\{\}\
	\cf3 else\cf0  \cf3 if\cf0  (sleep_block_counter[EM1] > 0)\
		EMU_EnterEM1();\
	\cf3 else\cf0  \cf3 if\cf0  (sleep_block_counter[EM2] > 0)\
			\cf7 EMU_EnterEM2\cf0 (true);\
	\cf3 else\cf0  \cf3 if\cf0  (sleep_block_counter[EM3] > 0)\
			\cf7 EMU_EnterEM3\cf0 (true);\
\
\}\
\
\
\cf2 //Peripheral functions\cf0 \
\cf2 /***************************************************************************/\cf0  \cf2 /**\cf0 \
\cf2   * CMU Setup\cf0 \
\cf2  ******************************************************************************* */\cf0 \
\cf3 void\cf0  CMU_Setup(\cf3 void\cf0 )\{\
\
\
	\cf7 CMU_OscillatorEnable\cf0 (\cf6 cmuOsc_LFXO\cf0 , true, true);\
	\cf7 CMU_ClockEnable\cf0 (cmuClock_CORELE, true);\
	\cf7 CMU_ClockEnable\cf0 (\cf6 cmuClock_HFPER\cf0 , true);\
	\cf7 CMU_ClockEnable\cf0 (\cf6 cmuClock_TIMER0\cf0 , true);\
	\cf7 CMU_ClockEnable\cf0 (\cf6 cmuClock_TIMER1\cf0 , true);\
	\cf7 CMU_ClockEnable\cf0 (\cf6 cmuClock_LETIMER0\cf0 , true);\
	\cf7 CMU_ClockEnable\cf0 (\cf6 cmuClock_GPIO\cf0 , true);\
	\cf7 CMU_ClockEnable\cf0 (\cf6 cmuClock_ACMP0\cf0 , true);\
	\cf7 CMU_ClockEnable\cf0 (\cf6 cmuClock_DMA\cf0 , true);\
	\cf7 CMU_ClockEnable\cf0 (\cf6 cmuClock_ADC0\cf0 , true);\
	\cf7 CMU_ClockEnable\cf0 (\cf6 cmuClock_I2C1\cf0 , true);\
	\cf7 CMU_ClockEnable\cf0 (\cf6 cmuClock_I2C0\cf0 , true);\
\
\
\
\
	\cf3 if\cf0  (LETIMER0_EM_Mode == EM3)\
	\{\
		\cf3 if\cf0 (CALIBRATION == 1)\
		\{\
			LETIMER0_Calibration();                                           \cf2 // Enter calibration if enable in constants\cf0 \
		\}\
\
		\cf7 CMU_ClockSelectSet\cf0 (\cf6 cmuClock_LFA\cf0 , \cf6 cmuSelect_ULFRCO\cf0 );               \cf2 // Select ULFRCO for EM3\cf0 \
		\cf7 CMU_OscillatorEnable\cf0 (\cf6 cmuOsc_LFXO\cf0 , false, false);\
\
\
	\}\
	\cf3 else\cf0  \cf7 CMU_ClockSelectSet\cf0 (\cf6 cmuClock_LFA\cf0 , \cf6 cmuSelect_LFXO\cf0 );                \cf2 // Select LFXO for all other modes\cf0 \
\
\
\}\
\
\
\cf2 /***************************************************************************/\cf0  \cf2 /**\cf0 \
\cf2   * GPIO Setup\cf0 \
\cf2  ******************************************************************************* */\cf0 \
\
\cf3 void\cf0  GPIO_Setup(\cf3 void\cf0 )\{\
\
  	\cf7 GPIO_PinModeSet\cf0  (port_internal_tempsensor, 6, \cf6 gpioModeInput\cf0 , 1);\
  	\cf7 GPIO_PinModeSet\cf0  (port_ADC_excite, 6, \cf6 gpioModePushPull\cf0 , 0);\
\
  	\cf7 GPIO_PinModeSet\cf0 (\cf6 gpioPortC\cf0 , 5, \cf6 gpioModeWiredAndPullUpFilter\cf0 , 1);			\cf2 /*GPIO pins for I2C - SDA and SCL*/\cf0 \
	\cf7 GPIO_PinModeSet\cf0 (\cf6 gpioPortC\cf0 , 4, \cf6 gpioModeWiredAndPullUpFilter\cf0 , 1);\
\
	\cf7 GPIO_PinModeSet\cf0 (\cf6 gpioPortD\cf0 , 7, \cf6 gpioModeWiredAndPullUpFilter\cf0 , 1);			\cf2 /*GPIO pins for I2C - SDA and SCL*/\cf0 \
    \cf7 GPIO_PinModeSet\cf0 (\cf6 gpioPortD\cf0 , 6, \cf6 gpioModeWiredAndPullUpFilter\cf0 , 1);\
\
    \cf7 GPIO_PinModeSet\cf0  (port_LED , 3, \cf6 gpioModePushPullDrive\cf0 , 0);\
	\cf7 GPIO_PinModeSet\cf0  (port_LED , 2, \cf6 gpioModePushPullDrive\cf0 , 0);\
\
  	\cf7 GPIO_PinModeSet\cf0  (\cf6 gpioPortB\cf0 , 11, \cf6 gpioModeInput\cf0 , 1);        \cf2 // LO+\cf0 \
  	\cf7 GPIO_PinModeSet\cf0  (\cf6 gpioPortB\cf0 , 12, \cf6 gpioModeInput\cf0 , 1);        \cf2 // LO-\cf0 \
  	\cf7 GPIO_PinModeSet\cf0  (\cf6 gpioPortC\cf0 , 0, \cf6 gpioModeInput\cf0 , 1);        \cf2 // OUT\cf0 \
\
\
\
\
\
\
  \}\
\
\
\cf2 /***************************************************************************/\cf0  \cf2 /**\cf0 \
\cf2   * \ul Powerup\ulnone  routine\cf0 \
\cf2  ******************************************************************************* */\cf0 \
\
\
\cf3 void\cf0  powerup(\cf3 void\cf0 )\
\{\
\
\
	\cf2 /*  MMA\cf0 \
\cf2 	 *\cf0 \
\cf2 	 * 3V3  4 - PD0\cf0 \
\cf2 	 * GND  19\cf0 \
\cf2 	 * SCL  9\cf0 \
\cf2 	 * SDA  7\cf0 \
\cf2 	 *\cf0 \
\cf2 	 *\cf0 \
\cf2 	 *\cf0 \
\cf2 	 *\cf0 \
\cf2 	 *\cf0 \
\cf2 	 *\cf0 \
\cf2 	 * 224 in i2cbuffer which is OUT_Z_MSB is the threshold value for alert\cf0 \
\cf2 	 */\cf0 \
\
\
\
	\cf2 /*  TSL\cf0 \
\cf2 	 *\cf0 \
\cf2 	 *\cf0 \
\cf2 	 * INT 6 - PD1\cf0 \
\cf2 	 * 3V3 8 - PD2\cf0 \
\cf2 	 * GND\cf0 \
\cf2 	 * SCL  PD7 - 17\cf0 \
\cf2 	 * SDA  PD6 - 16\cf0 \
\cf2 	 *\cf0 \
\cf2 	 */\cf0 \
\
\
\
\
	\cf7 GPIO_PinModeSet\cf0 (port_MMA_power, 0, \cf6 gpioModePushPull\cf0 , 0);				\cf2 // enable power to MMA\cf0 \
	GPIO_PinOutSet(port_MMA_power, 0);\
\
	\cf7 GPIO_PinModeSet\cf0 (port_TSL_power, 2, \cf6 gpioModePushPull\cf0 , 0);				\cf2 // enable power to MMA\cf0 \
	GPIO_PinOutSet(port_TSL_power, 2);\
\
	\cf5 TIMER_Init_TypeDef\cf0  timer0 =												\cf2 // initialize timer for using a delay to let TSL to stabilize on being powered up.\cf0 \
		     \{\
		       .enable     = false,\
		       .debugRun   = false,\
		       .prescale   = \cf6 timerPrescale1\cf0 ,\
		       .clkSel     = \cf6 timerClkSelHFPerClk\cf0 ,\
		       .fallAction = \cf6 timerInputActionNone\cf0 ,\
		       .riseAction = \cf6 timerInputActionNone\cf0 ,\
		       .mode       = \cf6 timerModeUp\cf0 ,\
		       .dmaClrAct  = false,\
		       .quadModeX4 = false,\
		       .oneShot    = true,\
		       .sync       = false,\
		     \};\
\
	\cf7 TIMER_Init\cf0 (TIMER0, &timer0);\
\
	TIMER0->\cf6 CNT\cf0  = 0x00;\
	TIMER0->\cf6 CMD\cf0  = timer_stop;												\cf2 // insuring timer is off before counting starts\cf0 \
\
	TIMER0->\cf6 CMD\cf0  = timer_start;\
\
	\cf3 while\cf0  (TIMER0->\cf6 CNT\cf0  <= timer_counts_TSLdelay);\
\
	TIMER0->\cf6 CMD\cf0  = timer_stop;\
\
\
\
\
\}\
\
\
\cf2 /***************************************************************************/\cf0  \cf2 /**\cf0 \
\cf2   * Power down routine\cf0 \
\cf2  ******************************************************************************* */\cf0 \
\
\
\cf3 void\cf0  powerdown()\
\{\
\
	NVIC_DisableIRQ(\cf6 GPIO_ODD_IRQn\cf0 );\
	\cf7 GPIO_PinModeSet\cf0 (port_MMA_SCL, 5, \cf6 gpioModeDisabled\cf0 , 0);				\cf2 /*GPIO pins for MMA I2C - SDA and SCL*/\cf0 \
    \cf7 GPIO_PinModeSet\cf0 (port_MMA_SDA, 4, \cf6 gpioModeDisabled\cf0 , 0);				\cf2 //SDA pin configured\cf0 \
\
	\cf7 GPIO_PinModeSet\cf0 (port_TSL_SCL, 7, \cf6 gpioModeDisabled\cf0 , 0);				\cf2 /*GPIO pins for TSL I2C - SDA and SCL*/\cf0 \
    \cf7 GPIO_PinModeSet\cf0 (port_TSL_SDA, 6, \cf6 gpioModeDisabled\cf0 , 0);				\cf2 //SDA pin configured\cf0 \
\
\
  	GPIO_IntConfig(port_i2c_interrupt, 1, false, true, false);			\cf2 //disable external interrupt\cf0 \
  	\cf7 GPIO_PinModeSet\cf0 (port_i2c_interrupt, 1, \cf6 gpioModeDisabled\cf0 , 0);\
\
    \cf7 GPIO_PinModeSet\cf0 (port_MMA_power, 0, \cf6 gpioModeDisabled\cf0 , 0);			\cf2 // disable power to MMA\cf0 \
    GPIO_PinOutClear(port_MMA_power, 0);\
\
    \cf7 GPIO_PinModeSet\cf0 (port_TSL_power, 2, \cf6 gpioModeDisabled\cf0 , 0);			\cf2 // disable power to TSL2561\cf0 \
    GPIO_PinOutClear(port_TSL_power, 2);\
       \cf2 //	GPIO_PinModeSet(gpioPortD, 1, gpioModeDisabled, 1);\cf0 \
       	\cf2 //	      GPIO_IntConfig( gpioPortD, 1, false, true, false );\cf0 \
\
\
\}\
\
\
\
\
\cf2 /**************************************************************************//**\cf0 \
\cf2 TSL Interrupt Clearing Routine\cf0 \
\cf2  *****************************************************************************/\cf0 \
\
\cf3 void\cf0  ClearTSLInterrupt_TSL(\cf3 void\cf0 )\
\{\
\
\
\
	   I2C0->\cf6 TXDATA\cf0  = TSLAddr_write;							\cf2 // give address to slave \ul fr\ulnone  clearing \ul interrut\ulnone  on TSL\cf0 \
	   I2C0->\cf6 CMD\cf0  = I2C_START;\
\
\
	   \cf3 while\cf0 (! (I2C0->\cf6 IF\cf0  & I2C_IF_ACK));\
\
	   ClearACK();\
\
     writetoTXDATA( TSL_clearinterrupt );					\cf2 // clearing TSL interrupt by clearing bit in command register\cf0 \
\
\
     I2C0->\cf6 CMD\cf0  = I2C_STOP;\
\
     ClearACK();\
\
\
\
\
\}\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\cf2 /**************************************************************************//**\cf0 \
\cf2 GPIO Interrupt Handling Routine\cf0 \
\cf2  *****************************************************************************/\cf0 \
\cf3 void\cf0  GPIO_ODD_IRQHandler(\cf3 void\cf0 )\
   \{\
	 INT_Disable();												\cf2 // disable or clear all interrupts\cf0 \
\
	GPIO_IntClear(gpioclear_extint);\
\
	blockSleepMode(EM1);										\cf2 // enter energy mode EM1 on starting communication with the slave device\cf0 \
\
\
	 I2C0->\cf6 TXDATA\cf0  = TSLAddr_write;								\cf2 // send address of device\cf0 \
    I2C0->\cf6 CMD\cf0  = I2C_START;									\cf2 // send start signal\cf0 \
\
    \cf3 while\cf0 (! (I2C0->\cf6 IF\cf0  & I2C_IF_ACK));							\cf2 // wait for ACK\cf0 \
\
    ClearACK();												\cf2 // clear ACK after receipt\cf0 \
\
    writetoTXDATA( Addr_ADCval );								\cf2 // access ADC registers on TSL2561 through command \ul reg\cf0 \ulnone \
\
    I2C0->\cf6 CMD\cf0  = I2C_START;									\cf2 // send a repeat start\cf0 \
    writetoTXDATA( TSLAddr_read );								\cf2 // send device address with LSB = 1 for read\cf0 \
\
    \cf3 while\cf0  (!(I2C0->\cf6 IF\cf0  & I2C_IF_RXDATAV));						\cf2 // start reading\cf0 \
    tsl_txBuffer2 = I2C0->\cf6 RXDATA\cf0 ;\
\
    I2C0->\cf6 CMD\cf0  |= I2C_CMD_ACK;\
\
    \cf3 while\cf0  (!(I2C0->\cf6 IF\cf0  & I2C_IF_RXDATAV));\
    tsl_txBuffer3 = I2C0->\cf6 RXDATA\cf0 ;\
\
    I2C0->\cf6 CMD\cf0  |= I2C_CMD_ACK;\
\
    \cf3 while\cf0  (!(I2C0->\cf6 IF\cf0  & I2C_IF_RXDATAV));\
    tsl_txBuffer4 = I2C0->\cf6 RXDATA\cf0 ;\
\
    I2C0->\cf6 CMD\cf0  |= I2C_CMD_ACK;\
\
    \cf3 while\cf0  (!(I2C0->\cf6 IF\cf0  & I2C_IF_RXDATAV));\
    tsl_txBuffer5 = I2C0->\cf6 RXDATA\cf0 ;\
\
    I2C0->\cf6 CMD\cf0  |= I2C_CMD_NACK;\
\
    I2C0->\cf6 CMD\cf0  = I2C_STOP;\
\
    unblockSleepMode(EM1);\
\
\
           a = 256*tsl_txBuffer3 + tsl_txBuffer2;						\cf2 // calculate output of ADC0 and compare\cf0 \
                 \cf3 if\cf0  (a<15 )\
                 \{\
               	  GPIO_PinOutSet(port_LED, 2);\
\
                 \}\
\
          b = 256*tsl_txBuffer5 +  tsl_txBuffer4;						\cf2 // calculate output of ADC1 and compare\cf0 \
                \cf3 if\cf0 (b>2048)\
                \{\
               	 GPIO_PinOutClear(port_LED, 2);\
\
                \}\
\
\
\
                ClearTSLInterrupt_TSL();\
          INT_Enable();\
\
    \}\
\
\
\
\
\
\
\
\
\cf2 /**************************************************************************//**\cf0 \
\cf2  Routine to clear ACK\cf0 \
\cf2  *****************************************************************************/\cf0 \
\
\cf3 void\cf0  ClearACK()\
\{\
	\cf3 int\cf0  fl = (I2C0->\cf6 IF\cf0  & I2C_IF_ACK);\
	I2C0->\cf6 IFC\cf0  = fl;\
\}\
\
\
\
\
\cf2 /**************************************************************************//**\cf0 \
\cf2  Write address/data to TXDATA\cf0 \
\cf2  *****************************************************************************/\cf0 \
\
\
\
\cf3 void\cf0  writetoTXDATA ( \cf3 int\cf0  n )\
\{\
	I2C0->\cf6 TXDATA\cf0  = n;\
	\cf3 while\cf0 (! (I2C0->\cf6 IF\cf0  & I2C_IF_ACK));\
	\cf3 int\cf0  fl = (I2C0->\cf6 IF\cf0  & I2C_IF_ACK);\
	I2C0->\cf6 IFC\cf0  = fl;\
\}\
\
\
\
\cf2 /***************************************************************************/\cf0  \cf2 /**\cf0 \
\cf2   * I2C Setup routine\cf0 \
\cf2  ******************************************************************************* */\cf0 \
\
\cf3 void\cf0  I2CSetup_TSL(\cf3 void\cf0 )\
\{\
	\cf5 I2C_Init_TypeDef\cf0  i2cInit1 = I2C_INIT_DEFAULT;\
\
	\cf7 I2C_Init\cf0 (I2C0, &i2cInit1);\
\
	\cf7 CMU_ClockEnable\cf0 (\cf6 cmuClock_HFPER\cf0 , true);\
	\cf7 CMU_ClockEnable\cf0 (\cf6 cmuClock_I2C0\cf0 , true);\
    \cf7 CMU_ClockEnable\cf0 (\cf6 cmuClock_GPIO\cf0 , true);\
\
\
	  \cf7 GPIO_PinModeSet\cf0 (port_TSL_SCL, 7, \cf6 gpioModeWiredAndPullUpFilter\cf0 , 1);\
	  \cf7 GPIO_PinModeSet\cf0 (port_TSL_SDA, 6, \cf6 gpioModeWiredAndPullUpFilter\cf0 , 1);\
      \cf7 GPIO_PinModeSet\cf0 (port_i2c_interrupt, 1, \cf6 gpioModeInput\cf0 , 1);\
\
\
\
\
\
	\cf3 for\cf0  (\cf3 int\cf0  i = 0; i < 9; i++)\
	  \{\
	    \cf2 /* TBD: Seems to be clocking at \ul appr\ulnone  80kHz-120kHz depending on compiler\cf0 \
\cf2 	     * optimization when running at 14MHz. A bit high for standard mode devices,\cf0 \
\cf2 	     * but DVK only has fast mode devices. Need however to add some time\cf0 \
\cf2 	     * measurement in order to not be dependable on frequency and code executed.\cf0 \
\cf2 	     */\cf0 \
	    \cf7 GPIO_PinModeSet\cf0 (port_TSL_SCL, 7, \cf6 gpioModeWiredAnd\cf0 , 0);\
	    \cf7 GPIO_PinModeSet\cf0 (port_TSL_SCL, 7, \cf6 gpioModeWiredAnd\cf0 , 1);\
	  \}\
\
\
    I2C0->\cf6 ROUTE\cf0  = I2C_ROUTE_SDAPEN |										\cf2 // route pins used for I2c buses SDA and SCL\cf0 \
    	              I2C_ROUTE_SCLPEN |\
    	       (I2C_ROUTE_LOCATION_LOC1);\
\
    \cf3 if\cf0 (I2C0->\cf6 STATE\cf0  & I2C_STATE_BUSY)\{										\cf2 // reset registers and device\cf0 \
    	I2C0->\cf6 CMD\cf0  = I2C_CMD_ABORT ;\
    	\}\
\
    \cf3 int\cf0  fl = I2C0->\cf6 IF\cf0 ;\
        I2C0->\cf6 IFC\cf0  = fl;\
\}\
\
\
\
\
\cf2 /***************************************************************************/\cf0  \cf2 /**\cf0 \
\cf2   * I2C Transfer\cf0 \
\cf2  ******************************************************************************* */\cf0 \
\
\cf3 void\cf0  performI2CTransferAndMonitor_TSL(\cf3 void\cf0 )\
\{\
	blockSleepMode(EM1);\
\
\
\
\
\
   I2C0->\cf6 TXDATA\cf0  = TSLAddr_write;\
   I2C0->\cf6 CMD\cf0  = I2C_START;\
\
\
   \cf3 while\cf0 (! (I2C0->\cf6 IF\cf0  & I2C_IF_ACK));\
\
   ClearACK();\
\
\
\
   writetoTXDATA( CMD_WORD );     										\cf2 /* Implementing word transfer, directly pass the subsequent commands.\cf0 \
\cf2  	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 No need to specify address every time as it is incremented by itself.*/\cf0 \
\
   writetoTXDATA( 0x00 );\
\
   writetoTXDATA( TimingReg );\
\
   writetoTXDATA( THLOLO );\
\
   writetoTXDATA( THLOHI );\
\
   writetoTXDATA( THHILO );\
\
   writetoTXDATA( THHIHI );\
\
   writetoTXDATA( TSL_IntReg );\
\
\
   I2C0->\cf6 CMD\cf0  = I2C_STOP;\
\
\
\
\
\
\cf2 //   ClearACK();\cf0 \
\
   I2C0->\cf6 TXDATA\cf0  = TSLAddr_write;\
      I2C0->\cf6 CMD\cf0  = I2C_START;\
\
\
      \cf3 while\cf0 (! (I2C0->\cf6 IF\cf0  & I2C_IF_ACK));\
\
      ClearACK();\
\
\
\
      writetoTXDATA( CMD_WORD );     									\cf2 /* Implementing word transfer, directly pass the subsequent commands.\cf0 \
\cf2     	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 No need to specify address every time as it is incremented by itself.*/\cf0 \
\
      writetoTXDATA( PowerUp );\
      I2C0->\cf6 CMD\cf0  = I2C_STOP;\
\
\
\
\
\
   \cf7 GPIO_PinModeSet\cf0 (port_i2c_interrupt, 1, \cf6 gpioModeInput\cf0 , 1);\
   	GPIO_IntConfig( port_i2c_interrupt, 1, false, true, true );\
   \cf2 //	 NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);\cf0 \
   	 NVIC_EnableIRQ(\cf6 GPIO_ODD_IRQn\cf0 );\
\
\
\
   unblockSleepMode(EM1);\
\
\
\
\
\
\
\}\
\
\
\cf3 void\cf0  I2CSetup_MMA(\cf3 void\cf0 )\
\{\
\
\
	\cf5 I2C_Init_TypeDef\cf0  i2cInit = I2C_INIT_DEFAULT;\
\
	\cf7 I2C_Init\cf0 (I2C1, &i2cInit);\
\
	\cf3 if\cf0 (I2C1->\cf6 STATE\cf0  & I2C_STATE_BUSY)\{\
	I2C1->\cf6 CMD\cf0  = I2C_CMD_ABORT ;\
	\}\
\
	\cf7 CMU_ClockEnable\cf0 (\cf6 cmuClock_HFPER\cf0 , true);\
	\cf7 CMU_ClockEnable\cf0 (\cf6 cmuClock_I2C1\cf0 , true);\
    \cf7 CMU_ClockEnable\cf0 (\cf6 cmuClock_GPIO\cf0 , true);\
\
\
	  \cf7 GPIO_PinModeSet\cf0 (port_MMA_SCL, 5, \cf6 gpioModeWiredAndPullUpFilter\cf0 , 1);\
	  \cf7 GPIO_PinModeSet\cf0 (port_MMA_SDA, 4, \cf6 gpioModeWiredAndPullUpFilter\cf0 , 1);\
\
\
	\cf3 for\cf0  (\cf3 int\cf0  i = 0; i < 9; i++)\
	  \{\
	    \cf2 /*\cf0 \
\cf2 	     * TBD: Seems to be clocking at \ul appr\ulnone  80kHz-120kHz depending on compiler\cf0 \
\cf2 	     * optimization when running at 14MHz. A bit high for standard mode devices,\cf0 \
\cf2 	     * but DVK only has fast mode devices. Need however to add some time\cf0 \
\cf2 	     * measurement in order to not be dependable on frequency and code executed.\cf0 \
\cf2 	     */\cf0 \
	    \cf7 GPIO_PinModeSet\cf0 (port_MMA_SCL, 5, \cf6 gpioModeWiredAnd\cf0 , 0);\
	    \cf7 GPIO_PinModeSet\cf0 (port_MMA_SCL, 5, \cf6 gpioModeWiredAnd\cf0 , 1);\
	  \}\
\
    I2C1->\cf6 ROUTE\cf0  = I2C_ROUTE_SDAPEN |										\cf2 // route pins used for I2c buses SDA and SCL\cf0 \
    	              I2C_ROUTE_SCLPEN |\
    	       (I2C_ROUTE_LOCATION_LOC0);\
\
    \cf3 if\cf0 (I2C1->\cf6 STATE\cf0  & I2C_STATE_BUSY)\{										\cf2 // reset registers and device\cf0 \
    	I2C1->\cf6 CMD\cf0  = I2C_CMD_ABORT ;\
    	\}\
\
    \cf3 int\cf0  fl = I2C1->\cf6 IF\cf0 ;\
        I2C1->\cf6 IFC\cf0  = fl;\
\
\
\
\}\
\
\
\cf3 void\cf0  performi2ctransfer_MMA(\cf3 void\cf0 )\
\{\
\
    I2C1->\cf6 IEN\cf0  = 0x1AFF;\
\
     \cf2 // data out rate\cf0 \
\
   writetoI2C(0x2A, 0x30);\
\
    \cf2 //  register scale set\cf0 \
\
   writetoI2C(0x0E, 0x01);\
\
   \cf2 // active mode\cf0 \
\
\
   writetoI2C(0x2A, 0x31);\
\
\}\
\
\
\
\
\
\
\
\cf3 void\cf0  writetoI2C(\cf3 int\cf0  reg_address, \cf3 int\cf0  data)\
\{\
	 I2C1->\cf6 CMD\cf0  = I2C_START;\
	 I2C1->\cf6 TXDATA\cf0  = 0x3A;\
	 \cf3 while\cf0 (! (I2C1->\cf6 IF\cf0  & I2C_IF_ACK));\
	 fl = (I2C1->\cf6 IF\cf0  & I2C_IF_ACK);\
	 I2C1->\cf6 IFC\cf0  = fl;\
\
	 I2C1->\cf6 TXDATA\cf0  = reg_address;\
	 \cf3 while\cf0 (! (I2C1->\cf6 IF\cf0  & I2C_IF_ACK));\
	 fl = (I2C1->\cf6 IF\cf0  & I2C_IF_ACK);\
     I2C1->\cf6 IFC\cf0  = fl;\
\
     I2C1->\cf6 TXDATA\cf0  = data;\
     \cf3 while\cf0 (! (I2C1->\cf6 IF\cf0  & I2C_IF_ACK));\
     fl = (I2C1->\cf6 IF\cf0  & I2C_IF_ACK);\
     I2C1->\cf6 IFC\cf0  = fl;\
\
     I2C1->\cf6 CMD\cf0  = I2C_STOP;\
\}\
\
\
\cf3 void\cf0  readfromI2C_MMA(\cf5 uint32_t\cf0  read_regaddress)\
\{\
\
	I2C1->\cf6 TXDATA\cf0  = 0x3A;\
	I2C1->\cf6 CMD\cf0  = I2C_START;\
	\cf3 while\cf0 (! (I2C1->\cf6 IF\cf0  & I2C_IF_ACK));\
	fl = (I2C1->\cf6 IF\cf0  & I2C_IF_ACK);\
	I2C1->\cf6 IFC\cf0  = fl;\
\
\
	I2C1->\cf6 TXDATA\cf0  = read_regaddress;\
	I2C1->\cf6 CMD\cf0  = I2C_START;\
	\cf3 while\cf0 (! (I2C1->\cf6 IF\cf0  & I2C_IF_ACK));\
	fl = (I2C1->\cf6 IF\cf0  & I2C_IF_ACK);\
	I2C1->\cf6 IFC\cf0  = fl;\
\
\
\
	I2C1->\cf6 TXDATA\cf0  = 0x3B;\
	\cf3 while\cf0 (! (I2C1->\cf6 IF\cf0  & I2C_IF_ACK));\
	fl = (I2C1->\cf6 IF\cf0  & I2C_IF_ACK);\
	I2C1->\cf6 IFC\cf0  = fl;\
\
\
\
	\cf3 while\cf0  (!(I2C1->\cf6 IF\cf0  & I2C_IF_RXDATAV));\
	i2c_txBuffer1 = I2C1->\cf6 RXDATA\cf0 ;\
	I2C1->\cf6 CMD\cf0  |= I2C_CMD_ACK;\
\
	\cf3 while\cf0  (!(I2C1->\cf6 IF\cf0  & I2C_IF_RXDATAV));\
	i2c_txBuffer2 = I2C1->\cf6 RXDATA\cf0 ;\
	I2C1->\cf6 CMD\cf0  |= I2C_CMD_ACK;\
\
	\cf3 while\cf0  (!(I2C1->\cf6 IF\cf0  & I2C_IF_RXDATAV));\
		i2c_txBuffer3 = I2C1->\cf6 RXDATA\cf0 ;\
		I2C1->\cf6 CMD\cf0  |= I2C_CMD_ACK;\
\
	\cf3 while\cf0  (!(I2C1->\cf6 IF\cf0  & I2C_IF_RXDATAV));\
		i2c_txBuffer4 = I2C1->\cf6 RXDATA\cf0 ;\
		I2C1->\cf6 CMD\cf0  |= I2C_CMD_ACK;\
\
	\cf3 while\cf0  (!(I2C1->\cf6 IF\cf0  & I2C_IF_RXDATAV));\
		i2c_txBuffer5 = I2C1->\cf6 RXDATA\cf0 ;\
		I2C1->\cf6 CMD\cf0  |= I2C_CMD_ACK;\
\
	  \cf3 while\cf0  (!(I2C1->\cf6 IF\cf0  & I2C_IF_RXDATAV));\
	  i2c_txBuffer6 = I2C1->\cf6 RXDATA\cf0 ;\
	  I2C1->\cf6 CMD\cf0  |= I2C_CMD_NACK;\
	  I2C1->\cf6 CMD\cf0  = I2C_STOP;\
\
\}\
\
\
\
\
\
\
\
\cf3 void\cf0  initLeuart(\cf3 void\cf0 )\
\{\
\
	\cf2 /* Start LFXO, and use LFXO for low-energy modules */\cf0 \
	 \cf2 // CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);\cf0 \
	  \cf7 CMU_ClockSelectSet\cf0 (\cf6 cmuClock_LFB\cf0 , \cf6 cmuSelect_LFRCO\cf0 );\
\
	  \cf2 /* Enabling clocks, all other remain disabled */\cf0 \
	  \cf7 CMU_ClockEnable\cf0 (cmuClock_CORELE, true);     \cf2 /* Enable CORELE clock */\cf0 \
\
	  \cf7 CMU_ClockEnable\cf0 (\cf6 cmuClock_GPIO\cf0 , true);       \cf2 /* Enable GPIO clock */\cf0 \
	  \cf7 CMU_ClockEnable\cf0 (\cf6 cmuClock_LEUART0\cf0 , true);    \cf2 /* Enable LEUART1 clock */\cf0 \
\
\
\cf2 /* Defining the LEUART0 initialization data */\cf0 \
\cf5 LEUART_Init_TypeDef\cf0  leuart0Init =\
\{\
  .enable   = \cf6 leuartEnableTx\cf0 ,       \cf2 /* Activate data reception on LEUn_TX pin. */\cf0 \
  .refFreq  = 0,                    \cf2 /* Inherit the clock \ul frequenzy\ulnone  from the LEUART clock source */\cf0 \
  .baudrate = 9600,                 \cf2 /* \ul Baudrate\ulnone  = 9600 \ul bps\ulnone  */\cf0 \
  .databits = \cf6 leuartDatabits8\cf0 ,      \cf2 /* Each LEUART frame \ul containes\ulnone  8 \ul databits\ulnone  */\cf0 \
  .parity   = \cf6 leuartNoParity\cf0 ,       \cf2 /* No parity bits in use */\cf0 \
  .stopbits = \cf6 leuartStopbits2\cf0 ,      \cf2 /* Setting the number of stop bits in a frame to 2 \ul bitperiods\ulnone  */\cf0 \
\};\
\
  \cf7 LEUART_Reset\cf0 (LEUART0);\
  \cf7 LEUART_Init\cf0 (LEUART0, &leuart0Init);\
  \cf3 while\cf0 ((LEUART0->\cf6 SYNCBUSY\cf0 ) != 0);\
\
  \cf2 /* Route LEUART0 TX pin to DMA location 0 */\cf0 \
  LEUART0->\cf6 ROUTE\cf0  = LEUART_ROUTE_TXPEN |\
                   LEUART_ROUTE_LOCATION_LOC0;\
\
  \cf2 /* Enable GPIO for LEUART0. TX is on D4 */\cf0 \
  \cf7 GPIO_PinModeSet\cf0 (\cf6 gpioPortD\cf0 ,                \cf2 /* GPIO port */\cf0 \
		          LEUART_tx_port,                        \cf2 /* GPIO port number */\cf0 \
                  \cf6 gpioModePushPull\cf0 ,         \cf2 /* Pin mode is set to push pull */\cf0 \
                  1);                       \cf2 /* High idle state */\cf0 \
\
  \cf2 /* Enable GPIO for LEUART0. RX is on D5 */\cf0 \
    \cf7 GPIO_PinModeSet\cf0 (\cf6 gpioPortD\cf0 ,                \cf2 /* GPIO port */\cf0 \
    		        LEUART_rx_port,                        \cf2 /* GPIO port number */\cf0 \
                    \cf6 gpioModePushPull\cf0 ,         \cf2 /* Pin mode is set to push pull */\cf0 \
                    1);\
\
    LEUART0->\cf6 CTRL\cf0  |= LEUART_LOOPBK;\
	    LEUART0->\cf6 CMD\cf0  |= LEUART_RXEN;\
\
    LEUART0->\cf6 IEN\cf0  = LEUART_TXBL;\
    blockSleepMode(EM2);\
\
    NVIC_EnableIRQ(\cf6 LEUART0_IRQn\cf0 );\
\
\}\
\
\cf2 /**************************************************************************/\cf0  \cf2 /**\cf0 \
\cf2   * DMA callBack routine\cf0 \
\cf2  ******************************************************************************* */\cf0 \
\cf3 void\cf0  ADCdmaTransferComplete(\cf3 unsigned\cf0  \cf3 int\cf0  channel, bool primary, \cf3 void\cf0  *user)\
\{\
  (\cf3 void\cf0 ) channel;\
  (\cf3 void\cf0 ) primary;\
  (\cf3 void\cf0 ) user;\
\
  ADC0->\cf6 CMD\cf0  |= adc_stop;                                                         \cf2 // Turn off ADC by writing 1 to stop bit in CMD register\cf0 \
  transferActive = false;\
  unblockSleepMode(EM1);                                                            \cf2 // Unblock previously blocked energy mode\cf0 \
\
  \cf3 int\cf0  sum = 0;\
\
  \cf3 for\cf0 (\cf3 int\cf0  i=0;i<ADCSAMPLES;i++)                                                     \cf2 //Calculate sum of data stored in location pointed by DstPtr\cf0 \
  \{\
	sum = sum + DstPtr[i];\
  \}\
\
  sum = sum/ADCSAMPLES;                                                              \cf2 // Calculate average\cf0 \
  average = convertToCelsius(sum);                                             \cf2 // Calculate average temperature in \ul celsius\cf0 \ulnone \
 str2 = &average;\
  tf[q]=putelement(str2,rb.\cf6 n\cf0 ,rb.\cf6 iput\cf0 ,str);\
  q++;\
\
\
  \cf3 float\cf0  rounded = ((\cf3 int\cf0 )(average*100 + .5)/100.0) ;\
  \cf2 //rounded = -1.33;\cf0 \
\
  \cf3 if\cf0 (rounded < 0)\
  \{\
	  sign = 0x6E;\
	  rounded = -1*rounded;\
  \}\
  \cf3 else\cf0  sign = 0x70;\
\
 \cf2 // \ul seperating\ulnone  the \ul int\ulnone  and float parts of the rounded value\cf0 \
\
  sumI = (\cf3 int\cf0 )rounded; \cf2 //sumI is the part of \ul int\cf0 \ulnone \
\
  sumF = (rounded - sumI)*100;\
\
\
 initLeuart();\
\
 unblockSleepMode(EM2);\
\
\
\
\
\
\
 \cf2 // turn on LED if temperature is outside \ul pre\ulnone -set limits\cf0 \
\
 \cf2 /* if( average<LowerTemp || average>UpperTemp)\cf0 \
\
\cf2   \{\cf0 \
\cf2     GPIO_PinOutSet(port_LED , LED_temperature_pin );\cf0 \
\cf2   \}\cf0 \
\
\cf2   else GPIO_PinOutClear(port_LED , LED_temperature_pin);*/\cf0 \
\
\}\
\
\cf2 /* This code is originally Silicon Labs and copyrighted by Silicon Labs in 2015 and Silicon Labs grants\cf0 \
\cf2  * permission to anyone to use the software for any purpose, including commercial applications, and to alter it,\cf0 \
\cf2  * and to redistribute it freely subject that the origin is not miss represented, altered source version must be\cf0 \
\cf2  * plainly marked, and this notice cannot be altered or removed from any source distribution.\cf0 \
\cf2  *\cf0 \
\cf2  * Routine include :\cf0 \
\cf2  *\cf0 \
\cf2  * float convertToCelsius\cf0 \
\cf2 */\cf0 \
\
\
\
\cf3 float\cf0  convertToCelsius(\cf5 int32_t\cf0  adcSample)\
\{\
  \cf3 float\cf0  temp;\
  \cf2 /* Factory calibration temperature from device information page. */\cf0 \
  \cf3 float\cf0  cal_temp_0 = (\cf3 float\cf0 )((DEVINFO->\cf6 CAL\cf0  & _DEVINFO_CAL_TEMP_MASK)\
                             >> _DEVINFO_CAL_TEMP_SHIFT);\
\
  \cf3 float\cf0  cal_value_0 = (\cf3 float\cf0 )((DEVINFO->\cf6 ADC0CAL2\cf0 \
                               & _DEVINFO_ADC0CAL2_TEMP1V25_MASK)\
                              >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);\
\
  \cf2 /* Temperature gradient (from \ul datasheet\ulnone ) */\cf0 \
  \cf3 float\cf0  t_grad = -6.27;\
\
  temp = (cal_temp_0 - ((cal_value_0 - adcSample)  / t_grad));\
\
  \cf3 return\cf0  temp;\
\}\
\
\
\cf2 /***************************************************************************/\cf0  \cf2 /**\cf0 \
\cf2   * DMA setup\cf0 \
\cf2  ******************************************************************************* */\cf0 \
\cf3 void\cf0  DMA_setup(\cf3 void\cf0 )\
\{\
    \cf2 // DMA \ul init\cf0 \ulnone \
\
	\cf5 DMA_Init_TypeDef\cf0  dma_init ;\
\
\
	dma_init.\cf6 hprot\cf0         =  0;\
    dma_init.\cf6 controlBlock\cf0  = dmaControlBlock;\
\
    \cf7 CMU_ClockEnable\cf0 (\cf6 cmuClock_HFPER\cf0 , true);\
    \cf7 CMU_ClockEnable\cf0 (\cf6 cmuClock_DMA\cf0 , true);\
\
    \cf7 DMA_Init\cf0 (&dma_init);\
\
\
    \cf2 // DMA channel configuration\cf0 \
\
    \cf5 DMA_CfgChannel_TypeDef\cf0   chnl_Cfg;\
\
    chnl_Cfg.\cf6 highPri\cf0    = false;\
    chnl_Cfg.\cf6 enableInt\cf0  = true;\
    chnl_Cfg.\cf6 select\cf0     = DMAREQ_ADC0_SINGLE;\
    chnl_Cfg.\cf6 cb\cf0         = &cb;\
    \cf7 DMA_CfgChannel\cf0  (DMA_CHANNEL_ADC, &chnl_Cfg);\
\
\
    \cf2 // DMA Descriptor configuration\cf0 \
    \cf5 DMA_CfgDescr_TypeDef\cf0   descrCfg;\
\
      descrCfg.\cf6 dstInc\cf0   = \cf6 dmaDataInc2\cf0 ;\
      descrCfg.\cf6 srcInc\cf0   = \cf6 dmaDataIncNone\cf0 ;\
      descrCfg.\cf6 size\cf0     = \cf6 dmaDataSize2\cf0 ;\
      descrCfg.\cf6 arbRate\cf0  = \cf6 dmaArbitrate1\cf0 ;\
      descrCfg.\cf6 hprot\cf0    = 0;\
      \cf7 DMA_CfgDescr\cf0 (DMA_CHANNEL_ADC, true, &descrCfg);\
\
\
      \cf2 // call back function setup\cf0 \
      cb.\cf6 cbFunc\cf0   = ADCdmaTransferComplete;\
      cb.\cf6 userPtr\cf0  = NULL;\
      cb.\cf6 primary\cf0  = true;\
\
      DMA->\cf6 IFC\cf0  |= 0x01;                               \cf2 // write 1 to clear DMA channel 0 previous interrupts.\cf0 \
      DMA->\cf6 IEN\cf0  |= 0x01;                               \cf2 // write 1 to enable DMA channel 0 interrupt.\cf0 \
\
\
\
  \}\
\
\cf2 // Activate function for DMA\cf0 \
\
\cf3 void\cf0  DMA_ACTIVATE()\
\{\
\
	\cf7 DMA_ActivateBasic\cf0 (DMA_CHANNEL_ADC,\
	                    true,\
	                    false,\
	                    (\cf3 void\cf0  *)DstPtr,\
	                    (\cf3 void\cf0  *)&(ADC0->\cf6 SINGLEDATA\cf0 ),\
	                    ADCSAMPLES - 1);\
\}\
\
\
\
\
\cf2 /***************************************************************************/\cf0  \cf2 /**\cf0 \
\cf2   * ADC initialization\cf0 \
\cf2  ******************************************************************************* */\cf0 \
\
\cf3 void\cf0  setupAdc(\cf3 void\cf0 )\
\{\
  \cf5 ADC_Init_TypeDef\cf0         adcInit       = ADC_INIT_DEFAULT;\
  \cf5 ADC_InitSingle_TypeDef\cf0   adcInitSingle = ADC_INITSINGLE_DEFAULT;\
\
\
  adcInit.\cf6 timebase\cf0  = \cf7 ADC_TimebaseCalc\cf0 (0);\
  adcInit.\cf6 prescale\cf0  = \cf7 ADC_PrescaleCalc\cf0 (2000000, 0);                     \cf2 //Calculate \ul prescalar\ulnone  for desired value \ul Tconv\ulnone  = (N+\ul Ta\ulnone )*P/14M\cf0 \
  \cf7 ADC_Init\cf0 (ADC0, &adcInit);\
\
  adcInitSingle.\cf6 input\cf0      =  adcSingleInpTemp;                         \cf2 /* Select input to ADC */\cf0 \
  adcInitSingle.\cf6 reference\cf0     = \cf6 adcRef1V25\cf0 ;                             \cf2 /* Reference voltage */\cf0 \
  adcInitSingle.\cf6 rep\cf0       = true;\
  adcInitSingle.\cf6 acqTime\cf0 	  = \cf6 adcAcqTime8\cf0 ;\
  \cf7 ADC_InitSingle\cf0 (ADC0, &adcInitSingle);\
\
\}\
\
\
\
\cf2 /***************************************************************************/\cf0  \cf2 /**\cf0 \
\cf2   * ACMP initialization\cf0 \
\cf2  ******************************************************************************* */\cf0 \
\cf3 void\cf0  ACMPInit(\cf3 void\cf0 )\
\{\
	  \cf3 const\cf0  \cf5 ACMP_Init_TypeDef\cf0  acmp_init =\
	  \{\
	    false,                              \cf2 /* Full bias current*/\cf0 \
	    true,                              \cf2 /* Half bias current */\cf0 \
	    7,                                  \cf2 /* \ul Biasprog\ulnone  current configuration */\cf0 \
	    false,                               \cf2 /* Enable interrupt for falling edge */\cf0 \
	    true,                               \cf2 /* Enable interrupt for rising edge */\cf0 \
	    \cf6 acmpWarmTime256\cf0 ,                    \cf2 /* Warm-up time in clock cycles, >140 cycles for 10us with 14MHz */\cf0 \
	    \cf6 acmpHysteresisLevel0\cf0 ,               \cf2 /* Hysteresis configuration */\cf0 \
	    0,                                  \cf2 /* Inactive comparator output value */\cf0 \
	    false,                              \cf2 /* Enable low power mode */\cf0 \
	    0x30,                                  \cf2 /* \ul Vdd\ulnone  reference scaling */\cf0 \
	    false,                               \cf2 /* Enable ACMP */\cf0 \
	  \};\
\
	  \cf7 ACMP_Init\cf0 (ACMP0, &acmp_init);\
	  \cf7 ACMP_ChannelSet\cf0 (ACMP0, \cf6 acmpChannel2V5\cf0 , \cf6 acmpChannel0\cf0 );\
	  ACMP0->\cf6 IEN\cf0  |= ACMP_IEN_EDGE;\
\
	 	NVIC_EnableIRQ(\cf6 ACMP0_IRQn\cf0 );\
\
 \}\
\
\
\
\cf3 void\cf0  LETIMER0_Calibration(\cf3 void\cf0 )\
  \{\
\
	  \cf5 TIMER_Init_TypeDef\cf0  timer0 =\
	     \{\
	       .enable     = false,                                 \cf2 // start counting after TIMER_Init is completed\cf0 \
	       .debugRun   = false,                                 \cf2 // Counter shall keep counting during debug halt\cf0 \
	       .prescale   = \cf6 timerPrescale1\cf0 ,\
	       .clkSel     = \cf6 timerClkSelHFPerClk\cf0 ,\
	       .fallAction = \cf6 timerInputActionNone\cf0 ,\
	       .riseAction = \cf6 timerInputActionNone\cf0 ,\
	       .mode       = \cf6 timerModeUp\cf0 ,\
	       .dmaClrAct  = false,\
	       .quadModeX4 = false,\
	       .oneShot    = false,\
	       .sync       = false,\
	     \};\
\
	   \cf5 TIMER_Init_TypeDef\cf0  timer1 =\
	     \{\
	       .enable     = false,                                 \cf2 // start counting after TIMER_Init is completed\cf0 \
	       .debugRun   = false,                                 \cf2 // Counter shall keep counting during debug halt\cf0 \
	       .prescale   = \cf6 timerPrescale1\cf0 ,\
	       .clkSel     = \cf6 timerClkSelCascade\cf0 ,\
	       .fallAction = \cf6 timerInputActionNone\cf0 ,\
	       .riseAction = \cf6 timerInputActionNone\cf0 ,\
	       .mode       = \cf6 timerModeUp\cf0 ,\
	       .dmaClrAct  = false,\
	       .quadModeX4 = false,\
	       .oneShot    = false,\
	       .sync       = true,\
	     \};\
\
	  \cf2 // Setup both the timers\cf0 \
\
	  \cf7 TIMER_Init\cf0 (TIMER0, &timer0);\
  	  \cf7 TIMER_Init\cf0 (TIMER1, &timer1);\
\
  	  \cf2 // Initialize both count values to 0\cf0 \
  	  TIMER0->\cf6 CNT\cf0  = 0x0000;\
  	  TIMER1->\cf6 CNT\cf0  = 0x0000;\
\
  	  TIMER0->\cf6 CMD\cf0  = timer_stop;\
\
\
  		\cf5 LETIMER_Init_TypeDef\cf0  LETIMER0_Calinit;\
  	    LETIMER0_Calinit.\cf6 bufTop\cf0  = false;                       \cf2 // do not load COMP! into COMP0 when REP0 = 0\cf0 \
  		LETIMER0_Calinit.\cf6 comp0Top\cf0  = true;                      \cf2 // load COMP0 into CNT on underflow\cf0 \
  		LETIMER0_Calinit.\cf6 debugRun\cf0  = false;                     \cf2 // stop counting when debug stops\cf0 \
  		LETIMER0_Calinit.\cf6 enable\cf0  = false;\
  		LETIMER0_Calinit.\cf6 out0Pol\cf0  = 0;                          \cf2 // idle output 0\cf0 \
  		LETIMER0_Calinit.\cf6 out1Pol\cf0  = 1;\
  		LETIMER0_Calinit.\cf6 repMode\cf0  = \cf6 letimerRepeatOneshot\cf0 ;        \cf2 // count until stopped by software\cf0 \
  		LETIMER0_Calinit.\cf6 rtcComp0Enable\cf0  = false;                \cf2 // do not start counting on COMP0 RTC match\cf0 \
  		LETIMER0_Calinit.\cf6 rtcComp1Enable\cf0  = false;                \cf2 // do not start counting on COMP1 RTC match\cf0 \
        LETIMER0_Calinit.\cf6 ufoa0\cf0  = \cf6 letimerUFOANone\cf0 ;		        \cf2 // underflow output actions\cf0 \
  		LETIMER0_Calinit.\cf6 ufoa1\cf0  = \cf6 letimerUFOANone\cf0 ;\
\
  		\cf7 CMU_ClockSelectSet\cf0 (\cf6 cmuClock_LFA\cf0 , \cf6 cmuSelect_LFXO\cf0 );\
  		\cf7 CMU_OscillatorEnable\cf0 (\cf6 cmuOsc_LFXO\cf0 , true, true);\
  		\cf7 CMU_ClockEnable\cf0 (\cf6 cmuClock_LETIMER0\cf0 , true);\
\
  		\cf7 LETIMER_Init\cf0 (LETIMER0, &LETIMER0_Calinit);\
  		LETIMER0->\cf6 CNT\cf0  = LETIMER0_LFXO_count;\
\
  		\cf7 LETIMER_Enable\cf0 (LETIMER0, true);\
\
  		TIMER0->\cf6 CMD\cf0  = timer_start;\
  		TIMER1->\cf6 CMD\cf0  = timer_start;\
  		\cf3 while\cf0  ( LETIMER0->\cf6 CNT\cf0  != 0 );\
\
  			TIMER0->\cf6 CMD\cf0  = timer_stop;\
  			TIMER1->\cf6 CMD\cf0  = timer_stop;\
  		\cf3 int\cf0  \cf3 long\cf0  a,LFXO_Count ;\
  		\cf3 int\cf0   b;\
  		a = TIMER1->\cf6 CNT\cf0 ;\
  		b = TIMER0->\cf6 CNT\cf0 ;\
\
  		LFXO_Count = a <<16| b;\
\
\
\
  	\cf2 // ULFRCO\cf0 \
\
  	TIMER0->\cf6 CNT\cf0  = 0x0000;\
  	TIMER1->\cf6 CNT\cf0  = 0x0000;\
\
    TIMER0->\cf6 CMD\cf0  = timer_stop;\
\
\
\
\
  		  \cf7 LETIMER_Init\cf0 (LETIMER0, &LETIMER0_Calinit);\
\
  		  \cf7 CMU_ClockSelectSet\cf0 (\cf6 cmuClock_LFA\cf0 , \cf6 cmuSelect_ULFRCO\cf0 );\
  		  \cf7 CMU_OscillatorEnable\cf0 (\cf6 cmuOsc_LFXO\cf0 , false, false);\
  		  \cf7 CMU_ClockEnable\cf0 (\cf6 cmuClock_LETIMER0\cf0 , true);\
  		  LETIMER0->\cf6 CNT\cf0 =LETIMER0_ULFRCO_count;\
  		  \cf7 LETIMER_Enable\cf0 (LETIMER0, true);\
\
  		  	  		TIMER0->\cf6 CMD\cf0  = timer_start;\
  		  	  	TIMER1->\cf6 CMD\cf0  = timer_start;\
  		  	  		\cf3 while\cf0  ( LETIMER0->\cf6 CNT\cf0  != 0 ) ;\
\
  		  	  		TIMER0->\cf6 CMD\cf0  = timer_stop;\
  		  	  	    TIMER1->\cf6 CMD\cf0  = timer_stop;\
\
  		  	  		\cf3 int\cf0  \cf3 long\cf0  ULFRCO_Count ;\
\
  		  	  		a = TIMER1->\cf6 CNT\cf0 ;\
  		  	  		b = TIMER0->\cf6 CNT\cf0 ;\
\
  		  	  		ULFRCO_Count = a<<16 | b;\
\
\
  		  	  	\cf3 float\cf0  osc_ratio;\
  		  	  	osc_ratio = LFXO_Count/(\cf3 float\cf0 )ULFRCO_Count;\
\
\
\
  		  	  	ULFRCO_Calibrated = osc_ratio*LETIMER0_ULFRCO_count;\
\
\
  \}\
\
\
\
\
\cf3 void\cf0  LETIMER0_Setup(\cf3 void\cf0 )\{\
\
   	\cf5 LETIMER_Init_TypeDef\cf0  LETIMER0_init;\
\
\
\
   	\cf3 int\cf0  Comp0_init;\
   	\cf3 int\cf0  Comp1_int_init;\
   	\cf3 float\cf0  Comp1_float_init;\
   	\cf3 int\cf0  LETIMER0_prescalar;\
   	\cf2 //\ul int\ulnone  ULFRCO_count_calibrated;\cf0 \
\
   	\cf2 // LETIMER initialization\cf0 \
   	LETIMER0_init.\cf6 bufTop\cf0  = false;                 \cf2 // do not load COMP! into COMP0 when REP0 = 0\cf0 \
   	LETIMER0_init.\cf6 comp0Top\cf0  = true;                \cf2 // load COMP0 into CNT on underflow\cf0 \
   	LETIMER0_init.\cf6 debugRun\cf0  = false;               \cf2 // stop counting when debug stops\cf0 \
   	LETIMER0_init.\cf6 enable\cf0  = false;\
   	LETIMER0_init.\cf6 out0Pol\cf0  = 0;                    \cf2 // idle output 0\cf0 \
   	LETIMER0_init.\cf6 out1Pol\cf0  = 1;\
   	LETIMER0_init.\cf6 repMode\cf0  = \cf6 letimerRepeatFree\cf0 ;    \cf2 // count until stopped by software\cf0 \
   	LETIMER0_init.\cf6 rtcComp0Enable\cf0  = false;        \cf2 // do not start counting on COMP0 RTC match\cf0 \
   	LETIMER0_init.\cf6 rtcComp1Enable\cf0  = false;       \cf2 // do not start counting on COMP1 RTC match\cf0 \
   	\cf2 // underflow output actions\cf0 \
   	LETIMER0_init.\cf6 ufoa0\cf0  = \cf6 letimerUFOANone\cf0 ;\
   	LETIMER0_init.\cf6 ufoa1\cf0  = \cf6 letimerUFOANone\cf0 ;\
\
\
       \cf2 // initialize period in ULFRCO for EM3 and LFXO for all other energy modes\cf0 \
   	\cf7 LETIMER_Init\cf0 (LETIMER0, &LETIMER0_init);\
\
   	\cf3 if\cf0  (LETIMER0_EM_Mode == EM3)\
   	\{\
   		\cf3 if\cf0 (CALIBRATION == 1)\{\
\
\
\
   				Comp0_init = LETIMER0_period * ULFRCO_Calibrated ;\
   		\}\
\
   		\cf3 else\cf0  Comp0_init = LETIMER0_period * LETIMER0_ULFRCO_count ;\
\
   	\}\
   	\cf3 else\cf0  \{\
   		LETIMER0_prescalar = LETIMER0_period/2;\
   		Comp0_init = LETIMER0_period * LETIMER0_LFXO_count;\
   		CMU->\cf6 LFAPRESC0\cf0  &= 0xfffff0ff;\
   		CMU->\cf6 LFAPRESC0\cf0  |= LETIMER0_prescalar <<8 ;\
   		LETIMER0_prescalar = 1 << LETIMER0_prescalar;\
\
   		Comp0_init = LETIMER0_period * (LETIMER0_LFXO_count /\
\
   		LETIMER0_prescalar);\
\
   		\}\
\
   		LETIMER0->\cf6 CNT\cf0  = Comp0_init;\
\
   		\cf7 LETIMER_CompareSet\cf0 (LETIMER0,0,Comp0_init);\
\
\
\
   	\cf2 //load comp1 register with a count to generate time in \ul ms\cf0 \ulnone \
   	\cf2 //load comp0 register with a count to initialize \ul cnt\cf0 \ulnone \
\
\
   	\cf2 // initialize on time in ULFRCO for EM3 and LFXO for all other energy modes\cf0 \
   	\cf3 if\cf0  (LETIMER0_EM_Mode == EM3)\
   	\{\
\
   		\cf3 if\cf0 (CALIBRATION == 1)\{\
\
   	     Comp1_int_init = LETIMER0_LEDontime * ULFRCO_Calibrated;\
   	     Comp1_float_init = LETIMER0_LEDontime * ULFRCO_Calibrated;\
\
   	     \cf3 if\cf0 (Comp1_float_init > Comp1_int_init)                            \cf2 /* Compare \ul int\ulnone  \ul nd\ulnone  float values of the multiplication for minimum */\cf0 \
   	     \{                                                                 \cf2 /*  accurate excite time of 4ms*/\cf0 \
   		  Comp1_int_init++;\
   	     \}\
\
   	  \cf3 else\cf0 \{\
   		  Comp1_int_init +=0;\
   	      \}\
\
  \}\
\
   		\cf3 else\cf0 \
   		\{\
   		Comp1_int_init = LETIMER0_LEDontime * LETIMER0_ULFRCO_count ;\
\
   		\}\
   \}\
\
\
   	\cf3 else\cf0 \
   		Comp1_int_init = LETIMER0_LEDontime * (LETIMER0_LFXO_count/LETIMER0_prescalar);\
\
   	\cf7 LETIMER_CompareSet\cf0 (LETIMER0,1,Comp1_int_init);                \cf2 // set compare register value ( pointer , compare register 0 to set, initialization value)\cf0 \
\
\
   	\cf3 while\cf0 ((LETIMER0->\cf6 SYNCBUSY\cf0 ) != 0);                             \cf2 //wait till synchronization bit goes low\cf0 \
\
   	\cf2 // setting corresponding flag bits in LETIMER_IEN\cf0 \
   	LETIMER0->\cf6 IEN\cf0  = LETIMER_IEN_UF\
   			| LETIMER_IEN_COMP1;\
\
   	blockSleepMode(LETIMER0_EM_Mode);\
\
   	NVIC_EnableIRQ(\cf6 LETIMER0_IRQn\cf0 );\
\
   \}\
\
\
\
\cf2 // circular buffer functions\cf0 \
\
\
 	 \cf5 uint8_t\cf0  emptycheck(\cf5 uint8_t\cf0  n)\
 	     \{    \cf3 if\cf0  (n>0)\
 	         \cf3 return\cf0  0;\
 	         \cf3 else\cf0 \
 	         \cf3 return\cf0  1;\
 	     \}\
\
 	 \cf5 uint8_t\cf0  fullcheck(\cf5 uint8_t\cf0  n)\
 	     \{    \cf3 if\cf0  (n<size_circularbuffer)\
 	         \cf3 return\cf0  0;\
 	         \cf3 else\cf0 \
 	         \cf3 return\cf0  1;\
 	     \}\
\
 	 \cf5 uint8_t\cf0  ringlocate(\cf5 uint8_t\cf0  i)\
 	     \{\
 	         \cf3 return\cf0 (i+1)==size_circularbuffer?0:i+1;\
 	     \}\
\
 	 \cf5 uint8_t\cf0  getelement(\cf5 uint8_t\cf0  n,\cf5 uint8_t\cf0  current_loc, \cf5 uint8_t\cf0  *str)\
 	     \{\
 	         \cf2 //uint8_t \ul pos\ulnone ;\cf0 \
 	         \cf5 uint8_t\cf0  check;\
 	         check=emptycheck(n);\
 	         \cf3 if\cf0 (check==1)\
 	         \{   \cf3 int\cf0  ii=0;\
 	             \cf3 while\cf0  (warning2[ii] != \cf4 '\\0'\cf0 )\{\
 	             \cf7 printf\cf0 (\cf4 "%c"\cf0 ,warning2[ii]);\
 	             ii++;\
 	             \}\
 	             \cf3 return\cf0  1;\
 	         \}\
 	         \cf3 else\cf0  \cf3 if\cf0 (st[current_loc]==1)\
 	         \{\
 	             rb.\cf6 iget\cf0 =ringlocate(current_loc);                                                          \cf2 //update rb.iget\cf0 \
 	             rb.\cf6 n\cf0 --;\
 	             des[p]=*(str+current_loc);\
 	             p++;\
 	             *(str+current_loc)=\cf4 ' '\cf0 ;\
 	             st[current_loc]=0;\
 	             \cf3 return\cf0  0;\
 	         \}\
 	         \cf3 else\cf0 \
 	         \{   \cf3 int\cf0  ii=0;\
 	             \cf3 while\cf0  (warning4[ii] != \cf4 '\\0'\cf0 )\{\
 	             \cf7 printf\cf0 (\cf4 "%c"\cf0 ,warning4[ii]);\
 	             ii++;\
 	             \}\
 	 \cf3 return\cf0  1;\
 	         \}\
 	     \}\
\
 	 \cf5 uint8_t\cf0  putelement(\cf5 uint8_t\cf0  *zz, \cf5 uint8_t\cf0  n, \cf5 uint8_t\cf0  current_loc,\cf5 uint8_t\cf0  *str)\
 	     \{\
 	         \cf5 uint8_t\cf0  check;\
 	         check=fullcheck(n);\
 	         \cf3 if\cf0 (check==1)\
 	         \{   \cf3 int\cf0  ii=0;\
 	             \cf3 while\cf0  (warning1[ii] != \cf4 '\\0'\cf0 )\{\
 	             \cf7 printf\cf0 (\cf4 "%c"\cf0 ,warning1[ii]);\
 	             ii++;\
 	             \}\
 	         \cf3 return\cf0  1;\
 	         \}\
 	         \cf3 else\cf0  \cf3 if\cf0 (st[current_loc]!=1)\
 	             \{\
 	             *(str+current_loc)=*zz;\
\
 	             rb.\cf6 iput\cf0 =ringlocate(current_loc);\
 	             rb.\cf6 n\cf0 ++;\
 	             st[current_loc]=1;\
 	             \cf3 return\cf0  0;\
\
\
 	             \}\
 	             \cf3 else\cf0 \
 	             \{\
 	                 \cf3 int\cf0  ii=0;\
 	                             \cf3 while\cf0  (warning3[ii] != \cf4 '\\0'\cf0 )\{\
 	                             \cf7 printf\cf0 (\cf4 "%c"\cf0 ,warning3[ii]);\
 	                             ii++;\}\
 	                             \cf3 return\cf0  1;\
 	             \}\
 	        \}\
\
 	 \cf3 void\cf0  test(\cf3 int\cf0  n)\
 	 \{\
 	     \cf3 if\cf0  (n!=0)\
 	                         \{\cf3 int\cf0  kk=0;\
 	                         \cf3 while\cf0  (fail[kk] != \cf4 '\\0'\cf0 )\{\
 	                                                                                 \cf7 printf\cf0 (\cf4 "%c"\cf0 ,fail[kk]);\
 	                                                 kk++;\
 	                                                 \}\}\
 	                         \cf3 else\cf0 \
 	                         \{\cf3 int\cf0  kk=0;\
 	                         \cf3 while\cf0  (pass[kk] != \cf4 '\\0'\cf0 )\{\
 	                                                 \cf7 printf\cf0 (\cf4 "%c"\cf0 ,pass[kk]);\
 	                                                 kk++;\
 	                                                 \}\}\
 	 \}\
\
\
\
\
\
\
\
\
\
\
  \cf2 /**************************************************************************//**\cf0 \
\cf2    * @brief ACMP0 Interrupt handler\cf0 \
\cf2    *****************************************************************************/\cf0 \
  \cf3 void\cf0  ACMP0_IRQHandler(\cf3 void\cf0 )\
  \{\
    \cf2 /* Clear interrupt flag */\cf0 \
	    ACMP0->\cf6 IFC\cf0  = ACMP_IFC_EDGE;\
\
		count = count + 1;\cf2 //For a high ACMP output turn off LED\cf0 \
\
\
  \}\
\
  \cf2 /**************************************************************************//**\cf0 \
\cf2        * @brief LEUART Interrupt handler\cf0 \
\cf2        *****************************************************************************/\cf0 \
  \cf3 void\cf0  LEUART0_IRQHandler(\cf3 void\cf0 )\
  \{\
\
	\cf2 // blockSleepMode(EM2);\cf0 \
\
  	    \cf3 int\cf0  flags = LEUART0->\cf6 IF\cf0 ;\
  	    LEUART0->\cf6 IFC\cf0  = flags;\
\
        tf[kk]=getelement(rb.\cf6 n\cf0 ,rb.\cf6 iget\cf0 ,str);\
\
  	    LEUART0->\cf6 TXDATA\cf0  = des[t];\
\
  	    \cf3 while\cf0  (!(LEUART0->\cf6 IF\cf0  & LEUART_IF_TXC));\
\
  	    \cf3 if\cf0  ( t<size_circularbuffer )\
  	    \{\
  	    	t++;\
  	    \}\
  	    \cf3 else\cf0  t=0;\
\
\
  		NVIC_DisableIRQ(\cf6 LEUART0_IRQn\cf0 );\
\
\
\
\
  	\cf2 // unblockSleepMode(EM2);\cf0 \
\
\
\
\
  \}\
\
  \cf2 /**************************************************************************//**\cf0 \
\cf2      * @brief LETIMER0 Interrupt handler\cf0 \
\cf2      *****************************************************************************/\cf0 \
\cf3 void\cf0  LETIMER0_IRQHandler(\cf3 void\cf0 )\{\
	\cf3 int\cf0  currentFlags;\
	currentFlags = LETIMER0->\cf6 IF\cf0 ;                                                          \cf2 // save IF register contents/state in a variable\cf0 \
	LETIMER0->\cf6 IFC\cf0  = currentFlags;                                                         \cf2 // Interrupt flag clear register\cf0 \
\
\
\
		\cf2 /* ACMP output check */\cf0 \
	\cf2 // depending upon the flag set, change LED functions for each of the interrupts\cf0 \
	\cf3 if\cf0  ((currentFlags & LETIMER_IF_COMP1) != 0)\
	\{\
		\cf3 if\cf0 ( Interval == 0)\{\
			powerup();\
		\}\
\
\}\
	\cf3 else\cf0  \cf3 if\cf0  ((currentFlags & LETIMER_IF_UF) != 0)\
	\{\
\
\
\
		timer_count++;\
\
		\cf3 if\cf0 (timer_count == 2)\
		\{\
  		heartbeat_count = 72; \cf2 //5*count;\cf0 \
  		timer_count = 0;\cf2 // Turn off ACMP\cf0 \
  		str1 = &heartbeat_count;\
\
        tf[q]=putelement(str1,rb.\cf6 n\cf0 ,rb.\cf6 iput\cf0 ,str);\
        q++;\
\
		\}\
\
\
\
		\cf2 /* DMA routine start here*/\cf0 \
\
		\cf3 if\cf0 ( withDMA == 1)                                                    \cf2 // Activate DMA here if used\cf0 \
		\{\
		DMA_ACTIVATE();\
		\}\
\
\
		\cf2 /*Non DMA routine starts here*/\cf0 \
\
		blockSleepMode(EM1);                                                 \cf2 // Block energy mode for ADC and DAC\cf0 \
\
		ADC0->\cf6 CMD\cf0  |= adc_start;                                                  \cf2 // Enable ADC\cf0 \
\
		\cf3 if\cf0 ( withDMA != 1)                                                   \cf2 // Follow the routine for no DMA\cf0 \
		\{\
		\cf3 for\cf0 (\cf3 int\cf0  j=0;j<ADCSAMPLES;j++)                                       \cf2 // Take each ADC conversion data and store in memory\cf0 \
		\{\
			\cf3 while\cf0  (!(ADC0->\cf6 STATUS\cf0  & ADC_STATUS_SINGLEDV));\
			Dstbuffer[j] = ADC0->\cf6 SINGLEDATA\cf0 ;\
\
		\}\
\
		ADC0->\cf6 CMD\cf0  |= adc_stop;                                           \cf2 // Disable ADC\cf0 \
		  transferActive = false;                                           \cf2 // Update user defined flag\cf0 \
		  unblockSleepMode(EM1);                                            \cf2 // Unblock sleep mode\cf0 \
\
\
\cf2 /* Routine to find the average \ul temperation\ulnone  in \ul celsius\ulnone  and change LED state accordingly */\cf0 \
\
		\cf3 int\cf0  sumtemp = 0;\
\
		  \cf3 for\cf0 (\cf3 int\cf0  i=0;i<ADCSAMPLES;i++)\
		  \{\
			sumtemp = sumtemp + Dstbuffer[i];\
\ul 		  \}\ulnone \
\
		  sumtemp = sumtemp/ADCSAMPLES;\
		  \cf3 float\cf0  average_nodma;\
		  average_nodma = convertToCelsius(sumtemp);\
\
		  \cf3 if\cf0 ( average_nodma<LowerTemp || average_nodma>UpperTemp)\
\
		  \{\
		    GPIO_PinOutSet(port_LED , LED_temperature_pin);\
		  \}\
\
		  \cf3 else\cf0  GPIO_PinOutClear(port_LED , LED_temperature_pin);\
\
		\}\
\
\
\
		\cf3 if\cf0 ( Interval == 0)\
		\{\
			\cf2 // \ul powerup\ulnone ();\cf0 \
			I2CSetup_MMA();												\cf2 // do i2c setup, configure TSL2561 registers and monitor if interrupt occurs\cf0 \
			performi2ctransfer_MMA();\
			I2CSetup_TSL();												\cf2 // do i2c setup, configure TSL2561 registers and monitor if interrupt occurs\cf0 \
			performI2CTransferAndMonitor_TSL();\
			Interval++;\
		\}\
\
\
		\cf3 else\cf0  \cf3 if\cf0 ( Interval == 1 )\
		\{\
	    	readfromI2C_MMA(0x01);\
\
	    	\cf3 if\cf0 (i2c_txBuffer5 >= 220)\
	    	\{\
          	  GPIO_PinOutSet(port_LED, 3);\
	    	\}\
\
	    	\cf3 else\cf0   GPIO_PinOutClear(port_LED, 3);\
\
	    Interval++;\
\
		\}\
\
\
\
		\cf3 else\cf0  \cf3 if\cf0 ( Interval == 2)\
		\{\
			powerdown();\
			Interval = 0;											\cf2 // enable power down routine\cf0 \
			\cf2 //unblockSleepMode(EM1);\cf0 \
\
		\}\
\
		\}\
\
\
\}\
\
\
\
\
\
\
 \cf3 int\cf0  main(\cf3 void\cf0 )\{\
	\cf2 //\ul int\ulnone  ULFRCO_Calibrated;\cf0 \
\
	CHIP_Init(); \cf2 // initialize all \ul errata\cf0 \ulnone \
\
	   str=(\cf3 char\cf0 *)\cf7 malloc\cf0 (size_circularbuffer);\
	   str = circularbuffer;\
\
\
	blockSleepMode(EM3);\
\
\
	CMU_Setup();\
	ACMPInit();\
	GPIO_Setup();\
\
	\cf3 if\cf0 ( withDMA == 1)\{\
    DMA_setup();\
	\}\
    setupAdc();\
\
	LETIMER0_Setup();\
\
	\cf7 LETIMER_Enable\cf0 (LETIMER0, true);\
\
	ACMP0->\cf6 CTRL\cf0  |= ACMP_CTRL_EN;\
\
  \cf2 /* Wait for \ul warmup\ulnone  */\cf0 \
     \cf3 while\cf0  (!(ACMP0->\cf6 STATUS\cf0  & ACMP_STATUS_ACMPACT)) ;\
\
\
\
\
\
	\cf3 while\cf0 (1)\{\
		Sleep(); \cf2 // call sleep routine\cf0 \
\
	 \cf3 while\cf0 (1)\
				  \{\
				    \cf2 //INT_Disable();\cf0 \
				    \cf3 if\cf0  ( transferActive )\
				      \{\
				      Sleep();\
				      \}\
				  \cf2 //  INT_Enable();\cf0 \
\
				    \cf2 /* Exit the loop if transfer has completed */\cf0 \
				    \cf3 if\cf0  ( !transferActive )\
				    \{\
				      \cf3 break\cf0 ;\
				    \}\
				  \}\
\
	\}\
\}\
\
\
\
\
}