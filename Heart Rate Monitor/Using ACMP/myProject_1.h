#ifndef __INIT_DEVICE_H__
#define __INIT_DEVICE_H__

#define EM0 0
#define EM1 1
#define EM2 2
#define EM3 3
#define EM4 4

#define ADCSAMPLES                        500
#define ADCSAMPLESPERSEC                  100000

#define LETIMER0_period 		    30                 // LED period
#define LETIMER0_LEDontime	        2              // excitation time
#define LETIMER0_cmp0ontime	        0.03             // excitation time
#define LETIMER0_LFXO_count			32768             // clock frequency / 2
#define LETIMER0_ULFRCO_count 		1000              // used in  case of EM3
#define LETIMER0_EM_Mode			EM3               // define the energy mode to be used
#define CALIBRATION                 1                 // Run calib routine if this variable is 1


#define Light_Sensor_ACMP_Channel    acmpChannel6     // Define channel for positive input to ACMP
#define Light_Sensor_ACMP_Ref        acmpChannelVDD   // Define the negative reference input to ACMP
#define Light_Sensor_Darkness_Ref     (0x02 << 8)     // Compare value for ACMP input to give positive output and LED on
#define Light_Sensor_Light_Ref        (0x3D << 8)     // Compare value for ACMP input to give negative output and LED off
#define ACMP_retain_particular_bits  0xFFFFC0FF
#define UseACMProutines               0



#define LowerTemp                     05              // Lower limit temperature
#define UpperTemp                     55              // Upper limit temperature
#define withDMA                       1               // Constant to select or unselect DMA
#define DMA_CHANNEL_ADC               0               // Define channel used for DMA transfers.
#define port_ADC_excite				gpioPortD		  // Define excite port for ADC
#define adc_start					0x01
#define adc_stop                    0x01 << 1
#define port_internal_tempsensor	gpioPortC		 // input port for onboard temperature sensor

#define port_LED                    gpioPortE

// i2c ports
#define port_i2c_power				gpioPortD
#define port_i2c_interrupt			gpioPortD
#define port_i2c_SDA				gpioPortC
#define port_i2c_SCL				gpioPortC

// timer defines
#define timer_start                 0x01
#define timer_stop                  0x02
#define timer_counts_TSLdelay    	28000			// counts corresponding to delay of 2ms ( required to allow TSL power through GPIO to stabilize)


/* defines for I2C code*/
#define TSLAddr_write 0x72
#define TSLAddr_read  0x73
#define I2C_START    0x01
#define I2C_STOP     0x02
#define CMD_WORD      0xE0					// 80||40||20  -> CMD, CLEAR,WORD bits in TSL2561
#define PowerUp       0x03					// power up command
#define TimingReg     0x01					// configure TSL timing registers
#define THLOLO        0x0F					// Lower byte of low threshold register
#define THLOHI        0x00					// Higher byte of low threshold register
#define THHILO        0x00					// Lower byte of high threshold register
#define THHIHI        0x08					// Higher byte of high threshold register
#define TSL_IntReg    0x14					// configure interrupt register
#define Addr_ADCval   0x8C					// Address for reading ADC values for both ADCs
#define ClearInt      0x02
#define I2C_EnableInt 0x1AFF				// enable required i2c interrupt
#define TSL_clearinterrupt   0xC0			// 80||40
#define gpioclear_extint  0x0002

#define I2C_INIT_DEFAULT                                                  \
{                                                                         \
  true,                    /* Enable when init done */                    \
  true,                    /* Set to master mode */                       \
  0,                       /* Use currently configured reference clock */ \
  I2C_FREQ_FAST_MAX,   /* Set to standard rate assuring being */      \
                           /* within I2C spec */                          \
  i2cClockHLRStandard      /* Set to use 4:4 low/high duty cycle */       \
}






#endif
