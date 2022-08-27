/*
 * Copyright (c) 2022 Mauro Medina Serangeli
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_DRIVERS_SENSOR_NAU7802_H_
#define ZEPHYR_DRIVERS_SENSOR_NAU7802_H_

#include <errno.h>

#include <zephyr/types.h>
#include <device.h>
#include <drivers/sensor.h>
#include <sys/util.h>

#include <drivers/gpio.h>
#include <drivers/i2c.h>

#define NAU7802_I2C_ADDR 			0x2A
#define NAU7802_CHIP_ID 			0x0F
#define NAU7802_SETTLE_SAMPLES 		6
#define NAU7802_MAX_CNT 			(1 << 23)

#define NAU7802_DATA_TIMEOUT_MS		1000

//Custom attribute definitions
#define SENSOR_ATTR_LDO (SENSOR_ATTR_PRIV_START + 1)
#define SENSOR_ATTR_GAIN (SENSOR_ATTR_PRIV_START + 2)
#define SENSOR_ATTR_CHN (SENSOR_ATTR_PRIV_START + 3)

//TEMPERATURE MODE
#define NAU7802_TEMP_REF_C 		25
#define NAU7802_TEMP_REF_MV 	109
#define NAU7802_TEMP_DELTA_UV 	360

//Attribute get macros
#define NAU7802_CAL(reg) (reg & 0x03)
#define NAU7802_SPS(reg) (reg & 0x70)
#define NAU7802_CHN(reg) (reg & 0x80)
#define NAU7802_LDO(reg) (reg & 0x38)
#define NAU7802_GAIN(reg)(reg & 0x07)
//Register Map
typedef enum
{
	NAU7802_PU_CTRL      = 0x00,
	NAU7802_CTRL1,
	NAU7802_CTRL2,
	NAU7802_OCAL1_B2,
	NAU7802_OCAL1_B1,
	NAU7802_OCAL1_B0,
	NAU7802_GCAL1_B3,
	NAU7802_GCAL1_B2,
	NAU7802_GCAL1_B1,
	NAU7802_GCAL1_B0,
	NAU7802_OCAL2_B2,
	NAU7802_OCAL2_B1,
	NAU7802_OCAL2_B0,
	NAU7802_GCAL2_B3,
	NAU7802_GCAL2_B2,
	NAU7802_GCAL2_B1,
	NAU7802_GCAL2_B0,
	NAU7802_I2C_CONTROL,
	NAU7802_ADCO_B2,
	NAU7802_ADCO_B1,
	NAU7802_ADCO_B0,
	NAU7802_ADC          = 0x15,	//Shared ADC and OTP 32:24
	NAU7802_OTP_B1,					//OTP 23:16 or 7:0?
	NAU7802_OTP_B0,					//OTP 15:8
	NAU7802_PGA          = 0x1B,
	NAU7802_PGA_PWR      = 0x1C,
	NAU7802_DEVICE_REV   = 0x1F,
} Scale_Registers;

//Bits within the PU_CTRL register
typedef enum
{
	NAU7802_PU_CTRL_RR = 0,
	NAU7802_PU_CTRL_PUD,
	NAU7802_PU_CTRL_PUA,
	NAU7802_PU_CTRL_PUR,
	NAU7802_PU_CTRL_CS,
	NAU7802_PU_CTRL_CR,
	NAU7802_PU_CTRL_OSCS,
	NAU7802_PU_CTRL_AVDDS,
} PU_CTRL_Bits;

//Bits within the CTRL1 register
typedef enum
{
	NAU7802_CTRL1_GAIN     = 2,
	NAU7802_CTRL1_VLDO     = 5,
	NAU7802_CTRL1_DRDY_SEL = 6,
	NAU7802_CTRL1_CRP      = 7,
} CTRL1_Bits;

//Bits within the CTRL2 register
typedef enum
{
	NAU7802_CTRL2_CALMOD    = 0,
	NAU7802_CTRL2_CALS      = 2,
	NAU7802_CTRL2_CAL_ERROR = 3,
	NAU7802_CTRL2_CRS       = 4,
	NAU7802_CTRL2_CHS       = 7,
} CTRL2_Bits;

//Bits within the PGA register
typedef enum
{
	NAU7802_PGA_CHP_DIS    = 0,
	NAU7802_PGA_INV        = 3,
	NAU7802_PGA_BYPASS_EN,
	NAU7802_PGA_OUT_EN,
	NAU7802_PGA_LDOMODE,
	NAU7802_PGA_RD_OTP_SEL,
} PGA_Bits;

//Bits within the PGA PWR register
typedef enum
{
	NAU7802_PGA_PWR_PGA_CURR       = 0,
	NAU7802_PGA_PWR_ADC_CURR       = 2,
	NAU7802_PGA_PWR_MSTR_BIAS_CURR = 4,
	NAU7802_PGA_PWR_PGA_CAP_EN     = 7,
} PGA_PWR_Bits;

//Bits within the I2C control register
typedef enum
{
	NAU7802_PU_I2C_CTRL_BGPCP = 0,
	NAU7802_PU_I2C_CTRL_TS,
	NAU7802_PU_I2C_CTRL_BOPGA,
	NAU7802_PU_I2C_CTRL_WPD,
	NAU7802_PU_I2C_CTRL_SI,
	NAU7802_PU_I2C_CTRL_SPE,
	NAU7802_PU_I2C_CTRLL_FRD,
	NAU7802_PU_I2C_CTRL_CRSD,
} PU_I2C_CTRL_Bits;

//Allowed Low drop out regulator voltages
typedef enum
{
	NAU7802_LDO_2V4 = 0b111,
	NAU7802_LDO_2V7 = 0b110,
	NAU7802_LDO_3V0 = 0b101,
	NAU7802_LDO_3V3 = 0b100,
	NAU7802_LDO_3V6 = 0b011,
	NAU7802_LDO_3V9 = 0b010,
	NAU7802_LDO_4V2 = 0b001,
	NAU7802_LDO_4V5 = 0b000,
} NAU7802_LDO_Values;

//Allowed gains
typedef enum
{
	NAU7802_GAIN_128 = 0b111,
	NAU7802_GAIN_64  = 0b110,
	NAU7802_GAIN_32  = 0b101,
	NAU7802_GAIN_16  = 0b100,
	NAU7802_GAIN_8   = 0b011,
	NAU7802_GAIN_4   = 0b010,
	NAU7802_GAIN_2   = 0b001,
	NAU7802_GAIN_1   = 0b000,
} NAU7802_Gain_Values;

//Allowed samples per second
typedef enum
{
	NAU7802_SPS_320 = 0b111,
	NAU7802_SPS_80  = 0b011,
	NAU7802_SPS_40  = 0b010,
	NAU7802_SPS_20  = 0b001,
	NAU7802_SPS_10  = 0b000,
} NAU7802_SPS_Values;

//Select between channel values
typedef enum
{
	NAU7802_CHANNEL_1 = 0,
	NAU7802_CHANNEL_2 = 1,
} NAU7802_Channels;

//Calibration state
typedef enum
{
	NAU7802_CAL_SUCCESS		= 0,
	NAU7802_CAL_IN_PROGRESS = 1,
	NAU7802_CAL_FAILURE		= 2,
} NAU7802_Cal_Status;

struct nau7802_data {
	const struct device *i2c_master;

	uint32_t sample;

#ifdef CONFIG_NAU7802_TRIGGER
	const struct device *drdy_gpio;
	struct gpio_callback drdy_cb;

	const struct device *dev;

	struct sensor_trigger trig;
	sensor_trigger_handler_t handler_drdy;

#ifdef CONFIG_NAU7802_TRIGGER_OWN_THREAD
	struct k_sem sem;
#endif

#ifdef CONFIG_NAU7802_TRIGGER_GLOBAL_THREAD
	struct k_work work;
#endif

#endif	/* CONFIG_NAU7802_TRIGGER */
};

struct nau7802_config {
	const char *i2c_bus;
	uint16_t i2c_addr;
	uint8_t resolution;
#ifdef CONFIG_NAU7802_TRIGGER
	const struct gpio_dt_spec gpio_drdy;
#endif /* CONFIG_NAU7802_TRIGGER */
};

#ifdef CONFIG_NAU7802_TRIGGER
int nau7802_trigger_set(const struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler);

int nau7802_setup_interrupt(const struct device *dev);

int nau7802_setIntPolarityHigh(const struct device *dev);																
int nau7802_setIntPolarityLow(const struct device *dev);
int nau7802_disable_irq(void);
int nau7802_enable_irq(void);
#endif

int nau7802_attr_set(const struct device *dev, 
			 enum sensor_channel chan,
		     enum sensor_attribute attr,
		     const struct sensor_value *val);


int nau7802_begin(const struct device *dev);						
int nau7802_start_conversions(const struct device *dev);
int nau7802_stop_conversions(const struct device *dev);																
int nau7802_reset(const struct device *dev);															
int nau7802_powerUp(const struct device *dev);																		
int nau7802_powerDown(const struct device *dev);																															
int nau7802_calibrateAFE(const struct device *dev);	

int nau7802_check_chip_id(const struct device *dev);

//Fetch		
int nau7802_getReading(const struct device *dev, int32_t *data);																	
int nau7802_data_available(const struct device *dev);

//Attributes 
int nau7802_setGain(const struct device *dev, uint8_t gainValue);													
int nau7802_setLDO(const struct device *dev, uint8_t ldoValue);															
int nau7802_setSampleRate(const struct device *dev, uint8_t rate);														
int nau7802_setChannel(const struct device *dev, uint8_t channelNumber);													
int nau7802_LDO_on(const struct device *dev);
int nau7802_LDO_off(const struct device *dev);	

//Comunication
int nau7802_setBit(const struct device *dev, uint8_t bitNumber, uint8_t registerAddress);					//Mask & set a given bit within a register
int nau7802_clearBit(const struct device *dev, uint8_t bitNumber, uint8_t registerAddress);				//Mask & clear a given bit within a register
int nau7802_getBit(const struct device *dev, uint8_t bitNumber, uint8_t registerAddress, uint8_t *data);					//Return a given bit within a register
int nau7802_getRegister(const struct device *dev, uint8_t registerAddress, uint8_t *value);				//Get contents of a register
int nau7802_setRegister(const struct device *dev, uint8_t registerAddress, uint8_t value);				//Send a given value to be written to given address. Return true if successful


/* TODO: Add scale functionality? 
* #ifdef NAU7802_CONFIG_SCALE
* int nau7802_getAverage(uint8_t samplesToTake);												//Return the average of a given number of readings
* int nau7802_calculateZeroOffset(uint8_t averageAmount);								//Also called taring. Call this with nothing on the scale
* void nau7802_setZeroOffset(int32_t newZeroOffset);												//Sets the internal variable. Useful for users who are loading values from NVM.
* int nau7802_getZeroOffset();																//Ask library for this value. Useful for storing value into NVM.
* void nau7802_calculateCalibrationFactor(float weightOnScale, uint8_t averageAmount);	//Call this with the value of the thing on the scale. Sets the calibration factor based on the weight on scale and zero offset.
* void nau7802_setCalibrationFactor(float calFactor);												//Pass a known calibration factor into library. Helpful if users is loading settings from NVM.
* float nau7802_getCalibrationFactor();															//Ask library for this value. Useful for storing value into NVM.
* float nau7802_getWeight(bool allowNegativeWeights, uint8_t samplesToTake);	//Once you've set zero offset and cal factor, you can ask the library to do the calculations for you.
* #endif
*/
#endif	//NAU7802_H_
