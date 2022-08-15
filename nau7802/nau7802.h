/*
 * Copyright (c) 2022 Mauro Medina Serangeli
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_DRIVERS_SENSOR_NAU7802_H_
#define ZEPHYR_DRIVERS_SENSOR_NAU7802_H_

#include <stdint.h>
#include <stdbool.h>

#include <drivers/gpio.h>
#include <drivers/i2c.h>


#define NAU7802_INT_DEFAULT_PIN 0
#define ADC_I2C_ADDR 0x2A
#define NAU7802_ID 0x0F

#define NAU7802_AVERAGE_TIMEOUT 1450
#define NAU7802_SETTLE_SAMPLES 6

#define NAU7802_SINGLE_CHN

#define NAU7802_MAX_CNT (1 << 23)
#define SHIFT_TEMP(val) ((val >> 8) & 0x80007FFF)
//TEMPERATURE MODE
#define NAU7802_TEMP_REF_C 		25
#define NAU7802_TEMP_REF_MV 	109
#define NAU7802_TEMP_DELTA_UV 	360

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



bool nau7802_begin(/*TwoWire &wirePort = Wire,*/ bool reset /*= true*/);						//Check communication and initialize sensor
bool nau7802_start_conversions(void);
bool nau7802_stop_conversions(void);
bool nau7802_isConnected();																		//Returns true if device acks at the I2C address

bool nau7802_available();																		//Returns true if Cycle Ready bit is set (conversion is complete)
int32_t nau7802_getReading();																	//Returns 24-bit reading. Assumes CR Cycle Ready bit (ADC conversion complete) has been checked by .available()

bool nau7802_setGain(uint8_t gainValue);														//Set the gain. x1, 2, 4, 8, 16, 32, 64, 128 are available
bool nau7802_setLDO(uint8_t ldoValue);															//Set the onboard Low-Drop-Out voltage regulator to a given value. 2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.2, 4.5V are avaialable
bool nau7802_setSampleRate(uint8_t rate);														//Set the readings per second. 10, 20, 40, 80, and 320 samples per second is available
bool nau7802_setChannel(uint8_t channelNumber);													//Select between 1 and 2
bool nau7802_LDO_on(void);
bool nau7802_LDO_off(void);

bool nau7802_calibrateAFE();																	//Synchronous calibration of the analog front end of the NAU7802. Returns true if CAL_ERR bit is 0 (no error)
void nau7802_beginCalibrateAFE();																//Begin asynchronous calibration of the analog front end of the NAU7802. Poll for completion with calAFEStatus() or wait with waitForCalibrateAFE().
bool nau7802_waitForCalibrateAFE(uint32_t timeout_ms /*= 0*/);									//Wait for asynchronous AFE calibration to complete with optional timeout.
NAU7802_Cal_Status nau7802_calAFEStatus();														//Check calibration status.

/*bool reset();*/bool nau7802_reset();															//Resets all registers to Power Of Defaults

bool nau7802_powerUp();																			//Power up digital and analog sections of scale, ~2mA
bool nau7802_powerDown();																		//Puts scale into low-power 200nA mode

bool nau7802_setIntPolarityHigh();																//Set Int pin to be high when data is ready (default)
bool nau7802_setIntPolarityLow();																//Set Int pin to be low when data is ready

bool nau7802_setBit(uint8_t bitNumber, uint8_t registerAddress);								//Mask & set a given bit within a register
bool nau7802_clearBit(uint8_t bitNumber, uint8_t registerAddress);								//Mask & clear a given bit within a register
bool nau7802_getBit(uint8_t bitNumber, uint8_t registerAddress);								//Return a given bit within a register

uint8_t nau7802_getRegister(uint8_t registerAddress);											//Get contents of a register
bool nau7802_setRegister(uint8_t registerAddress, uint8_t value);								//Send a given value to be written to given address. Return true if successful

bool nau7802_check_chip_id(void);

/**
 * \brief Register adc interrupt callback
 *
 * \param[in] pin  CPU pin connected to DRDY adc pin.
 *						 					
 * \param[in] cb   Callback for data ready events
 *						
 * \return Registration status.
 * \retval -1 Passed parameters were invalid
 * \retval 0 The callback registration is completed successfully
 */
bool nau7802_drdy_cb_register(const uint32_t pin, ext_irq_cb_t cb);

int32_t nau7802_disable_irq(void);
int32_t nau7802_enable_irq(void);

int32_t nau7802_internal_irq_enable(void);
int32_t nau7802_internal_irq_disable(void);
#ifdef NAU7802_CONFIG_REV_CODE
uint8_t nau7802_getRevisionCode();																//Get the revision code of this IC. Always 0x0F.
#endif
#ifdef NAU7802_CONFIG_SCALE
int32_t nau7802_getAverage(uint8_t samplesToTake);												//Return the average of a given number of readings
int32_t nau7802_calculateZeroOffset(uint8_t averageAmount /*= 8*/);								//Also called taring. Call this with nothing on the scale
void nau7802_setZeroOffset(int32_t newZeroOffset);												//Sets the internal variable. Useful for users who are loading values from NVM.
int32_t nau7802_getZeroOffset();																//Ask library for this value. Useful for storing value into NVM.
void nau7802_calculateCalibrationFactor(float weightOnScale, uint8_t averageAmount /*= 8*/);	//Call this with the value of the thing on the scale. Sets the calibration factor based on the weight on scale and zero offset.
void nau7802_setCalibrationFactor(float calFactor);												//Pass a known calibration factor into library. Helpful if users is loading settings from NVM.
float nau7802_getCalibrationFactor();															//Ask library for this value. Useful for storing value into NVM.
float nau7802_getWeight(bool allowNegativeWeights /*= false*/, uint8_t samplesToTake /*= 8*/);	//Once you've set zero offset and cal factor, you can ask the library to do the calculations for you.
#endif
#ifdef NAU7802_CONFIG_TEMP
bool nau7802_temperature_input_set(bool tmp_sens);
int32_t nau7802_temperature_get(uint8_t average_samples);
#endif
#endif	//NAU7802_h
