/*
 * Copyright (c) 2022 Mauro Medina Serangeli
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//----------------------------------------------------------------------
//	INCLUCIÓN DE ARCHIVOS
//----------------------------------------------------------------------
#include <stdbool.h>
#include <stdint.h>

#include <zephyr.h>
#include <errno.h>

#include "nau7802.h"	
#include <drivers/i2c.h>

//----------------------------------------------------------------------
//	MACROS
//----------------------------------------------------------------------


//----------------------------------------------------------------------
//	Global variables
//----------------------------------------------------------------------

//NAU7802 IRQ variables
static uint32_t irq_pin 				= 0xFFFFFFFF;
static ext_irq_cb_t irq_user_cb 		= NULL;

static volatile bool irq_drdy_flg 		= false;
static volatile int32_t irq_data_raw 	= 0;
#ifdef CONFIG_NAU7802_TEMP
static uint8_t gain_mode 				= NAU7802_GAIN_128;
static uint16_t ldo_mode				= NAU7802_LDO_3V0;
#endif
//// /*NM*/
/*
#ifdef CONFIG_NAU7802_SCALE
//private:
//TwoWire *_i2cPort;					//This stores the user's requested i2c port
const uint8_t _deviceAddress = 0x2A;	

//y = mx+b

static int32_t _zeroOffset;				//This is b
static float _calibrationFactor;		//This is m. User provides this number so that we can output y when requested
#endif
*/
////

//----------------------------------------------------------------------
//	Local Functions
//----------------------------------------------------------------------
static int nau7802_waitForCalibrateAFE(uint32_t timeout_ms);
static void nau7802_beginCalibrateAFE(void);
static int32_t nau7802_internal_irq_enable(void);
static int32_t nau7802_internal_irq_disable(void);

//----------------------------------------------------------------------
//	Funci�n:			nau7802_irq_cb()
//----------------------------------------------------------------------
//	Objetivo:		   Handler interno de interrupcion del GPIO del NAU.
//					   Util para hacer polling de mediciones.
//					   Solo se utiliza cuando se pide promedio o 
//					   se llama a nau7802_internal_irq_enable() 
//----------------------------------------------------------------------
//	Param de entrada:   ninguno
//----------------------------------------------------------------------
//	Param de salida:	ninguno
//----------------------------------------------------------------------
static void nau7802_irq_cb(void){

	irq_data_raw = nau7802_getReading();
	
	irq_drdy_flg = true;
}


static int32_t get_irq_data(void){
	
	while(!irq_drdy_flg);
	
	irq_drdy_flg = false;
	
	return irq_data_raw;
}

//Sets up the NAU7802 for basic function
//If initialize is true (or not specified), default init and calibration is performed
//If initialize is false, then it's up to the caller to initalize and calibrate
//Returns true upon completion
int nau7802_begin(void)
{
	int ret = 0;		//Accumulate a result as we do the setup

	ret &= nau7802_powerUp();											//Power on analog and digital sections of the scale

	ret &= nau7802_check_chip_id();

	if(ret != 0)
	{
		return ret;
	}
#if (CONFIG_NAU7802_LDO == 8)
	result &= nau7802_LDO_off();
#else
	ret &= nau7802_setLDO(CONFIG_NAU7802_LDO);								//Set LDO to 3.3V note: 2v4 not recommended (doesnt work) 
#endif
	ret &= nau7802_setGain(CONFIG_NAU7802_PGA);							//Set gain to 128

	ret &= nau7802_setSampleRate(CONFIG_NAU7802_SPS); //40					//Set samples per second to 10
	
	ret &= nau7802_setRegister(NAU7802_ADC, 0x30);						//Turn off CLK_CHP. From 9.1 power on sequencing.	
#ifdef CONFIG_NAU7802_SINGLE_CHN
	result &= nau7802_setBit(NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR);	//Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note
#endif	
	ret &= nau7802_calibrateAFE();										//Re-cal analog front end when we change gain, sample rate, or channel
	
	//Get 6 samples to stabilize adc
	if(ret)
	{
		nau7802_internal_irq_enable();
		nau7802_start_conversions();
		for(uint8_t i = 0; i < NAU7802_SETTLE_SAMPLES; i++){

			get_irq_data();
		}
		nau7802_internal_irq_disable();
		nau7802_stop_conversions();
	}
	return (ret);
}

int nau7802_start_conversions(void){
	
	nau7802_getReading(NULL);

	return nau7802_setBit(NAU7802_PU_CTRL_CS, NAU7802_PU_CTRL);
}

int nau7802_stop_conversions(void){
	
	return nau7802_clearBit(NAU7802_PU_CTRL_CS, NAU7802_PU_CTRL);
}

//Returns true if Cycle Ready bit is set (conversion is complete)
int nau7802_data_available()
{
	return !(nau7802_getBit(NAU7802_PU_CTRL_CR, NAU7802_PU_CTRL) == 1);
}


//Calibrate analog front end of system. Returns true if CAL_ERR bit is 0 (no error)
//Takes approximately 344ms to calibrate; wait up to 1000ms.
//It is recommended that the AFE be re-calibrated any time the gain, SPS, or channel number is changed.
int nau7802_calibrateAFE()
{
	nau7802_beginCalibrateAFE(); 
	return(nau7802_waitForCalibrateAFE(1000)); 
}


//Begin asynchronous calibration of the analog front end.
// Poll for completion with calAFEStatus() or wait with waitForCalibrateAFE()
static void nau7802_beginCalibrateAFE(void)
{
	nau7802_setBit(NAU7802_CTRL2_CALS, NAU7802_CTRL2);
}


//Check calibration status.
static NAU7802_Cal_Status nau7802_calAFEStatus(void)
{
	if (nau7802_getBit(NAU7802_CTRL2_CALS, NAU7802_CTRL2))
	{
		return NAU7802_CAL_IN_PROGRESS;
	}

	if (nau7802_getBit(NAU7802_CTRL2_CAL_ERROR, NAU7802_CTRL2))
	{
		return NAU7802_CAL_FAILURE;
	}

	// Calibration passed
	return NAU7802_CAL_SUCCESS;
}


//Wait for asynchronous AFE calibration to complete with optional timeout.
static int nau7802_waitForCalibrateAFE(uint64_t timeout_ms)
{
	NAU7802_Cal_Status cal_ready = NAU7802_CAL_IN_PROGRESS;
	uint64_t t_ini = k_uptime_get();

	while ((cal_ready = nau7802_calAFEStatus()) == NAU7802_CAL_IN_PROGRESS)
	{
		if ((k_uptime_get() - t_ini) >= timeout_ms)
		{
			break;
		}
		k_msleep(1);
	}

	if (cal_ready == NAU7802_CAL_SUCCESS)
	{
		return (0);
	}
	
	return (-ETIME);
}


//Set the readings per second
//10, 20, 40, 80, and 320 samples per second is available
int nau7802_setSampleRate(uint8_t rate)
{
	uint8_t value = 0;
	int ret = 0;

	if (rate > 0b111)
	{
		rate = 0b111;		//Error check
	}

	nau7802_getRegister(NAU7802_CTRL2, &value);

	value &= 0b10001111;	//Clear CRS bits
	value |= rate << 4;		//Mask in new CRS bits

	return (nau7802_setRegister(NAU7802_CTRL2, value));
}


//Select between 1 and 2
int nau7802_setChannel(uint8_t channelNumber)
{
	if (channelNumber == NAU7802_CHANNEL_1)
	{
		nau7802_setBit(NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR);

		return (nau7802_clearBit(NAU7802_CTRL2_CHS, NAU7802_CTRL2));	//Channel 1 (default)
	}
	else
	{
		nau7802_clearBit(NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR);

		return (nau7802_setBit(NAU7802_CTRL2_CHS, NAU7802_CTRL2));		//Channel 2
	}
}

int nau7802_LDO_on(void){
	
	return nau7802_setBit(NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL);
}

int nau7802_LDO_off(void){
	
	return nau7802_clearBit(NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL);
}

//Power up digital and analog sections of scale
int nau7802_powerUp()
{
	nau7802_setBit(NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
	nau7802_setBit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL);

	//Wait for Power Up bit to be set - takes approximately 200us
	uint8_t counter = 0;
	
	while (1)
	{
		if (nau7802_getBit(NAU7802_PU_CTRL_PUR, NAU7802_PU_CTRL) == true)
		{
			break;				//Good to go
		}
		
		/*delay(1);*/espera_ms(1);
		
		if (counter++ > 100)
		{
			return (false);		//Error
		}
	}
	
	return (true);
}

int nau7802_check_chip_id(void){
	
	return (nau7802_getRegister(NAU7802_DEVICE_REV) == NAU7802_CHIP_ID);
}

//Puts scale into low-power mode
int nau7802_powerDown()
{
	nau7802_clearBit(NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
	return (nau7802_clearBit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL));
}


//Resets all registers to Power Of Defaults
int nau7802_reset()
{
	nau7802_setBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL);				//Set RR
		
	espera_ms(1);
	
	return nau7802_clearBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL);	//Clear RR to leave reset state
}


//Set the onboard Low-Drop-Out voltage regulator to a given value
//2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.2, 4.5V are available
int nau7802_setLDO(uint8_t ldoValue)
{
	if (ldoValue > 0b111)
	{
		ldoValue = 0b111;		//Error check
	}
	
	switch (ldoValue)
	{
		case NAU7802_LDO_2V4:
#ifdef CONFIG_NAU7802_TEMP			
			ldo_mode = 2400;
#endif			
			break;
		case NAU7802_LDO_2V7:
#ifdef CONFIG_NAU7802_TEMP			
			ldo_mode = 2700;
#endif			
			break;
		case NAU7802_LDO_3V0:
#ifdef CONFIG_NAU7802_TEMP		
			ldo_mode = 3000;
#endif			
			break;
		case NAU7802_LDO_3V3:
#ifdef CONFIG_NAU7802_TEMP		
			ldo_mode = 3300;
#endif			
			break;
		case NAU7802_LDO_3V6:
#ifdef CONFIG_NAU7802_TEMP		
			ldo_mode = 3600;
#endif			
			break;
		case NAU7802_LDO_3V9:
#ifdef CONFIG_NAU7802_TEMP		
			ldo_mode = 3900;
#endif			
			break;
		case NAU7802_LDO_4V2:
#ifdef CONFIG_NAU7802_TEMP		
			ldo_mode = 4200;
#endif			
			break;
		case NAU7802_LDO_4V5:
#ifdef CONFIG_NAU7802_TEMP		
			ldo_mode = 4500;
#endif			
			break;
		default:
#ifdef CONFIG_NAU7802_TEMP			
			ldo_mode = 3000;
#endif			
			break;
	}
	//Set the value of the LDO
	uint8_t value = nau7802_getRegister(NAU7802_CTRL1);
	value &= 0b11000111;		//Clear LDO bits
	value |= ldoValue << 3;		//Mask in new LDO bits
	nau7802_setRegister(NAU7802_CTRL1, value);

	return (nau7802_setBit(NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL));	//Enable the internal LDO
}


//Set the gain
//x1, 2, 4, 8, 16, 32, 64, 128 are avaialable
int nau7802_setGain(uint8_t gainValue)
{
	if (gainValue > 0b111)
	{
		gainValue = 0b111;		//Error check
	}
#ifdef CONFIG_NAU7802_TEMP
	gain_mode = gainValue;
#endif
	uint8_t value = nau7802_getRegister(NAU7802_CTRL1);
	value &= 0b11111000;		//Clear gain bits
	value |= gainValue;			//Mask in new bits

	return (nau7802_setRegister(NAU7802_CTRL1, value));
}

#ifdef CONFIG_NAU7802_REV_CODE
//Get the revision code of this IC
uint8_t nau7802_getRevisionCode()
{
	uint8_t revisionCode = nau7802_getRegister(NAU7802_DEVICE_REV);
	return (revisionCode & 0x0F);
}
#endif

//Returns 24-bit reading
//Assumes CR Cycle Ready bit (ADC conversion complete) has been checked to be 1
int32_t nau7802_getReading()
{
	uint32_t aux = 0;
	int32_t value = 0;
	
	aux = (uint32_t)(nau7802_getRegister(NAU7802_ADCO_B2) << 16);
	
	aux |= (uint32_t)(nau7802_getRegister(NAU7802_ADCO_B1) << 8);
	
	aux |= (uint32_t)(nau7802_getRegister(NAU7802_ADCO_B0));
	
	value = (int32_t) (aux << 8);
	
	int32_t val = (value >> 8) & 0x807FFFFF;
	
	return val;
}

//Set Int pin to be high when data is ready (default)
int nau7802_setIntPolarityHigh()
{
	return (nau7802_clearBit(NAU7802_CTRL1_CRP, NAU7802_CTRL1)); //0 = CRDY pin is high active (ready when 1)
}


//Set Int pin to be low when data is ready
int nau7802_setIntPolarityLow()
{
	return (nau7802_setBit(NAU7802_CTRL1_CRP, NAU7802_CTRL1)); //1 = CRDY pin is low active (ready when 0)
}


//Mask & set a given bit within a register
int nau7802_setBit(uint8_t bitNumber, uint8_t registerAddress)
{
	uint8_t value;
	int ret = nau7802_getRegister(registerAddress, &value);

	if(ret != 0)
	{
		return ret;
	}

	value |= (1 << bitNumber);		//Set this bit

	return (nau7802_setRegister(registerAddress, value));
}


//Mask & clear a given bit within a register
int nau7802_clearBit(uint8_t bitNumber, uint8_t registerAddress)
{
	uint8_t value;
	int ret = nau7802_getRegister(registerAddress, &value);

	if(ret != 0){return ret;}

	value &= ~(1 << bitNumber);		//Set this bit

	return (nau7802_setRegister(registerAddress, value));
}


//Return a given bit within a register
int nau7802_getBit(uint8_t bitNumber, uint8_t registerAddress, uint8_t *data)
{
	uint8_t value;
	int ret = nau7802_getRegister(registerAddress, &value);

	if(ret == 0)
	{
		value &= (1 << bitNumber);		//Clear all but this bit
		*data = value;
	}

	return !!ret;
}


//Get contents of a register
int nau7802_getRegister(const struct device *dev, uint8_t registerAddress, uint8_t *value)
{
	struct nau7802_data *data = dev->data;
	const struct nau7802_config *cfg = dev->config;

	return i2c_write_read(data->i2c_master, cfg->i2c_addr, &registerAddress, 1U, value, 1u); 
}


//Send a given value to be written to given address
//Return true if successful
int nau7802_setRegister(const struct device *dev, uint8_t registerAddress, uint8_t value)
{
	struct nau7802_data *data = dev->data;
	const struct nau7802_config *cfg = dev->config;

	return i2c_reg_write_byte(data->i2c_master, cfg->i2c_addr, registerAddress, value);
}

#ifdef CONFIG_NAU7802_TRIGGER
int nau7802_drdy_cb_register(const uint32_t pin, ext_irq_cb_t cb){
	
	irq_pin = pin;
	irq_user_cb = cb;
	
	return ext_irq_register(pin, cb) == 0;
}

int32_t nau7802_enable_irq(void){
	
	if(irq_pin != 0xFFFFFFFF){

		nau7802_getReading();
	
		return ext_irq_register(irq_pin, irq_user_cb);
	}else
	{
		return ext_irq_register(NAU7802_INT_DEFAULT_PIN, nau7802_irq_cb);
	}	
	return -1;
}

int32_t nau7802_disable_irq(void){
	
	if(irq_pin != 0xFFFFFFFF){
		
		return ext_irq_register(irq_pin, NULL);
	}else
	{
		return ext_irq_register(NAU7802_INT_DEFAULT_PIN, nau7802_irq_cb);
	}
	return -1;
}

static int32_t nau7802_internal_irq_enable(void){
	
	if(irq_pin != 0xFFFFFFFF){
		
		nau7802_getReading();

		return ext_irq_register(irq_pin, nau7802_irq_cb);
	}else
	{
		nau7802_getReading();

		return ext_irq_register(NAU7802_INT_DEFAULT_PIN, nau7802_irq_cb);
	}
	return -1;
}

static int32_t nau7802_internal_irq_disable(void){
	
	return nau7802_disable_irq();
}
#endif
