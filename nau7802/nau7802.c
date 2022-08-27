/*
 * Copyright (c) 2022 Mauro Medina Serangeli
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_nau7802

//----------------------------------------------------------------------
//	INCLUCIÓN DE ARCHIVOS
//----------------------------------------------------------------------
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr.h>
#include <errno.h>

#include "nau7802.h"	
#include <drivers/i2c.h>
#include <logging/log.h>
#include <pm/device.h>

LOG_MODULE_REGISTER(nau7802, CONFIG_SENSOR_LOG_LEVEL);
//----------------------------------------------------------------------
//	MACROS
//----------------------------------------------------------------------


//----------------------------------------------------------------------
//	Global variables
//----------------------------------------------------------------------

//NAU7802 IRQ variables
// static uint32_t irq_pin 				= 0xFFFFFFFF;
// static ext_irq_cb_t irq_user_cb 		= NULL;

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
static int nau7802_waitForCalibrateAFE(const struct device *dev, uint64_t timeout_ms);
static void nau7802_beginCalibrateAFE(const struct device *dev);
#ifdef CONFIG_NAU7802_TRIGGER
static int32_t nau7802_internal_irq_enable(void);
static int32_t nau7802_internal_irq_disable(void);
#endif
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
#ifdef CONFIG_NAU7802_TRIGGER
static void nau7802_irq_cb(void){

	irq_data_raw = nau7802_getReading();
	
	irq_drdy_flg = true;
}


static int32_t get_irq_data(void){
	
	while(!irq_drdy_flg);
	
	irq_drdy_flg = false;
	
	return irq_data_raw;
}
#endif
//Sets up the NAU7802 for basic function
//If initialize is true (or not specified), default init and calibration is performed
//If initialize is false, then it's up to the caller to initalize and calibrate
//Returns true upon completion
static int nau7802_begin(const struct device *dev)
{
	const struct nau7802_config *const config = dev->config;
	int ret = 0;		//Accumulate a result as we do the setup

	if (!device_is_ready(config->bus.bus)) {
		LOG_ERR("I2C bus %s is not ready", config->bus.bus->name);
		return -ENODEV;
	}

	ret = nau7802_powerUp(dev);											//Power on analog and digital sections of the scale
	if (ret < 0) {
		LOG_ERR("Failed to power up device.");
		return ret;
	}

	ret = nau7802_check_chip_id(dev);
	if(ret <= 0)
	{
		LOG_ERR("Chip ID error.");
		return ret;
	}

#if (CONFIG_NAU7802_LDO == 8)
	result |= nau7802_LDO_off(dev);
#else
	ret |= nau7802_setLDO(dev, /*CONFIG_NAU7802_LDO*/NAU7802_LDO_2V7);								//Set LDO to 3.3V note: 2v4 not recommended (doesnt work) 
#endif
	ret |= nau7802_setGain(dev, /* CONFIG_NAU7802_PGA */NAU7802_GAIN_8);							//Set gain to 128

	ret |= nau7802_setSampleRate(dev, /* CONFIG_NAU7802_SPS */NAU7802_SPS_40); //40					//Set samples per second to 10
	
	ret |= nau7802_setRegister(dev, NAU7802_ADC, 0x30);						//Turn off CLK_CHP. From 9.1 power on sequencing.	
#ifdef CONFIG_NAU7802_SINGLE_CHN
	ret |= nau7802_setBit(dev, NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR);	//Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note
#endif	
	if(ret <= 0)
	{
		LOG_ERR("Configuration error.");
		return ret;
	}

	ret = nau7802_calibrateAFE(dev);										//Re-cal analog front end when we change gain, sample rate, or channel
	if(ret <= 0)
	{
		LOG_ERR("Calibration error.");
		return ret;
	}

	//Get 6 samples to stabilize adc
#ifdef CONFIG_NAU7802_TRIGGER		
	nau7802_internal_irq_enable();
#endif		
	ret = nau7802_start_conversions(dev);
	if(ret <= 0)
	{
		LOG_ERR("Estabilization error.")
		return ret;
	}
	int64_t t_init = k_uptime_get();
	for(uint8_t i = 0; i < NAU7802_SETTLE_SAMPLES; i++){
#ifdef CONFIG_NAU7802_TRIGGER
		get_irq_data();
#else
		while(nau7802_data_available(dev) != 0)
		{
			if((k_uptime_get() - t_init) > NAU7802_DATA_TIMEOUT_MS)
				break;
			k_msleep(10);	
		}
		nau7802_getReading(dev, NULL);
#endif
	}
#ifdef CONFIG_NAU7802_TRIGGER		
	nau7802_internal_irq_disable();
#endif		
	ret = nau7802_stop_conversions(dev);
	
	return (ret);
}

/**
 * @brief sensor attribute set
 *
 * @retval 0 for success
 * @retval -ENOTSUP for unsupported channels
 * @retval -EIO for i2c write failure
 */
static int nau7802_attr_set(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr,
			   const struct sensor_value *val)
{
	const struct nau7802_config *config = dev->config;
	uint16_t data = val->val1;

	switch (attr) {
		case SENSOR_ATTR_SAMPLING_FREQUENCY:
			return nau7802_setSampleRate(dev, data);
		case SENSOR_ATTR_CALIB_TARGET:
			return 0;
		case SENSOR_ATTR_LDO:
			return nau7802_setLDO(dev, data);
		case SENSOR_ATTR_GAIN:
			return nau7802_setGain(dev, data);
		case SENSOR_ATTR_CHN:
			return nau7802_setChannel(dev, data);
		default:
			LOG_ERR("NAU7802 attribute not supported.");
			return -ENOTSUP;
	}
}

/**
 * @brief sensor attribute get
 *
 * @retval 0 for success
 * @retval -ENOTSUP for unsupported channels
 * @retval -EIO for i2c read failure
 */
static int nau7802_attr_get(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr,
			   struct sensor_value *val)
{
	const struct ina230_config *config = dev->config;
	uint16_t data;
	uint8_t reg;
	int ret;

	switch (attr) {
		case SENSOR_ATTR_SAMPLING_FREQUENCY:
			ret = nau7802_getRegister(dev, NAU7802_CTRL2, &reg);
			data = NAU7802_SPS(reg);
			break;
		case SENSOR_ATTR_CALIB_TARGET:
			ret = nau7802_getRegister(dev, NAU7802_CTRL2, &reg);
			data = NAU7802_CAL(reg);
			break;
		case SENSOR_ATTR_LDO:
			ret = nau7802_getRegister(dev, NAU7802_CTRL1, &reg);
			data = NAU7802_LDO(reg);
			break;
		case SENSOR_ATTR_GAIN:
			ret = nau7802_getRegister(dev, NAU7802_CTRL1, &reg);
			data = NAU7802_GAIN(reg);
			break;
		case SENSOR_ATTR_CHN:
			ret = nau7802_getRegister(dev, NAU7802_CTRL2, &reg);
			data = NAU7802_CHN(reg);
			break;
		default:
			LOG_ERR("NAU7802 attribute not supported.");
			return -ENOTSUP;
	}

	if (ret < 0) {
		return ret;
	}
	val->val1 = data;
	val->val2 = 0;

	return 0;
}

int nau7802_start_conversions(const struct device *dev){
	
	// nau7802_getReading(NULL);

	return nau7802_setBit(dev, NAU7802_PU_CTRL_CS, NAU7802_PU_CTRL);
}

int nau7802_stop_conversions(const struct device *dev){
	
	return nau7802_clearBit(dev, NAU7802_PU_CTRL_CS, NAU7802_PU_CTRL);
}

//Returns true if Cycle Ready bit is set (conversion is complete)
int nau7802_data_available(const struct device *dev)
{
	int ret;
	uint8_t data;

	ret = nau7802_getBit(dev, NAU7802_PU_CTRL_CR, NAU7802_PU_CTRL, &data);

	return (ret || !(data == 1));
}


//Calibrate analog front end of system. Returns true if CAL_ERR bit is 0 (no error)
//Takes approximately 344ms to calibrate; wait up to 1000ms.
//It is recommended that the AFE be re-calibrated any time the gain, SPS, or channel number is changed.
int nau7802_calibrateAFE(const struct device *dev)
{
	nau7802_beginCalibrateAFE(dev); 
	return(nau7802_waitForCalibrateAFE(dev, 1000)); 
}


//Begin asynchronous calibration of the analog front end.
// Poll for completion with calAFEStatus() or wait with waitForCalibrateAFE()
static void nau7802_beginCalibrateAFE(const struct device *dev)
{
	nau7802_setBit(dev, NAU7802_CTRL2_CALS, NAU7802_CTRL2);
}


//Check calibration status.
static NAU7802_Cal_Status nau7802_calAFEStatus(const struct device *dev)
{
	int ret;
	uint8_t data;

	ret = nau7802_getBit(dev, NAU7802_CTRL2_CALS, NAU7802_CTRL2, &data);
	if (ret == 0 && data == 1)
	{
		return NAU7802_CAL_IN_PROGRESS;
	}

	ret |= nau7802_getBit(dev, NAU7802_CTRL2_CAL_ERROR, NAU7802_CTRL2, &data);
	if (ret == 0 && data == 1)
	{
		return NAU7802_CAL_FAILURE;
	}

	if(ret == 0)
	{
		// Calibration passed
		return NAU7802_CAL_SUCCESS;
	}
	return ret;
}


//Wait for asynchronous AFE calibration to complete with optional timeout.
static int nau7802_waitForCalibrateAFE(const struct device *dev, uint64_t timeout_ms)
{
	NAU7802_Cal_Status cal_ready = NAU7802_CAL_IN_PROGRESS;
	uint64_t t_ini = k_uptime_get();

	while ((cal_ready = nau7802_calAFEStatus(dev)) == NAU7802_CAL_IN_PROGRESS)
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
int nau7802_setSampleRate(const struct device *dev,  uint8_t rate)
{
	int ret;
	uint8_t value = 0;

	if (rate > 0b111)
	{
		rate = 0b111;		//Error check
	}

	ret = nau7802_getRegister(dev, NAU7802_CTRL2, &value);
	if(ret != 0)
	{
		return ret;
	}

	value &= 0b10001111;	//Clear CRS bits
	value |= rate << 4;		//Mask in new CRS bits

	return (nau7802_setRegister(dev, NAU7802_CTRL2, value));
}


//Select between 1 and 2
int nau7802_setChannel(const struct device *dev, uint8_t channelNumber)
{
	if (channelNumber == NAU7802_CHANNEL_1)
	{
		nau7802_setBit(dev, NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR);

		return (nau7802_clearBit(dev, NAU7802_CTRL2_CHS, NAU7802_CTRL2));	//Channel 1 (default)
	}
	else if(channelNumber == NAU7802_CHANNEL_2)
	{
		nau7802_clearBit(dev, NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR);

		return (nau7802_setBit(dev, NAU7802_CTRL2_CHS, NAU7802_CTRL2));		//Channel 2
	}

	return -EINVAL;
}

int nau7802_LDO_on(const struct device *dev){
	
	return nau7802_setBit(dev, NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL);
}

int nau7802_LDO_off(const struct device *dev){
	
	return nau7802_clearBit(dev, NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL);
}

//Power up digital and analog sections of scale
int nau7802_powerUp(const struct device *dev)
{
	int ret;
	uint8_t data;

	//Wait for Power Up bit to be set - takes approximately 200us
	uint8_t counter = 0;

	ret = nau7802_setBit(dev, NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
	ret |= nau7802_setBit(dev, NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL);

	while (ret == 0)
	{
		nau7802_getBit(dev, NAU7802_PU_CTRL_PUR, NAU7802_PU_CTRL, &data);
		if (data == 1)
		{
			break;				//Good to go
		}
		
		k_msleep(1);
		
		if (counter++ > 100)
		{
			return (-ETIME);		//Error
		}
	}
	
	return ret;
}

int nau7802_check_chip_id(const struct device *dev){
	
	int ret;
	uint8_t value;

	ret = nau7802_getRegister(dev, NAU7802_DEVICE_REV, &value);

	return (!(value == NAU7802_CHIP_ID) || ret);
}

//Puts scale into low-power mode
int nau7802_powerDown(const struct device *dev)
{
	nau7802_clearBit(dev, NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
	return (nau7802_clearBit(dev, NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL));
}


//Resets all registers to Power Of Defaults
int nau7802_reset(const struct device *dev)
{
	nau7802_setBit(dev, NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL);				//Set RR
		
	k_msleep(1);
	
	return nau7802_clearBit(dev, NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL);	//Clear RR to leave reset state
}


//Set the onboard Low-Drop-Out voltage regulator to a given value
//2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.2, 4.5V are available
int nau7802_setLDO(const struct device *dev, uint8_t ldoValue)
{
	int ret;
	uint8_t value;

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
	ret = nau7802_getRegister(dev, NAU7802_CTRL1, &value);
	
	value &= 0b11000111;		//Clear LDO bits
	value |= ldoValue << 3;		//Mask in new LDO bits
	
	ret |= nau7802_setRegister(dev, NAU7802_CTRL1, value);

	return (nau7802_setBit(dev, NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL) || ret);	//Enable the internal LDO
}


//Set the gain
//x1, 2, 4, 8, 16, 32, 64, 128 are avaialable
int nau7802_setGain(const struct device *dev, uint8_t gainValue)
{
	int ret;
	uint8_t value;

	if (gainValue > 0b111)
	{
		gainValue = 0b111;		//Error check
	}
#ifdef CONFIG_NAU7802_TEMP
	gain_mode = gainValue;
#endif
	ret = nau7802_getRegister(dev, NAU7802_CTRL1, &value);
	value &= 0b11111000;		//Clear gain bits
	value |= gainValue;			//Mask in new bits

	return (nau7802_setRegister(dev, NAU7802_CTRL1, value) || ret);
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
int nau7802_getReading(const struct device *dev, int32_t *data)
{
	int ret;
	uint32_t aux = 0;
	int32_t value = 0;
	uint8_t buf[4] = {0};
	
	ret = nau7802_getRegister(dev, NAU7802_ADCO_B2, &buf[2]);

	ret |= nau7802_getRegister(dev, NAU7802_ADCO_B1, &buf[1]);

	ret |= nau7802_getRegister(dev, NAU7802_ADCO_B1, &buf[0]);

	if(ret == 0)
	{
		memcpy(&aux, buf, sizeof(value));

		value = (int32_t) (aux << 8);
	
		*data = (value >> 8) & 0x807FFFFF;
	}
	
	return ret;
}

//Set Int pin to be high when data is ready (default)
int nau7802_setIntPolarityHigh(const struct device *dev)
{
	return (nau7802_clearBit(dev, NAU7802_CTRL1_CRP, NAU7802_CTRL1)); //0 = CRDY pin is high active (ready when 1)
}


//Set Int pin to be low when data is ready
int nau7802_setIntPolarityLow(const struct device *dev)
{
	return (nau7802_setBit(dev, NAU7802_CTRL1_CRP, NAU7802_CTRL1)); //1 = CRDY pin is low active (ready when 0)
}


//Mask & set a given bit within a register
int nau7802_setBit(const struct device *dev, uint8_t bitNumber, uint8_t registerAddress)
{
	uint8_t value;
	int ret = nau7802_getRegister(dev, registerAddress, &value);

	if(ret != 0)
	{
		return ret;
	}

	value |= (1 << bitNumber);		//Set this bit

	return (nau7802_setRegister(dev, registerAddress, value));
}


//Mask & clear a given bit within a register
int nau7802_clearBit(const struct device *dev, uint8_t bitNumber, uint8_t registerAddress)
{
	uint8_t value;
	int ret = nau7802_getRegister(dev, registerAddress, &value);

	if(ret != 0){return ret;}

	value &= ~(1 << bitNumber);		//Set this bit

	return (nau7802_setRegister(dev, registerAddress, value));
}


//Return a given bit within a register
int nau7802_getBit(const struct device *dev, uint8_t bitNumber, uint8_t registerAddress, uint8_t *data)
{
	uint8_t value;
	int ret = nau7802_getRegister(dev, registerAddress, &value);

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

static const struct sensor_driver_api nau7802_driver_api = {
	.attr_set = nau7802_attr_set,
	.attr_get = nau7802_attr_get,
#ifdef CONFIG_NAU7802_TRIGGER
	.trigger_set = nau7802_trigger_set,
#endif
	.sample_fetch = nau7802_sample_fetch,
	.channel_get = nau7802_channel_get,
};

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "NAU7802 driver enabled without any devices"
#endif

#ifdef CONFIG_NAU7802_TRIGGER
#define NAU7802_CFG_IRQ(inst)				\
	.trig_enabled = true,				\
	.mask = DT_INST_PROP(inst, mask),		\
	.alert_limit = DT_INST_PROP(inst, alert_limit),	\
	.gpio_alert = GPIO_DT_SPEC_INST_GET(inst, irq_gpios)
#else
#define NAU7802_CFG_IRQ(inst)
#endif /* CONFIG_NAU7802_TRIGGER */

#define NAU7802_DEVICE_INIT(inst)					\
	static struct nau7802_data drv_data_##inst;		    \
	static const struct nau7802_config drv_config_##inst = {	    \
		.bus = I2C_DT_SPEC_INST_GET(inst),		    \
		.config = DT_INST_PROP(inst, config),		    \
		.current_lsb = DT_INST_PROP(inst, current_lsb),	    \
		.rshunt = DT_INST_PROP(inst, rshunt),		    \
		COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, irq_gpios), \
			    (NAU7802_CFG_IRQ(inst)), ())		    \
	};							    \
	DEVICE_DT_INST_DEFINE(inst,					\
			    nau7802_begin,				\
			    &nau7802_data_##inst,			\
			    &nau7802_config_##inst,			\
			    POST_KERNEL,				\
			    CONFIG_SENSOR_INIT_PRIORITY,		\
			    &nau7802_driver_api);

DT_INST_FOREACH_STATUS_OKAY(NAU7802_DRIVER_INIT)
