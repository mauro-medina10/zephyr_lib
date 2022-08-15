//----------------------------------------------------------------------
//						KRETZ S.A. 	-	 Balanza PS
//----------------------------------------------------------------------
//	ARCHIVO:	ADC_NAU7802.c
//----------------------------------------------------------------------
//	OBJETIVO:	Rutinas generales del ADC NAU7802.
//----------------------------------------------------------------------
//	HISTORIA: 	01-Feb-2022			V1.0			Nicolás Matteucci
//----------------------------------------------------------------------


//----------------------------------------------------------------------
//	INCLUCIÓN DE ARCHIVOS
//----------------------------------------------------------------------
#include <stdbool.h>
#include <stdint.h>

#include <zephyr.h>
#include <errno.h>

#include "Drv_I2C.h"	
//----------------------------------------------------------------------
//	MACROS
//----------------------------------------------------------------------
#define ADC_DISABLE_IRQ()	if(irq_pin != 0xFFFFFFFF)ext_irq_disable((const uint32_t) irq_pin)
#define ADC_ENABLE_IRQ()	if(irq_pin != 0xFFFFFFFF)ext_irq_enable((const uint32_t) irq_pin)
 

//#define NAU7802_LDO_OFF
//#define NAU7802_I2C_PULL_S

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
#ifdef CONFIG_NAU7802_SCALE
//private:
//TwoWire *_i2cPort;					//This stores the user's requested i2c port
const uint8_t _deviceAddress = 0x2A;	//Default unshifted 7-bit address of the NAU7802		*** VER DE PONER COMO #define ***

//y = mx+b

static int32_t _zeroOffset;					//This is b
static float _calibrationFactor;				//This is m. User provides this number so that we can output y when requested
#endif
////

//----------------------------------------------------------------------
//	Local Functions
//----------------------------------------------------------------------

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
bool nau7802_begin(/*TwoWire &wirePort,*/ bool initialize)
{
	bool result = true;		//Accumulate a result as we do the setup

	//Inits I2C interface
	result = I2C_init(&I2C_0, ADC_I2C_ADDR, I2C_M_SEVEN);

	if (initialize && result)
	{
		result &= nau7802_reset();												//Reset all registers

		result &= nau7802_powerUp();											//Power on analog and digital sections of the scale

		result &= nau7802_check_chip_id();

#ifdef NAU7802_LDO_OFF
		result &= nau7802_LDO_off();
#else
		result &= nau7802_setLDO(NAU7802_LDO_3V3);								//Set LDO to 3.3V note: 2v4 not recommended (doesnt work) 
#endif
#ifdef NAU7802_I2C_PULL_S
		result &= nau7802_setBit(NAU7802_PU_I2C_CTRL_SPE, NAU7802_I2C_CONTROL);
#endif
		result &= nau7802_setGain(NAU7802_GAIN_128);							//Set gain to 128

		result &= nau7802_setSampleRate(NAU7802_SPS_40); //40					//Set samples per second to 10
		
		result &= nau7802_setRegister(NAU7802_ADC, 0x30);						//Turn off CLK_CHP. From 9.1 power on sequencing.	
#ifdef NAU7802_SINGLE_CHN
		result &= nau7802_setBit(NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR);	//Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note
#endif	
		result &= nau7802_calibrateAFE();										//Re-cal analog front end when we change gain, sample rate, or channel
	}
	//Get 6 samples to stabilize adc
	if(result)
	{
		nau7802_internal_irq_enable();
		nau7802_start_conversions();
		for(uint8_t i = 0; i < NAU7802_SETTLE_SAMPLES; i++){

			get_irq_data();
		}
		nau7802_internal_irq_disable();
		nau7802_stop_conversions();
	}
	return (result);
}

bool nau7802_start_conversions(void){
	
	nau7802_getReading();

	return nau7802_setBit(NAU7802_PU_CTRL_CS, NAU7802_PU_CTRL);
}

bool nau7802_stop_conversions(void){
	
	return nau7802_clearBit(NAU7802_PU_CTRL_CS, NAU7802_PU_CTRL);
}

//Returns true if Cycle Ready bit is set (conversion is complete)
bool nau7802_available()
{
	return (nau7802_getBit(NAU7802_PU_CTRL_CR, NAU7802_PU_CTRL));
}


//Calibrate analog front end of system. Returns true if CAL_ERR bit is 0 (no error)
//Takes approximately 344ms to calibrate; wait up to 1000ms.
//It is recommended that the AFE be re-calibrated any time the gain, SPS, or channel number is changed.
bool nau7802_calibrateAFE()
{
	nau7802_beginCalibrateAFE(); 
	return(nau7802_waitForCalibrateAFE(1000)); 
}


//Begin asynchronous calibration of the analog front end.
// Poll for completion with calAFEStatus() or wait with waitForCalibrateAFE()
void nau7802_beginCalibrateAFE()
{
	nau7802_setBit(NAU7802_CTRL2_CALS, NAU7802_CTRL2);
}


//Check calibration status.
NAU7802_Cal_Status nau7802_calAFEStatus()
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
//If timeout is not specified (or set to 0), then wait indefinitely.
//Returns true if calibration completes succsfully, otherwise returns false.
bool nau7802_waitForCalibrateAFE(uint32_t timeout_ms)
{
	//uint32_t begin = millis();
	
	NAU7802_Cal_Status cal_ready;

	uiTmrPsj = timeout_ms;
	
	while ((cal_ready = nau7802_calAFEStatus()) == NAU7802_CAL_IN_PROGRESS)
	{
		if ((timeout_ms > 0) && (uiTmrPsj == 0))
		{
			break;
		}

		espera_ms(1);
	}

	if (cal_ready == NAU7802_CAL_SUCCESS)
	{
		return (true);
	}
	
	return (false);
}


//Set the readings per second
//10, 20, 40, 80, and 320 samples per second is available
bool nau7802_setSampleRate(uint8_t rate)
{
	if (rate > 0b111)
	{
		rate = 0b111;		//Error check
	}

	uint8_t value = nau7802_getRegister(NAU7802_CTRL2);
	value &= 0b10001111;	//Clear CRS bits
	value |= rate << 4;		//Mask in new CRS bits

	return (nau7802_setRegister(NAU7802_CTRL2, value));
}


//Select between 1 and 2
bool nau7802_setChannel(uint8_t channelNumber)
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

bool nau7802_LDO_on(void){
	
	return nau7802_setBit(NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL);
}

bool nau7802_LDO_off(void){
	
	return nau7802_clearBit(NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL);
}

//Power up digital and analog sections of scale
bool nau7802_powerUp()
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


//Puts scale into low-power mode
bool nau7802_powerDown()
{
	nau7802_clearBit(NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
	return (nau7802_clearBit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL));
}


//Resets all registers to Power Of Defaults
bool nau7802_reset()
{
	nau7802_setBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL);				//Set RR
		
	espera_ms(1);
	
	return nau7802_clearBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL);	//Clear RR to leave reset state
}


//Set the onboard Low-Drop-Out voltage regulator to a given value
//2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.2, 4.5V are available
bool nau7802_setLDO(uint8_t ldoValue)
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
bool nau7802_setGain(uint8_t gainValue)
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

#ifdef CONFIG_NAU7802_SCALE
//Return the average of a given number of readings
//Gives up after 1000ms so don't call this function to average 8 samples setup at 1Hz output (requires 8s)
int32_t nau7802_getAverage(uint8_t averageAmount)
{
	int32_t total = 0;
	uint8_t samplesAquired = 0;

	nau7802_disable_irq();
	nau7802_internal_irq_enable();

	//unsigned long startTime = millis();
	uiTmrPsj = NAU7802_AVERAGE_TIMEOUT;
	
	while (1)
	{
		total += get_irq_data();
		if (++samplesAquired == averageAmount)
		{
			break;		//All done
		}			
		if (uiTmrPsj == 0)
		{
			return (-1);		//Timeout - Bail with error
		}
		//espera_ms(1);
	}
	
	nau7802_internal_irq_disable();
	nau7802_enable_irq();
	
	total /= averageAmount;

	
	return (total);
}

//Call when scale is setup, level, at running temperature, with nothing on it
int32_t nau7802_calculateZeroOffset(uint8_t averageAmount)
{
	
	nau7802_setZeroOffset(nau7802_getAverage(averageAmount));
	
	return _zeroOffset;
}


//Sets the internal variable. Useful for users who are loading values from NVM.
void nau7802_setZeroOffset(int32_t newZeroOffset)
{
	_zeroOffset = newZeroOffset;
}


int32_t nau7802_getZeroOffset()
{
	return (_zeroOffset);
}


//Call after zeroing. Provide the float weight sitting on scale. Units do not matter.
void nau7802_calculateCalibrationFactor(float weightOnScale, uint8_t averageAmount)
{
	int32_t onScale = nau7802_getAverage(averageAmount);
	float newCalFactor = (onScale - _zeroOffset) / (float)weightOnScale;
	nau7802_setCalibrationFactor(newCalFactor);
}


//Pass a known calibration factor into library. Helpful if users is loading settings from NVM.
//If you don't know your cal factor, call setZeroOffset(), then calculateCalibrationFactor() with a known weight
void nau7802_setCalibrationFactor(float newCalFactor)
{
	_calibrationFactor = newCalFactor;
}


float nau7802_getCalibrationFactor()
{
	return (_calibrationFactor);
}


//Returns the y of y = mx + b using the current weight on scale, the cal factor, and the offset.
float nau7802_getWeight(bool allowNegativeWeights, uint8_t samplesToTake)
{
	int32_t onScale = nau7802_getAverage(samplesToTake);

	//Prevent the current reading from being less than zero offset
	//This happens when the scale is zero'd, unloaded, and the load cell reports a value slightly less than zero value
	//causing the weight to be negative or jump to millions of pounds
	if (allowNegativeWeights == false)
	{
		if (onScale < _zeroOffset)
		{
			onScale = _zeroOffset;		//Force reading to zero
		}
	}

	float weight = (onScale - _zeroOffset) / _calibrationFactor;
	
	return (weight);
}
#endif

//Set Int pin to be high when data is ready (default)
bool nau7802_setIntPolarityHigh()
{
	return (nau7802_clearBit(NAU7802_CTRL1_CRP, NAU7802_CTRL1)); //0 = CRDY pin is high active (ready when 1)
}


//Set Int pin to be low when data is ready
bool nau7802_setIntPolarityLow()
{
	return (nau7802_setBit(NAU7802_CTRL1_CRP, NAU7802_CTRL1)); //1 = CRDY pin is low active (ready when 0)
}


//Mask & set a given bit within a register
bool nau7802_setBit(uint8_t bitNumber, uint8_t registerAddress)
{
	uint8_t value = nau7802_getRegister(registerAddress);
	value |= (1 << bitNumber);		//Set this bit
	return (nau7802_setRegister(registerAddress, value));
}


//Mask & clear a given bit within a register
bool nau7802_clearBit(uint8_t bitNumber, uint8_t registerAddress)
{
	uint8_t value = nau7802_getRegister(registerAddress);
	value &= ~(1 << bitNumber);		//Set this bit
	return (nau7802_setRegister(registerAddress, value));
}


//Return a given bit within a register
bool nau7802_getBit(uint8_t bitNumber, uint8_t registerAddress)
{
	uint8_t value = nau7802_getRegister(registerAddress);
	value &= (1 << bitNumber);		//Clear all but this bit
	return (bool)value;
}


//Get contents of a register
uint8_t nau7802_getRegister(uint8_t registerAddress)
{
	uint8_t val = 0;
	
	if(I2C_byte_read(registerAddress, &val)){
		
		return val;
	}
	
	return (-1);		//Error
}


//Send a given value to be written to given address
//Return true if successful
bool nau7802_setRegister(uint8_t registerAddress, uint8_t value)
{
	return I2C_byte_transfer(registerAddress, value);
}

bool nau7802_drdy_cb_register(const uint32_t pin, ext_irq_cb_t cb){
	
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

int32_t nau7802_internal_irq_enable(void){
	
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

int32_t nau7802_internal_irq_disable(void){
	
	return nau7802_disable_irq();
}

bool nau7802_check_chip_id(void){
	
	return (nau7802_getRegister(NAU7802_DEVICE_REV) == NAU7802_ID);
}

#ifdef CONFIG_NAU7802_TEMP
/*
*	Sensor de temperatura:	
*		Salida: 109 mV a 25°C
*		Delta: 360 uV/°C (relativo 25°c)
*											*/
bool nau7802_temperature_input_set(bool tmp_sens){

	bool ret = false;

	if(tmp_sens){

		ret = nau7802_setGain(NAU7802_GAIN_1);

		ret |= nau7802_setBit(NAU7802_PU_I2C_CTRL_TS, NAU7802_I2C_CONTROL);
	}else{

		ret = nau7802_setGain(gain_mode);

		ret |= nau7802_clearBit(NAU7802_PU_I2C_CTRL_TS, NAU7802_I2C_CONTROL);
	}

	ret |= nau7802_calibrateAFE();

	return ret;
}

int32_t nau7802_temperature_get(uint8_t average_samples){

	int32_t temp_accum = 0;
	int32_t temp_aux;

	nau7802_temperature_input_set(true);

	nau7802_disable_irq();

	irq_drdy_flg = false;

	for(uint8_t i = 0; i < average_samples; i++){

		while(!irq_drdy_flg);

		temp_accum += irq_data_raw;

		irq_drdy_flg = false;
	}
	
	temp_aux = (int32_t)(temp_accum / average_samples);	//Promedio adc

	temp_aux = (int32_t)((SHIFT_TEMP(temp_aux) * ldo_mode) / (NAU7802_MAX_CNT >> 8)); //Temperatura en mv
	temp_aux = (temp_aux - NAU7802_TEMP_REF_MV) * 1000; //delta temp en uv
	temp_aux = (int32_t)(temp_aux / NAU7802_TEMP_DELTA_UV); //delta temp en °C

	temp_aux = NAU7802_TEMP_REF_C + temp_aux;				//Temperatura en °C
	
	nau7802_enable_irq();

	return temp_aux;
}
#endif