/*
 * ADC_NAU7802_TEST.c
 *
 * Created: 6/7/2022 12:30:39
 *  Author: mmedina
 */ 
#ifdef CUSTOM_MAIN
#include "SRC/Testing/test_conf.h"

#ifdef ADC_NAU7802_TEST

//----------------------------------------------------------------------
//	INCLUDES
//----------------------------------------------------------------------
#include <atmel_start.h>
#include <hri_port_d20.h>
#include <hal_io.h>
#include <hal_i2c_m_sync.h>

#include "../RTT/rtt_log.h"
#include "../NAU7802/ADC_NAU7802.h"
#include "SRC/Include/PS_IO.h"
#include "SRC/Drivers/Drv_TC.h"
#include "SRC\Principal\Estados.h"
#include "SRC\Drivers\Drv_SYSTICK.h"
#include "SRC\Drivers\Drv_FLASH.h"
#include "SRC\Drivers\Drv_UART.h"
#include "SRC\Drivers\Drv_WDT.h"
#include "SRC\Pesaje\Pesaje.h"

//----------------------------------------------------------------------
//	Defines
//----------------------------------------------------------------------
//#define ADC_OUTPUT_DATA
//#define ADC_SI_TEST

#define ADC_TOGGLE_MS 1500

/* LOGGING MODULE REGISTER */
#define TEST_RTT_TERMINAL 0
#define TEST_RTT_LEVEL 0

RTT_LOG_REGISTER(nau7802_test, TEST_RTT_LEVEL, TEST_RTT_TERMINAL);

//----------------------------------------------------------------------
//	Global variables
//----------------------------------------------------------------------
static volatile bool drdy_evnt_flag = false;
static uint8_t avrg_samp_tot = 1;
static uint8_t samp_bit_shift = 0;
static bool irq_on = false;

//----------------------------------------------------------------------
//	Interrupciones
//----------------------------------------------------------------------

void HardFault_Handler(void){
	
	RTT_LOG("Hard Fault.\n Reset");
	
	NVIC_SystemReset();
}

//----------------------------------------------------------------------
//	Local functions
//----------------------------------------------------------------------

void adc_drdy_handler(void){

	nau7802_disable_irq();
	
 	drdy_evnt_flag = true;	
}

void tc_1_cb(const struct timer_task *const timer_task){

	if(uiTmr>0) {uiTmr--;}		//Uso: EstadosPpal		//*** Actualiza temporizador de uso general ***
	if(uiTmr2>0){uiTmr2--;}		//Uso: EstadosPpal		//*** Actualiza temporizador de uso general ***
	if(uiTmr3>0){uiTmr3--;}		//Uso: Libreria			//*** Actualiza temporizador de uso general ***	
	if(ucTmr4>0){ucTmr4--;}		//Uso: ReBeep			//*** Actualiza temporizador de uso general ***
	if(uiTmrPsj>0){uiTmrPsj--;}
}

static uint8_t rtt_print_menu(void){
	
	uint8_t key = 0xff;
	
	RTT_WAIT_LOG();

	RTT_PRINT("\n");
	RTT_LOG("## Nau7802 Test menu. ##\n");
	RTT_LOG("1- Start Sampling.\n");
	RTT_LOG("2- Take average n samples.\n");
	RTT_LOG("3- Set speed.\n");
	RTT_LOG("4- Set gain.\n");
	RTT_LOG("5- Set LDO voltage.\n");
	RTT_LOG("6- Set n bit shift. \n");
	RTT_LOG("7- Set num average samples.\n");
	RTT_LOG("8- Do internal calibration.\n");
	RTT_LOG("9- Toggle irq.\n");
	RTT_LOG("Enter option: ");

	RTT_WAIT_LOG();
	
	do{

		key = SEGGER_RTT_WaitKey();	
	}while(key < 0x30 || key > 0x39);
	
	RTT_PRINT("%c.\n", key);
	
	RTT_CLEAR_RX();

	return key;
}

static void nau7802_test_set_ldo(void){
	
	uint8_t key = 0xFF;
	
	RTT_WAIT_LOG();

 	RTT_LOG("1- 4v5.\n");		//ADC alimentado con 3v3 ==> ldo max 3v0
 	RTT_LOG("2- 4v2.\n");
 	RTT_LOG("3- 3v9.\n");
 	RTT_LOG("4- 3v7.\n");
 	RTT_LOG("5- 3v3.\n");
	RTT_LOG("6- 3v0.\n");
	RTT_LOG("7- 2v7.\n");
	RTT_LOG("8- 2v4.\n");
	RTT_LOG("9- LDO off.\n");
	RTT_LOG("Enter LDO mode: ");
	
	key = SEGGER_RTT_WaitKey();
	
	RTT_PRINT("%c.\n", key);
	
	key -= 0x31;
	
	if(key >= 0 && key <= 7){
		
		nau7802_setLDO(key);

		RTT_LOG("LDO on.\n");
	}else if(key == 8){
		//LDO off
		nau7802_clearBit(NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL);

		RTT_LOG("LDO off.\n");
	}	
	RTT_CLEAR_RX();
}

static void nau7802_test_set_gain(void){
	uint8_t key = 0xFF;
	
	RTT_WAIT_LOG();

	RTT_LOG("1- 1 GAIN.\n");
	RTT_LOG("2- 2 GAIN.\n");
	RTT_LOG("3- 4 GAIN.\n");
	RTT_LOG("4- 8 GAIN.\n");
	RTT_LOG("5- 16 GAIN.\n");
	RTT_LOG("6- 32 GAIN.\n");
	RTT_LOG("7- 64 GAIN.\n");
	RTT_LOG("8- 128 GAIN.\n");
	RTT_LOG("Enter gain: ");
	
	key = SEGGER_RTT_WaitKey();
	
	RTT_PRINT("%c.\n", key);
	
	key -= (1 + 0x30);
	
	if(key >= 0 && key <= 7){
		
		nau7802_setGain(key);
		
		key = nau7802_getRegister(NAU7802_CTRL1);
		
		RTT_LOG("PGA GAIN: %d.\n", 1 << (key & 0x7));	
	}
	
	RTT_CLEAR_RX();
}

static bool nau7802_test_speed(uint16_t sps)
{
	uint32_t err_rel = 0;
	uint16_t t_ref = 20000 / sps;

	uiTmr = t_ref * 2;

	drdy_evnt_flag = false;

	nau7802_getReading();

	nau7802_enable_irq();

	for (uint8_t i = 0; i < 20; i++)
	{
		while(!drdy_evnt_flag);
		drdy_evnt_flag = false;
		nau7802_getReading();
		nau7802_enable_irq();
	}
	
	err_rel = ((uiTmr - t_ref) * 100) / t_ref;
	
	if(err_rel <= 10){ //10% de error
		return true;
	}
	return false;
}

static void nau7802_test_set_speed(void){
	
	uint8_t key = 0xFF;
	bool ret = false;

	RTT_WAIT_LOG();

	RTT_LOG("1- 10 SPS.\n");
	RTT_LOG("2- 20 SPS.\n");
	RTT_LOG("3- 40 SPS.\n");
	RTT_LOG("4- 80 SPS.\n");
	RTT_LOG("5- 320 SPS.\n");
	RTT_LOG("Enter sample rate: ");
	
	key = SEGGER_RTT_WaitKey();
	
	RTT_PRINT("%c.\n", key);
	
	key -= (1 + 0x30);
	
	if(key < 0 || key > 4){return;}
		
	switch(key){
		
		case 0:
		case 1:
		case 2:
		case 3:
		
		nau7802_setSampleRate(key);

		nau7802_calibrateAFE();

		
		ret = nau7802_test_speed((1 << (key & 0x7)) * 10);

		RTT_LOG("ADC speed: %d ==> %s.\n", (1 << (key & 0x7)) * 10, RTT_CHECK(ret));
		break;
		case 4:
		
		nau7802_setSampleRate(NAU7802_SPS_320);

		ret = nau7802_test_speed(320);

		RTT_LOG("ADC speed: 320 ==> %s.\n", RTT_CHECK(ret));
		break;
		default:
		
		nau7802_setSampleRate(NAU7802_SPS_40);

		RTT_LOG("ADC speed default: 40.\n");
		break;
	}	

	RTT_CLEAR_RX();
}

static void nau7802_test_get_int_cal(void){
	int32_t val = 0;
	uint8_t val_reg = 0;
	
	val_reg = nau7802_getRegister(0x03);
	val = val_reg << 16;
	val_reg = nau7802_getRegister(0x04);
	val |= val_reg << 8;
	val_reg = nau7802_getRegister(0x05);
	val |= val_reg;
	RTT_LOG("Offset interno: %d. \n", val);
	
	val = 0;
	val_reg = nau7802_getRegister(0x06);
	val = val_reg << 24;
	val_reg = nau7802_getRegister(0x07);
	val |= val_reg << 16;
	val_reg = nau7802_getRegister(0x08);
	val |= val_reg << 8;
	val_reg = nau7802_getRegister(0x09);
	val |= val_reg;
	RTT_LOG("Ganancia interna: %d. \n", (val & 0x800000) >> 23);	
}

static void nau7802_test_get_n_average(void){
	
	char n_samp[4];
	uint8_t key = 0;
	uint8_t i = 0;
	uint16_t n_samples = 0;
	int32_t average = 0;
	
	while(SEGGER_RTT_GetKey() >= 0);
				
	RTT_LOG("Enter N samples: ");
				
	do{
		
		key = SEGGER_RTT_WaitKey();
		
		n_samp[i++] = key;
		
	}while(key != '\n' && i < 3);
		
	n_samples = atoi(n_samp);
	
	average = nau7802_getAverage(n_samples);
	
	RTT_LOG_CONSOLE(1, "Average %d: %d.\n", n_samples, average >> samp_bit_shift);
}

// static uint8_t sampling_menu(void){

// 	uint8_t key = 0xFF;
	
// 	RTT_WAIT_LOG();

// 	RTT_LOG("1- ADC sampling.\n");
// 	RTT_LOG("2- Temperature sampling.\n");
// 	RTT_LOG("Enter mode: ");
	
// 	key = SEGGER_RTT_WaitKey();
	
// 	RTT_PRINT("%c.\n", key);
	
// 	key -= (1 + 0x30);
	
// 	RTT_CLEAR_RX();

// 	return key;
// }

static int32_t nau7802_test_shift_macro(uint8_t n_shift){

	switch (n_shift)
	{
	case 0:
		return DATA_24_BIT(nau7802_getReading());
		break;
	case 1:
		return DATA_23_BIT(nau7802_getReading());
		break;
	case 2:
		return DATA_22_BIT(nau7802_getReading());
		break;
	case 3:
		return DATA_21_BIT(nau7802_getReading());
		break;
	case 4:
		return DATA_20_BIT(nau7802_getReading());
		break;
	case 5:
		return DATA_19_BIT(nau7802_getReading());
		break;
	case 6:
		return DATA_18_BIT(nau7802_getReading());
		break;
	default:
		return DATA_24_BIT(nau7802_getReading());
		break;
	}
}

static void nau7802_test_start_sampling(void){
	
	uint32_t med = 0;
	uint8_t val_off = 0;
	uint8_t key = 0; //sampling_menu();
	int32_t temp = 0;

	if(key > 1) return;

	RTT_LOG("Sampling, enter Q to stop. \n");

	switch (key)
	{
	case 0:

		drdy_evnt_flag = false;

		nau7802_getReading();

		if(irq_on){

			nau7802_enable_irq();
		}
		while(val_off != 'q' && val_off != 'Q'){
		
			med = 0;
			for(uint8_t i = 0; i < avrg_samp_tot; i++){
				//Espera interrupcion
				while(!drdy_evnt_flag){
					val_off = SEGGER_RTT_GetKey();

					if(val_off == 'q' || val_off == 'Q') return;
				}
			
				drdy_evnt_flag = false;
			
				med += nau7802_test_shift_macro(samp_bit_shift);
				//med += nau7802_getReading() >> samp_bit_shift;
			
				nau7802_enable_irq();
			}
			RTT_LOG_CONSOLE(1, "Average: %d \n", (med / avrg_samp_tot));
		
			val_off = SEGGER_RTT_GetKey();
		}	
		break;
	case 1:
		while(val_off != 'q' && val_off != 'Q'){

			temp = nau7802_temperature_get(20);

			RTT_LOG("Temperature: %d.\n", temp);

			val_off = SEGGER_RTT_GetKey();
		}
	break;
	default:
		break;
	}
}

int main (void)
{
	bool ret = 0;
	int32_t val = 0;
	uint8_t val_off = 0;
	char rtt_buff[5] = {0};
	
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	//WDT init
	iniciar_wdt();
	//// Activa VA5V
	gpio_set_pin_level(_VA5V_EN, true);		//1: On
	////
	//LED ON
	LED1 = 0;

	//Timer init
	iniciar_timer1(1000);
	iniciar_timer0(1000000);
	
	//WDT init
	iniciar_wdt();
	
	//UART init
	iniciar_uart();
	habilitar_interrupcion_uart();	
	//FLASH init
	flash_items_init();

	// ADC config
	if((ret = nau7802_begin(true)) != true){
		
		RTT_LOG("ADC init error. \n");
		
	    return -1;
	}
	RTT_LOG("NAU7802 init: %s. \n", RTT_CHECK(ret));
	
	ret = nau7802_start_conversions();
	RTT_LOG("NAU7802 start: %s. \n", RTT_CHECK(ret));
	
	ret = nau7802_check_chip_id();
	RTT_LOG("Chip ID: %s. \n", RTT_CHECK(ret));

	//val = nau7802_calculateZeroOffset(10);
	RTT_LOG("Celda offset: %ld. %s.\n\n", val, (val >= 0) ? RTT_B_GREEN(RTT_BLACK("OK")) : RTT_B_RED(RTT_BLACK("Error")));
			
	RTT_LOG("IRQ test.\n");
	ret = nau7802_drdy_cb_register(ADC24_INT_PIN, adc_drdy_handler);
	RTT_LOG("IRQ callback: %s. \n", RTT_CHECK(ret));
	
	irq_on = true;

	nau7802_test_get_int_cal();

	for(;;)
	{
		val_off = rtt_print_menu();
		
		switch(val_off){
			
			case '1':
							
				nau7802_test_start_sampling();
						
				RTT_LOG_CONSOLE(1, "Ending sampling.\n");		
			break;
			case '2':
			
				nau7802_test_get_n_average();
			break;			
			case '3':
				nau7802_test_set_speed();
				
				//nau7802_calibrateAFE();		
			break;
			case '4':
				nau7802_test_set_gain();
				
				//nau7802_calibrateAFE();
			break;
			case '5':
				nau7802_test_set_ldo();
				
				//nau7802_calibrateAFE();
			break;
			case '6':
				RTT_LOG("Enter total samples bits to shift: ");
				
				while(SEGGER_RTT_ReadNoLock(0, rtt_buff, 5) <= 1);
				
				RTT_PRINT("%s.\n", rtt_buff);
				
				val_off = atoi(rtt_buff);
				
				samp_bit_shift = val_off;			
			break;
			case '7':
				RTT_LOG("Enter total samples for average: ");
				
				while(SEGGER_RTT_ReadNoLock(0, rtt_buff, 5) <= 1);
				
				RTT_PRINT("%s.\n", rtt_buff);
						
				val_off = atoi(rtt_buff);
				
				avrg_samp_tot = val_off;
			break;
			case '8':
			
				ret = nau7802_calibrateAFE();
				
				RTT_LOG("Internal calibration: %s. \n", RTT_CHECK(ret));	
				
				nau7802_test_get_int_cal();	
			break;
			case '9':

				if(irq_on){

					ret = nau7802_disable_irq();

					RTT_LOG("IRQ disable: %s.\n", RTT_CHECK(ret == 0));
				}else{

					ret = nau7802_enable_irq();

					RTT_LOG("IRQ enable: %s.\n", RTT_CHECK(ret == 0));
				}
				irq_on = !irq_on;
			break;
			default:
			break;			
		}
	}
}
#endif
#endif