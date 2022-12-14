//----------------------------------------------------------------------
//						KRETZ S.A. 	-	 Balanza PS
//----------------------------------------------------------------------
//	ARCHIVO:	Drv_I2C.h
//----------------------------------------------------------------------
//	OBJETIVO:	Definiciones utilizadas por el driver de control de la
//				interface I2C.
//----------------------------------------------------------------------
//	HISTORIA: 	01-Feb-2014			V2.0			Nicol�s Matteucci
//----------------------------------------------------------------------

#ifndef DRV_I2C_H
#define DRV_I2C_H

//----------------------------------------------------------------------
//	DEFINICIONES
//----------------------------------------------------------------------

//----------------------------------------------------------------------
//	PROTOTIPOS DE FUNCIONES
//----------------------------------------------------------------------	
int32_t I2C_init(int16_t addr);

int32_t I2C_byte_transfer(uint8_t reg_addr, uint8_t data);
int32_t I2C_byte_read(uint8_t reg_addr, uint8_t *data);
int32_t I2C_bit_write(uint8_t reg, uint8_t mask, uint8_t dat);
#endif    // DRV_I2C_H
	
