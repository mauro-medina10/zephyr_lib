/*
 * Copyright (c) 2022 Mauro Medina Serangeli
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//----------------------------------------------------------------------
//	Includes
//----------------------------------------------------------------------
#include <stdint.h>

#include <zephyr.h>
#include <errno.h>
#include <drivers/i2c.h>
#include <device.h>

#include "Drv_I2C.h"			// Definiciones de la interface I2C

//----------------------------------------------------------------------
//	I2C - Software Master Mode
//----------------------------------------------------------------------
static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
static uint8_t dev_addr = 0;

//----------------------------------------------------------------------
//	Funci�n:			I2C_init()
//----------------------------------------------------------------------
//	Objetivo:			Inicializa el dispositivo I2C.
//----------------------------------------------------------------------
//	Param de entrada:   1- i2c estructura I2C que se usara para la comunicaion
//						2- addr direccion slave del dispositivo
//						3. addr_len formato de direccion 7 / 10 bit
//----------------------------------------------------------------------
//	Param de salida:	ninguno
//----------------------------------------------------------------------
int32_t I2C_init(int16_t addr)
{
	int32_t ret = 0;

	if (!device_is_ready(i2c_dev)) {
		return (-EBUSY);
	}

	dev_addr = addr;

	return 0;
}

//----------------------------------------------------------------------
//	Funci�n:			I2C_byte_transfer()
//----------------------------------------------------------------------
//	Objetivo:			Escribe un registro en el dispositivo I2C
//----------------------------------------------------------------------
//	Param de entrada:   1- reg direccion del registro a escribir
//						2- dat valor del registro a escribir
//----------------------------------------------------------------------
//	Param de salida:	bool true si se realizo correctamente
//----------------------------------------------------------------------
int32_t I2C_byte_transfer(uint8_t reg_addr, uint8_t data)
{
	struct i2c_msg msgs;
	uint8_t wr_addr[2];

	wr_addr[0] = reg_addr;
	wr_addr[1] = data;

	/* Send the address to read from */
	msgs.buf = &wr_addr;
	msgs.len = 2U;
	msgs.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer(i2c_dev, &msgs, 1, dev_addr);
}

//----------------------------------------------------------------------
//	Funci�n:			I2C_byte_read()
//----------------------------------------------------------------------
//	Objetivo:			Lee un registro por I2C
//----------------------------------------------------------------------
//	Param de entrada:   1- reg direccion del registro a leer
//						2- value buffer donde se guardara el valor 
//----------------------------------------------------------------------
//	Param de salida:	bool true si se realizo correctamente
//----------------------------------------------------------------------
int32_t I2C_byte_read(uint8_t reg_addr, uint8_t *data)
{
	struct i2c_msg msgs[2];
	uint8_t wr_addr = reg_addr;

	/* Send the address to read from */
	msgs[0].buf = &wr_addr;
	msgs[0].len = 1U;
	msgs[0].flags = I2C_MSG_WRITE | BIT(5);

	/* Read from device. STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = 1U;
	msgs[1].flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP;

	return i2c_transfer(i2c_dev, msgs, 2, dev_addr);
}

//----------------------------------------------------------------------
//	Funci�n:			I2C_bit_write()
//----------------------------------------------------------------------
//	Objetivo:			Escribe un bit de un registro en el 
//						dispositivo I2C
//----------------------------------------------------------------------
//	Param de entrada:   1- reg direccion del registro a escribir
//						2- mask mascara del bit a escribir
//						3- valor del bit a escribir
//----------------------------------------------------------------------
//	Param de salida:	bool true si se realizo correctamente
//----------------------------------------------------------------------
int32_t I2C_bit_write(uint8_t reg, uint8_t mask, uint8_t dat){
	
	uint8_t val = 0;
	int32_t ret = I2C_byte_read(reg, &val);

	if(ret != 0){	
		return ret;	
	}
	val &= ~(mask);
	val |= dat;
	
	return I2C_byte_transfer(reg, val);
}
