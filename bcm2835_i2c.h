/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022
 * Devendra Devadiga <devendradevadiga01@gmail.com>
 */
#ifndef __DRIVERS_BCM_I2C_H
#define __DRIVERS_BCM_I2C_H

#include <stdint.h>
#include <tee_api_types.h>

#define BCM_GPIO_BASE   0x7E200000
#define BCM_BSC0_BASE   0x7E205000
#define BCM_BSC1_BASE   0x7E804000
#define BCM_BSC2_BASE   0x7E805000

#define BCM_GPIO_GPFSEL(__x)	((__x) * 0x4)

static struct io_pa_va i2c_bus[3] = {
#if defined(BCM_BSC0_BASE)
	[0] = { .pa = BCM_BSC0_BASE, },
#endif
#if defined(BCM_BSC1_BASE)
	[1] = { .pa = BCM_BSC1_BASE, },
#endif
#if defined(BCM_BSC2_BASE)
	[2] = { .pa = BCM_BSC2_BASE, },
#endif
};

static struct io_pa_va bcm_i2c_mux = {
	.base.pa = BCM_GPIO_BASE
};

#define BCM2835_I2C_C		0x0
#define BCM2835_I2C_S		0x4
#define BCM2835_I2C_DLEN	0x8
#define BCM2835_I2C_A		0xc
#define BCM2835_I2C_FIFO	0x10
#define BCM2835_I2C_DIV		0x14
#define BCM2835_I2C_DEL		0x18
#define BCM2835_I2C_CLKT	0x1c

#define BCM2835_I2C_C_READ	BIT(0)
#define BCM2835_I2C_C_CLEAR	BIT(4) /* bits 4 and 5 both clear */
#define BCM2835_I2C_C_ST	BIT(7)
#define BCM2835_I2C_C_INTD	BIT(8)
#define BCM2835_I2C_C_INTT	BIT(9)
#define BCM2835_I2C_C_INTR	BIT(10)
#define BCM2835_I2C_C_I2CEN	BIT(15)

#define BCM2835_I2C_S_TA	BIT(0)
#define BCM2835_I2C_S_DONE	BIT(1)
#define BCM2835_I2C_S_TXW	BIT(2)
#define BCM2835_I2C_S_RXR	BIT(3)
#define BCM2835_I2C_S_TXD	BIT(4)
#define BCM2835_I2C_S_RXD	BIT(5)
#define BCM2835_I2C_S_TXE	BIT(6)
#define BCM2835_I2C_S_RXF	BIT(7)
#define BCM2835_I2C_S_ERR	BIT(8)
#define BCM2835_I2C_S_CLKT	BIT(9)
#define BCM2835_I2C_S_LEN	BIT(10) /* Fake bit for SW error reporting */

#define BCM2835_I2C_FEDL_SHIFT	16
#define BCM2835_I2C_REDL_SHIFT	0

#define BCM2835_I2C_CDIV_MIN	0x0002
#define BCM2835_I2C_CDIV_MAX	0xFFFE
#define BCM2835_I2C_FIFO_SIZE	16
#define BCM2835_I2C_CLR_STAT	(BCM2835_I2C_S_ERR|BCM2835_I2C_S_CLKT|BCM2835_I2C_S_DONE)
#define BCM2835_I2C_CLR_FIFO	BCM2835_I2C_C_CLEAR
#define BCM2835_I2C_START_READ	(BCM2835_I2C_C_I2CEN|BCM2835_I2C_C_ST|BCM2835_I2C_C_CLEAR|BCM2835_I2C_C_READ)
#define BCM2835_I2C_START_WRITE	(BCM2835_I2C_C_I2CEN|BCM2835_I2C_C_ST)

#define I2C_CLOCK_100_kHz   2500	///< 2500 = 10us = 100 kHz
#define I2C_CLOCK_400_kHz   626		///< 622 = 2.504us = 399.3610 kHz
#define I2C_CLOCK_1666_kHz  150		///< 150 = 60ns = 1.666 MHz (default at reset)
#define I2C_CLOCK_1689_kHz  148		///< 148 = 59ns = 1.689 MHz

/* GPIO function selection */
typedef enum {
    GPIO_FSEL_IN,
    GPIO_FSEL_OUT,
    GPIO_FSEL_ALT5,
    GPIO_FSEL_ALT4,
    GPIO_FSEL_ALT0,
    GPIO_FSEL_ALT1,
    GPIO_FSEL_ALT2,
    GPIO_FSEL_ALT3,
} bcm_gpio_fsel;

typedef enum {
	I2C_OK			= 0x00,		///< Success
	I2C_ERR_NACK 	= 0x01,		///< Received a NACK
	I2C_ERR_CLKT 	= 0x02,		///< Received Clock Stretch Timeout
	I2C_ERR_DATA 	= 0x04		///< Not all data is sent / received
} BCM_I2C_Status;

TEE_Result bcm_i2c_write(uint8_t bid, uint32_t dev_addr, uint32_t reg_addr,
                    uint32_t len, const uint8_t *data);

TEE_Result bcm_i2c_set_reg(uint8_t bid, uint32_t dev_addr,
                    uint32_t reg_addr, uint8_t value);

TEE_Result bcm_i2c_read(uint8_t bid, uint32_t dev_addr,
                    uint32_t reg_addr, int32_t len, const uint8_t *data);

TEE_Result bcm_i2c_get_reg(uint32_t dev_addr, uint32_t reg_addr,
                    uint8_t *result);

void bcm_gpio_fun_sel(uint32_t pin, uint32_t func);

TEE_Result bcm_i2c_probe(uint8_t bid, uint8_t chip);

TEE_Result bcm_i2c_init(uint8_t bid, int bps);

#endif /*__DRIVERS_BCM_I2C_H*/
