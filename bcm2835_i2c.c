// SPDX-License-Identifier: BSD-2-Clause
/*
 * (c) 2022 Devendra Devadiga <devendradevadiga01@gmail.com>
 */

#include <arm.h>
#include <drivers/bcm2835_i2c.h>
#include <initcall.h>
#include <io.h>
#include <kernel/boot.h>
#include <kernel/delay.h>
#include <kernel/dt.h>
#include <libfdt.h>
#include <mm/core_memprot.h>
#include <mm/core_mmu.h>
#include <platform_config.h>
#include <stdlib.h>
#include <trace.h>
#include <util.h>

static uint32_t i2c_io_read32(uint8_t bid, uint32_t address)
{
	return io_read32(i2c_bus[bid].va + address);
}

static void i2c_io_write32(uint8_t bid, uint32_t address, uint8_t data)
{
	return io_write32(i2c_bus[bid].va + address, data);
}

static void i2c_set_bus_speed(uint8_t bid, int bps)
{
	uint32_t val = 0;
	bps = bps/1000;

	/* configure clock devider based on bps value */
	switch (bps) {
	case 100:
		DMSG("I2C Speed: 100 Kbps");
		val = I2C_CLOCK_100_kHz;
		break;
	case 400:
		DMSG("I2C Speed: 400 Kbps");
		val = I2C_CLOCK_400_kHz;
		break;
	case 1666:
		DMSG("I2C Speed: 1666 Kbps");
		val = I2C_CLOCK_1666_kHz;
		break;
	case 1689:
		DMSG("I2C Speed: 1666 Kbps");
		val = I2C_CLOCK_1689_kHz;
		break;
	default:
		EMSG("Invalid I2C Speed: %d Kbps", bps);
		panic();
	}

	i2c_io_write32(bid, BCM2835_I2C_DIV, val);
}

/* 
* Function to wait for the I2C transaction to complete
*/
void wait_i2c_done(uint8_t bid)
{
	volatile uint32_t isDone = 0;
    while(1){
		isDone = i2c_io_read32(bid, BCM2835_I2C_S);
        if(!(isDone & BCM2835_I2C_S_DONE)) {
            break;
        } 
    }
}


TEE_Result bcm_i2c_write(uint8_t bid, uint32_t dev_addr, uint32_t reg_addr,
                   uint32_t len, const uint8_t *data)
{
    int32_t i = -1;
    uint32_t rem = len + 1;

	TEE_Result res = TEE_SUCCESS;
    uint32_t status = 0;
	BCM_I2C_Status i2c_status;
	uint8_t val;

	i2c_io_write32(bid, BCM2835_I2C_C, BCM2835_I2C_C_CLEAR);
	i2c_io_write32(bid, BCM2835_I2C_S, BCM2835_I2C_CLR_STAT);
	i2c_io_write32(bid, BCM2835_I2C_A, (dev_addr & 0x7F));
	i2c_io_write32(bid, BCM2835_I2C_DEL, (len + 1));

    while ((0 != rem)  && (i < BCM2835_I2C_FIFO_SIZE)) {
		val = i < 0 ? reg_addr : *(data+i);
		i2c_io_write32(bid, BCM2835_I2C_FIFO, val);
		++i;
		--rem;
	}

	i2c_io_write32(bid, BCM2835_I2C_C, BCM2835_I2C_START_WRITE);

    while (!(i2c_io_read32(bid, BCM2835_I2C_S) & BCM2835_I2C_S_DONE)) {
		while ((0 != rem) && (i2c_io_read32(bid, BCM2835_I2C_S) & BCM2835_I2C_S_TXD)) {
			val = i < 0 ? reg_addr : *(data+i);
			i2c_io_write32(bid, BCM2835_I2C_FIFO, val);
			++i;
			--rem;
    	}
    }

    status = i2c_io_read32(bid, BCM2835_I2C_S);
    if(status & BCM2835_I2C_S_ERR){
		i2c_status = I2C_ERR_NACK;
    } else if (status & BCM2835_I2C_S_CLKT){
		i2c_status = I2C_ERR_CLKT;	
    } else if (0 != rem) {
		i2c_status = I2C_ERR_DATA;	
    }

	if(i2c_status) {
		EMSG("i2c transfer failed: 0x%x\n", i2c_status);
		res = TEE_ERROR_GENERIC;
	}

	i2c_io_write32(bid, BCM2835_I2C_S, BCM2835_I2C_S_DONE);

    return res;
}

TEE_Result bcm_i2c_set_reg(uint8_t bid, uint32_t dev_addr,
                     uint32_t reg_addr, uint8_t value)
{
    return bcm_i2c_write(bid, dev_addr, reg_addr, 1, &value);
}

TEE_Result _read(uint8_t bid, uint8_t *data, uint32_t len)
{
    uint32_t rem = len;
    uint32_t i = 0;
    TEE_Result res = TEE_SUCCESS;
    uint32_t status = 0;
	BCM_I2C_Status i2c_status;

	i2c_io_write32(bid, BCM2835_I2C_C, BCM2835_I2C_C_CLEAR);
	i2c_io_write32(bid, BCM2835_I2C_S, BCM2835_I2C_CLR_STAT);
	i2c_io_write32(bid, BCM2835_I2C_DEL, len);
	i2c_io_write32(bid, BCM2835_I2C_C, BCM2835_I2C_START_READ);

    while (!(i2c_io_read32(bid, BCM2835_I2C_S) & BCM2835_I2C_S_DONE)){
        while((i2c_io_read32(bid, BCM2835_I2C_S) & BCM2835_I2C_S_RXD)){
            *(data+i) = (uint8_t)(i2c_io_read32(bid, BCM2835_I2C_FIFO) & 0xFF);
            ++i;
            --rem;
        }
    }

    while ((0 != rem) && (i2c_io_read32(bid, BCM2835_I2C_S) & BCM2835_I2C_S_RXD)){
        *(data+i) = (uint8_t)(i2c_io_read32(bid, BCM2835_I2C_FIFO) & 0xFF);
        ++i;
        --rem;
    }

    status = i2c_io_read32(bid, BCM2835_I2C_S);
    if(status & BCM2835_I2C_S_ERR){
		i2c_status = I2C_ERR_NACK;
    } else if (status & BCM2835_I2C_S_CLKT){
		i2c_status = I2C_ERR_CLKT;	
    } else if (0 != rem) {
		i2c_status = I2C_ERR_DATA;	
    }

	if(i2c_status) {
		EMSG("i2c read failed: 0x%x\n", i2c_status);
		res = TEE_ERROR_GENERIC;
	}

	i2c_io_write32(bid, BCM2835_I2C_S, BCM2835_I2C_S_DONE);
    
    return res;
}

TEE_Result bcm_i2c_read(uint8_t bid, uint32_t dev_addr,
                  uint32_t reg_addr, int32_t len, uint8_t *data)
{
    TEE_Result res = bcm_i2c_write(bid, dev_addr, reg_addr, 0, 0);
    if(TEE_SUCCESS == res){
        res = _read(bid, data, len);
    }

    return res;
}

TEE_Result bcm_i2c_get_reg(uint32_t dev_addr, uint32_t reg_addr,
                        	uint8_t *result)
{
    return bcm_i2c_read(dev_addr, reg_addr, 1, result);
}

TEE_Result bcm_i2c_probe(uint8_t bid, uint8_t chip)
{
	if (bid >= ARRAY_SIZE(i2c_bus))
		return TEE_ERROR_BAD_PARAMETERS;

	if (!i2c_bus[bid].va)
		return TEE_ERROR_BAD_PARAMETERS;

	if (chip > 0x7F)
		return TEE_ERROR_BAD_PARAMETERS;

	return bcm_i2c_write(bid, chip, 0, 0, NULL);
}

void bcm_gpio_fun_sel(uint32_t pin, uint32_t func) {
	struct io_pa_va *mux = &bcm_i2c_mux;
    int offset = pin / 10;
    uint32_t val = io_read32(mux->base.va + BCM_GPIO_GPFSEL(offset));
    int item = pin % 10;
    val &= ~(0x7 << (item * 3));
    val |= ((func & 0x7) << (item * 3));
	io_write32(mux->base.va + BCM_GPIO_GPFSEL(offset), val));
}  

/*
 * I2C bus initialization: configure the IOMUX and enable the clock.
 * @bid: Bus ID: (0=I2C1), (1=I2C2), (2=I2C3), (3=I2C4).
 * @bps: Bus baud rate, in bits per second.
 */
TEE_Result bcm_i2c_init(uint8_t bid, int bps)
{

	if (bid >= ARRAY_SIZE(i2c_bus))
		return TEE_ERROR_BAD_PARAMETERS;

	if (!bps)
		return TEE_ERROR_BAD_PARAMETERS;

	if (!i2c_bus[bid].va)
		return TEE_ERROR_BAD_PARAMETERS;

    bcm_gpio_fun_sel(2, GPIO_FSEL_IN);
    bcm_gpio_fun_sel(3, GPIO_FSEL_IN);
    
    bcm_gpio_fun_sel(2, GPIO_FSEL_ALT0);
    bcm_gpio_fun_sel(3, GPIO_FSEL_ALT0);

	/* Baud rate in bits per second */
	i2c_set_bus_speed(bid, bps);

	return TEE_SUCCESS;
}

static TEE_Result get_va(paddr_t pa, vaddr_t *va)
{
	*va = (vaddr_t)core_mmu_add_mapping(MEM_AREA_IO_SEC, pa, 0x1000);
	if (!*va)
		return TEE_ERROR_GENERIC;

	return TEE_SUCCESS;
}

static TEE_Result i2c_map_controller(void)
{
	TEE_Result ret = TEE_ERROR_GENERIC;
	size_t n = 0;

	for (n = 0; n < ARRAY_SIZE(i2c_bus); n++) {
		if (i2c_bus[n].pa) {
			if (get_va(i2c_bus[n].pa, &i2c_bus[n].va))
				EMSG("i2c%zu not enabled", n + 1);
			else
				ret = TEE_SUCCESS;
		} else {
			IMSG("i2c%zu not enabled", n + 1);
		}
	}

	return ret;
}

static TEE_Result i2c_init(void)
{
	if (get_va(bcm_i2c_mux.base.pa, &bcm_i2c_mux.base.va))
		return TEE_ERROR_GENERIC;

	return i2c_map_controller();
}

early_init(i2c_init);
