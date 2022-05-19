# raspberry-optee-i2c
OP-TEE I2C Driver for Raspberry Pi

Refer this article for more information:

https://bit.ly/3Kh2dRT

Also provided video about OP-TEE:

https://youtube.com/playlist?list=PLrwOamjP8JAKZ3hIdH1z3hxvYSq7XoI8d

File locations:

optee_os/core/drivers/bcm2835_i2c.c

optee_os/core/drivers/sub.mk

optee_os/core/include/drivers/bcm2835_i2c.h

To enable I2C Driver add below macro in platform configuration file:
CFG_BCM_I2C

optee_os/core/arch/arm/plat-bcm/conf.mk +28

CFG_BCM_I2C ?= y

or

$(call force,CFG_BCM_I2C,y)


The GPIO Number used for SDA and SCL is GPIO2 and GPIO3 respectively. Currently hardcoded inn driver itself.


