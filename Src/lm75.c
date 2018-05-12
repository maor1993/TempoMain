/*
 * lm75.c
 *
 *  Created on: 17 באפר׳ 2017
 *      Author: maor
 */
#include "lm75.h"
#include "stdint.h"
#include "i2c_driver.h"


uint16_t lm75_get_temp_raw()
{
	uint8_t buf[2] =
			{0,0};

	//read the temprerture from the sensor.
	i2c_read_blocking(buf,sizeof(buf),1000,LM75_I2C_ADDR);

	return (buf[1]|(buf[0]<<8));
}
