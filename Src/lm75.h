/*
 * lm75.h
 *
 *  Created on: 17 באפר׳ 2017
 *      Author: maor
 */
#include "stdint.h"
#define LM75_I2C_ADDR 0x9e //note: relevent only to tempo v2 since a2..a0 are held to '1'
uint16_t lm75_get_temp_raw();

