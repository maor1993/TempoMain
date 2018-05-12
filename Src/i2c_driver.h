/*
 * i2c_driver.h
 *
 *  Created on: 24 בפבר׳ 2017
 *      Author: maor
 */

/*note: i'm an asshole so this driver will be
 * designed to work only on i2c1
 */


#ifndef I2C_DRIVER_H_
#define I2C_DRIVER_H_

#include <stdint.h>
#include "stm32f042x6.h"


//stolen cuz im an asshole
#define TIMING_CLEAR_MASK   (0xF0FFFFFFU)  /*!< I2C TIMING clear register Mask */
#define I2C_TIMEOUT_ADDR    (10000U)       /*!< 10 s  */
#define I2C_TIMEOUT_BUSY    (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_DIR     (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_RXNE    (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_STOPF   (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_TC      (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_TCR     (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_TXIS    (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_FLAG    (25U)          /*!< 25 ms */




typedef struct{
	I2C_TypeDef* pDevice_Address;
	I2C_TypeDef sReg_buf;
}i2c_handler_type;



uint8_t i2c_write_blocking(uint8_t* pData,uint8_t nSize,uint32_t nTimeout,uint8_t nSlave_addr);
uint8_t i2c_read_blocking(uint8_t* pData,uint8_t nSize,uint32_t nTimeout,uint8_t nSlave_addr);


#endif /* I2C_DRIVER_H_ */
