#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_i2c.h>
#include <stm32f0xx_hal_def.h>
#include <stdint.h>






HAL_StatusTypeDef TM_I2C_Write(uint8_t address, uint8_t reg, uint8_t data);
void TM_I2C_WriteMulti(uint8_t address, uint8_t reg, uint8_t* data, uint16_t count);

void TM_GPIO_SetPinAsInput(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) ;

void TM_GPIO_SetPinAsOutput(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) ;
void reverse(char s[]);
void to_ascii(int n, char s[]);

/**
 * @brief  Sets pin(s) low
 * @note   Defined as macro to get maximum speed using register access
 * @param  GPIOx: GPIOx PORT where you want to set pin low
 * @param  GPIO_Pin: Select GPIO pin(s). You can select more pins with | (OR) operator to set them low
 * @retval None
 */
#define TM_GPIO_SetPinLow(GPIOx, GPIO_Pin)			((GPIOx)->BRR = (GPIO_Pin))

/**
 * @brief  Sets pin(s) high
 * @note   Defined as macro to get maximum speed using register access
 * @param  GPIOx: GPIOx PORT where you want to set pin high
 * @param  GPIO_Pin: Select GPIO pin(s). You can select more pins with | (OR) operator to set them high
 * @retval None
 */
#define TM_GPIO_SetPinHigh(GPIOx, GPIO_Pin) 		((GPIOx)->BSRR = (GPIO_Pin))


/**
 * @brief  Gets input data bit
 * @note   Defined as macro to get maximum speed using register access
 * @param  GPIOx: GPIOx PORT where you want to read input bit value
 * @param  GPIO_Pin: GPIO pin where you want to read value
 * @retval 1 in case pin is high, or 0 if low
 */
#define TM_GPIO_GetInputPinValue(GPIOx, GPIO_Pin)	(((GPIOx)->IDR & (GPIO_Pin)) == 0 ? 0 : 1)
