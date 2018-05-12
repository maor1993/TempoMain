#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_i2c.h>
#include <stm32f0xx_hal_def.h>
#include <stm32f0xx_hal_gpio.h>
#include "util.h"
#include <string.h>


HAL_StatusTypeDef TM_I2C_Write(uint8_t address, uint8_t reg, uint8_t data)
{
	uint8_t nData_to_send[2];
		HAL_StatusTypeDef res = HAL_OK;
	nData_to_send[0] = reg;
	nData_to_send[1] = data;


	res = i2c_write_blocking(nData_to_send,sizeof(nData_to_send),10000,address);
	//HAL_Delay(100);
	return res;

}
void TM_I2C_WriteMulti(uint8_t address, uint8_t reg, uint8_t* data, uint16_t count)
{
	uint8_t nData_to_send[128];
HAL_StatusTypeDef res = HAL_OK;
	nData_to_send[0] = reg;
	
	memcpy(nData_to_send+1,data,count);

	res = i2c_write_blocking(nData_to_send,sizeof(nData_to_send)+1,10000,address);

}
/* reverse:  reverse string s in place */
 void reverse(char s[])
 {
     int i, j;
     char c;

     for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
         c = s[i];
         s[i] = s[j];
         s[j] = c;
     }
 }
void to_ascii(int n, char s[])
{
    int i, sign;

    if ((sign = n) < 0)  /* record sign */
        n = -n;          /* make n positive */
    i = 0;
    do {       /* generate digits in reverse order */
        s[i++] = n % 10 + '0';   /* get next digit */
    } while ((n /= 10) > 0);     /* delete it */
    if (sign < 0)
        s[i++] = '-';
    s[i] = '\0';
    reverse(s);
}
