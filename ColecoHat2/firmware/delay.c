#include "delay.h"
#include <stm32l0xx_hal.h>

void delay_ms(int ms)
{
    HAL_Delay(ms);
}

void delay_100ms(unsigned char count)
{
    HAL_Delay(count * 100);
}

