#include "stm32_lib/stm32f10x.h"
#include "stm32_lib/stm32f10x_conf.h"
#include "stm32_lib/system_stm32f10x.h"

void beep_init(void)
{
    GPIO_InitTypeDef GPIOB_Init;

    GPIOB_Init.GPIO_Pin = GPIO_Pin_12 |GPIO_Pin_13;
    GPIOB_Init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIOB_Init.GPIO_Mode = GPIO_Mode_Out_PP;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_Init(GPIOB, &GPIOB_Init);

    GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET); 
    GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_RESET); 
}

void beep(void) // Bridge connected piezo buzzer to pins C13 and C14
{
    GPIOB->ODR ^= (GPIO_Pin_12 | GPIO_Pin_13);
}
