#include "stm32_lib/stm32f10x.h"
#include "stm32_lib/stm32f10x_conf.h"
#include "stm32_lib/system_stm32f10x.h"

void beep_init(void)
{
    GPIO_InitTypeDef GPIOC_Init;

    GPIOC_Init.GPIO_Pin = GPIO_Pin_13 |GPIO_Pin_14;
    GPIOC_Init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIOC_Init.GPIO_Mode = GPIO_Mode_Out_PP;

  //  RCC_HSICmd(DISABLE);
  //  RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9);
  //  RCC_PLLCmd(ENABLE);
  //  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_Init(GPIOC,&GPIOC_Init);

    GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET); 
    GPIO_WriteBit(GPIOC, GPIO_Pin_14, Bit_RESET); 
}

void beep(void) // Bridge connected piezo buzzer to pins C13 and C14
{
    GPIOC->ODR ^= (GPIO_Pin_13 | GPIO_Pin_14);
}
