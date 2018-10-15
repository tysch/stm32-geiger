#include "stm32_lib/stm32f10x.h"
#include "stm32_lib/stm32f10x_conf.h"
#include "stm32_lib/system_stm32f10x.h"

// 100ms timer and C15 port for LED flashing upon count
void led_init(void) 
{
    // 100ms timer TIM2 
    TIM_TimeBaseInitTypeDef TIMER_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseStructInit(&TIMER_InitStructure);
    TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    // TIMER_InitStructure.TIM_Prescaler = 7200;
    TIMER_InitStructure.TIM_Prescaler = 800;
    TIMER_InitStructure.TIM_Period = 100;
    TIM_TimeBaseInit(TIM2, &TIMER_InitStructure);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    // IRQ for TIM2
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // C15 port for LED
    GPIO_InitTypeDef GPIOB_Init;
    GPIOB_Init.GPIO_Pin = GPIO_Pin_14;
    GPIOB_Init.GPIO_Speed = GPIO_Speed_10MHz;
    GPIOB_Init.GPIO_Mode = GPIO_Mode_Out_PP;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_Init(GPIOB, &GPIOB_Init);
}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_RESET);
        TIM_Cmd(TIM2, DISABLE);
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

void led_blink(void)
{
    TIM_Cmd(TIM2, ENABLE); // LED BLINK
    TIM_SetCounter(TIM2, 0);
    GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_SET);
}
