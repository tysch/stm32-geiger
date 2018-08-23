#include "stm32_lib/stm32f10x.h"
#include "stm32_lib/stm32f10x_conf.h"
#include "stm32_lib/system_stm32f10x.h"

#include "led_display.h"
#include "driver_5110_lcd.h"
#include "display.h"
#include "xprintf.h"
#include <stdint.h>

//#include "stm32f10x_gpio.h"
//#include "stm32f10x_rcc.h"
//#include "stm32f10x_usart.h"
//#include "stm32f10x_exti.h"
//#include "stm32f10x_tim.h"

#include "filters.h"
#include "adc.h"



void beep_init(void)
{
	GPIO_InitTypeDef GPIOC_Init;

	GPIOC_Init.GPIO_Pin = GPIO_Pin_13 |GPIO_Pin_14;
	GPIOC_Init.GPIO_Speed = GPIO_Speed_10MHz;
	GPIOC_Init.GPIO_Mode = GPIO_Mode_Out_PP;

	RCC_HSICmd(DISABLE);
	RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9);
	RCC_PLLCmd(ENABLE);
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_Init(GPIOC,&GPIOC_Init);

//	GPIOx->BSRR = ;

	GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET); 
	GPIO_WriteBit(GPIOC, GPIO_Pin_14, Bit_RESET); 
}

void beep(void)
{
//	static char x = 0;
	//GPIOC->BSRR =~GPIOC->BSRR;
//if(x) {
//	GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET); 
//	GPIO_WriteBit(GPIOC, GPIO_Pin_14, Bit_RESET);
//	x = 0;
//	}
//	else
//	{
//	GPIO_WriteBit(GPIOC, GPIO_Pin_14, Bit_SET); 
//	GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
//	x = 1;
//	}

	 GPIOC->ODR ^= GPIO_Pin_13;
	 GPIOC->ODR ^= GPIO_Pin_14;

}



void SetSysClockTo72(void)
{
    ErrorStatus HSEStartUpStatus;
    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
    /* RCC system reset(for debug purpose) */
    RCC_DeInit();
 
    /* Enable HSE */
    RCC_HSEConfig( RCC_HSE_ON);
 
    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
 
    if (HSEStartUpStatus == SUCCESS)
    {
        /* Enable Prefetch Buffer */
        //FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);
 
        /* Flash 2 wait state */
        //FLASH_SetLatency( FLASH_Latency_2);
 
        /* HCLK = SYSCLK */
        RCC_HCLKConfig( RCC_SYSCLK_Div1);
 
        /* PCLK2 = HCLK */
        RCC_PCLK2Config( RCC_HCLK_Div1);
 
        /* PCLK1 = HCLK/2 */
        RCC_PCLK1Config( RCC_HCLK_Div2);
 
        /* PLLCLK = 8MHz * 9 = 72 MHz */
        RCC_PLLConfig(0x00010000, RCC_PLLMul_9);
 
        /* Enable PLL */
        RCC_PLLCmd( ENABLE);
 
        /* Wait till PLL is ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }
 
        /* Select PLL as system clock source */
        RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK);
 
        /* Wait till PLL is used as system clock source */
        while (RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }
    else
    { /* If HSE fails to start-up, the application will have wrong clock configuration.
     User can add here some code to deal with this error */
 
        /* Go to infinite loop */
        while (1)
        {
        }
    }
}
 
uint32_t number = 0;
 
volatile uint8_t FLAG_ECHO = 0;
volatile uint16_t SonarValue;
 

void sonar_init() {

    /* Timer TIM3 enable clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 
    /* Timer TIM3 settings */
    TIM_TimeBaseInitTypeDef timer_base;
    TIM_TimeBaseStructInit(&timer_base);
    timer_base.TIM_CounterMode = TIM_CounterMode_Up;
    timer_base.TIM_Prescaler = 7200;
    TIM_TimeBaseInit(TIM3, &timer_base);
    TIM_Cmd(TIM3, ENABLE);
 
    //EXTI
 
    /* Set variables used */
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
 
    /* Enable clock for AFIO */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
 
    /* Set pin as input */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
 
    /* Add IRQ vector to NVIC */
    /* PB0 is connected to EXTI_Line0, which has EXTI0_IRQn vector */
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    /* Set priority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
    /* Set sub priority */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
    /* Enable interrupt */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStruct);
 
    /* Tell system that you will use PB0 for EXTI_Line0 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
 
    /* PD0 is connected to EXTI_Line0 */
    EXTI_InitStruct.EXTI_Line = EXTI_Line0;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising and falling edge */
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStruct);
}
 

uint32_t avgs1;
uint32_t avgs2;

void EXTI0_IRQHandler(void) {
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) != 0) {
            // Rising
  //          SonarValue = TIM_GetCounter(TIM3);

//		number = 10000 /  SonarValue ;
//
//		avgs1 *= 5;

//		avgs1 += number;

//		avgs1 /= 6;

//avgs2 *= 99;
//avgs2 += number;
//avgs2 /= 100;

number++;
beep();

TIM_Cmd(TIM2, ENABLE); // LED BLINK
TIM_SetCounter(TIM2, 0);
GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_SET);



	//	update_display(1, 100, avgs1, 2, avgs2, 5);
//TIM_SetCounter(TIM3, 0);

        }
 
        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

unsigned int sonar_get() {
    return (unsigned int)SonarValue;
}
 
int ticks = 0;

int prevnum;

void TIM4_IRQHandler(void)
{
        if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
        {

			FLAG_ECHO = 1;
			++ticks;
	update_display(1, 100, (7*number)/ticks, 2, MedianFilter(7*(number - prevnum)), 30);






prevnum = number;
            TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        }
}




void led_init(void)
{
  // TIMER2 Двічі за секунду викликає sonar_start(); і встановлює FLAG_ECHO = 1;
    TIM_TimeBaseInitTypeDef TIMER_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseStructInit(&TIMER_InitStructure);
    TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIMER_InitStructure.TIM_Prescaler = 7200;
    TIMER_InitStructure.TIM_Period = 100;
    TIM_TimeBaseInit(TIM2, &TIMER_InitStructure);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
 
    /* NVIC Configuration */
    /* Enable the TIM2_IRQn Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);




	GPIO_InitTypeDef GPIOC_Init;

	GPIOC_Init.GPIO_Pin = GPIO_Pin_15;
	GPIOC_Init.GPIO_Speed = GPIO_Speed_10MHz;
	GPIOC_Init.GPIO_Mode = GPIO_Mode_Out_PP;

//	RCC_HSICmd(DISABLE);
//	RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9);
//	RCC_PLLCmd(ENABLE);
//	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_Init(GPIOC,&GPIOC_Init);


}

void TIM2_IRQHandler(void)
{
        if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
        {
			GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_RESET);
			TIM_Cmd(TIM2, DISABLE);
            TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        }
}



int main(void)
{
 
display_init();
beep_init();
led_init();
    SetSysClockTo72();
	adc_init();
	
 
    // TIMER4 Двічі за секунду викликає sonar_start(); і встановлює FLAG_ECHO = 1;
    TIM_TimeBaseInitTypeDef TIMER_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_TimeBaseStructInit(&TIMER_InitStructure);
    TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIMER_InitStructure.TIM_Prescaler = 7200;
    TIMER_InitStructure.TIM_Period = 10000;
    TIM_TimeBaseInit(TIM4, &TIMER_InitStructure);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
 
    /* NVIC Configuration */
    /* Enable the TIM4_IRQn Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
 
    sonar_init();
	
 
	LCD5110_init();
	LCD5110_set_XY(0,0);
	LCD5110_write_string("Hello World");

	while(1)
	{
		display_number(expfilter(get_adc_value()), 0);	
	}
}
