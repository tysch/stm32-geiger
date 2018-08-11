#include "stm32_lib/stm32f10x.h"
#include "stm32_lib/stm32f10x_conf.h"
#include "stm32_lib/system_stm32f10x.h"

#include "led_display.h"
//#include "LCD.h"
#include "driver_5110_lcd.h"
#include "xprintf.h"

//#include <string.h>
//#include <stdlib.h>
#include <stdint.h>


#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_tim.h"















void strcat(char * dest, const char * src)
{
	while(*dest) dest++;
	while((*dest++ = *src++));
}










void format_readings(char * prefix, char * valstr, uint32_t val)
{
	char mju = 'z' + 3;
    // Set range indication for main reading
	if(val >= 100000000)
	{
		int r = val/1000000;
		xsprintf(prefix, " "); 
		xsprintf(valstr, "%4d", r);
		return;
	}
	if((val >= 10000000) && (val <= 99999999))
	{
		int r = val/1000000;
		int mr = (val % 1000000) / 100000;
		xsprintf(prefix, " "); 
		xsprintf(valstr, "%2d.%01d", r, mr);
		return;
	}
	if((val >= 1000000) && (val <= 9999999))
	{
		int r = val/1000000;
		int mr = (val % 1000000) / 10000;
		xsprintf(prefix, " "); 
		xsprintf(valstr, "%d.%02d", r, mr);
		return;
	}
	if((val >= 100000) && (val <= 999999))
	{
		int mr = val/1000;
		xsprintf(prefix, "m"); 
		xsprintf(valstr, "%4d", mr);
		return;
	}
	if((val >= 10000) && (val <= 99999))
	{
		int mr = val/1000;
		int fr_mr = (val % 1000) / 100;
		xsprintf(prefix, "m"); 
		xsprintf(valstr, "%2d.%01d", mr, fr_mr);
		return;
	}
	if((val >= 1000) && (val <= 9999))
	{
		int mr = val/1000;
		int fr_mr = (val % 1000) / 10;
		xsprintf(prefix, "m"); 
		xsprintf(valstr, "%d.%02d", mr, fr_mr);
		return;
	}
	if(val < 1000) 
	{
		xsprintf(valstr, "%4d", val);
		xsprintf(prefix, "%c", mju); 
		return;
	}
}









char str[80];
char tmpstr[80];
char main_val_str[80];
char sec_val_str_prefix[5];

uint32_t number = 0;

char battery = 'z' + 4;

char plus_minus = 'z' + 2 ;


void update_display(char sound, char battery_charge, uint32_t main_val, char main_err, uint32_t fast_val, char fast_err)
{
	LCD5110_set_XY(0,0);

	// Set ticks indication
	if(sound)
	{
	    str[0] = 'z' + 5; 
		str[1] = '\0';
	}
	else
	{
	    str[0] = ' '; 
		str[1] = '\0';
	}

	// Set battery indication
	xsprintf(tmpstr, "        %c%3d%%                        ", battery, battery_charge);
	strcat(str, tmpstr);

    // Set range indication for main reading
    format_readings(tmpstr, main_val_str, main_val);
	strcat(str, tmpstr);

	// Place units and error marking
	xsprintf(tmpstr, "R/h          %c", plus_minus); 
	strcat(str, tmpstr);

	// Place main error 
	xsprintf(tmpstr, "%2d%%              ", main_err);
	strcat(str, tmpstr);

	// Set range indication for secondary reading 
	format_readings(sec_val_str_prefix, tmpstr, fast_val);
	strcat(str, tmpstr);
	strcat(str, sec_val_str_prefix);
	xsprintf(tmpstr, "R/h %c%2d%%", plus_minus, fast_err);
	strcat(str, tmpstr);

	LCD5110_write_string(str);

	LCD5110_num_string(main_val_str,9,10);
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
 
void EXTI0_IRQHandler(void) {
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) != 0) {
            // Rising
    //        TIM_SetCounter(TIM3, 0);

		number++;
      //  update_display();



		update_display(1, 100, number, 2, 862197000, 5);






        }
  //      if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 0) {
  //          // Falling
   //         SonarValue = TIM_GetCounter(TIM3);
    //    }
 


        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}



int main(void)
{
display_init();



SetSysClockTo72();




    /* NVIC Configuration */
    /* Enable the TIM4_IRQn Interrupt */
//    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);





    GPIO_InitTypeDef gpio_cfg;
    GPIO_StructInit(&gpio_cfg);
 
    //Trigger Pin
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    gpio_cfg.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio_cfg.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOB, &gpio_cfg);
 
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


	LCD5110_init();
	LCD5110_set_XY(0,0);
	LCD5110_write_string("Hello World");



	while(1)
	{
		//for(int i = 0; i < 1000; i++) 
		display_number(number, 0);	
//		xsprintf(str, "%d\n", number);
//		LCD5110_set_XY(0,0);
//		LCD5110_write_string(str);
		//number++;
	}
}










































