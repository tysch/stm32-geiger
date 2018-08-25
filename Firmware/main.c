#include <stdint.h>

#include "stm32_lib/stm32f10x.h"
#include "stm32_lib/stm32f10x_conf.h"
#include "stm32_lib/system_stm32f10x.h"

#include "led_display.h"
#include "driver_5110_lcd.h"
#include "display.h"
#include "xprintf.h"

#include "filters.h"
#include "adc.h"
#include "beep.h"
#include "led_blink.h"


/////////////////////////////////////////////////////////////////////////////////////////////
//
//    Settings
//
/////////////////////////////////////////////////////////////////////////////////////////////


#define NRH_PER_CPS 7000                // multiplier, nR/h per one pulse per second

#define BUTTON_LONG_PRESS_DELAY 2       // in seconds
#define BUTTON_VERY_LONG_PRESS_DELAY 5

#define MAX_VOLTAGE 4200                // Voltage range for battery
#define MIN_VOLTAGE 3000

#define SHUTDOWN_VOLTAGE 3200           // Undervoltage shutdown threshold

/////////////////////////////////////////////////////////////////////////////////////////////
//
//    Global data
//
/////////////////////////////////////////////////////////////////////////////////////////////

static volatile uint32_t seconds = 0;

static volatile uint32_t counts = 0;
static volatile uint32_t prev_counts = 0;

static volatile uint32_t nrh = 0;
static volatile uint8_t  error = 0;

static volatile uint32_t background_nrh = 0;
static volatile uint8_t background_err = 0;

static volatile uint8_t backlight = 0;
static volatile uint8_t is_beeping;
static volatile uint8_t batt_charge;

uint32_t invsqrt(uint32_t n);

/////////////////////////////////////////////////////////////////////////////////////////////
//
//    Initizlizations
//
/////////////////////////////////////////////////////////////////////////////////////////////

void SetSysClockTo72(void)
{
    ErrorStatus HSEStartUpStatus;
    // SYSCLK, HCLK, PCLK2 and PCLK1 configuration 
    // RCC system reset(for debug purpose)
    RCC_DeInit();
    // Enable HSE 
    RCC_HSEConfig(RCC_HSE_ON);
    // Wait till HSE is ready 
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    if (HSEStartUpStatus == SUCCESS)
    {
        // HCLK = SYSCLK
        RCC_HCLKConfig( RCC_SYSCLK_Div1);
        // PCLK2 = HCLK
        RCC_PCLK2Config( RCC_HCLK_Div1);
        // PCLK1 = HCLK/2
        RCC_PCLK1Config( RCC_HCLK_Div2);
        // PLLCLK = 8MHz * 9 = 72 MHz 
        RCC_PLLConfig(0x00010000, RCC_PLLMul_9);
        // Enable PLL 
        RCC_PLLCmd( ENABLE);
        // Wait till PLL is ready 
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }
        // Select PLL as system clock source 
        RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK);
        /// Wait till PLL is used as system clock source 
        while (RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }
    else
    { // If HSE fails to start-up, the application will have wrong clock configuration.
    // User can add here some code to deal with this error 
        while (1)
        {
        }
    }
}

// Fires once per second and updates readings
void init_display_timer(void) 
{
    // Enable TIM4
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
 
    // Enable the TIM4_IRQn Interrupt 
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


// Sets interrupts for pushbuttons on A0 and A2
void pushbuttons_init(void)
{
    //EXTI

    // Set variables used
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    // Enable clock for AFIO 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // Set pin as input 
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD; // Pulldown
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Add IRQ vector to NVIC
    // PB0 is connected to EXTI_Line0, which has EXTI0_IRQn vector 
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    // Set priority
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;
    // Set sub priority
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x03;
    // Enable interrupt
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    // Add to NVIC
    NVIC_Init(&NVIC_InitStruct);

    // Tell system that you will use PA0 for EXTI_Line0
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

    // PD0 is connected to EXTI_Line0
    EXTI_InitStruct.EXTI_Line = EXTI_Line0;
    // Enable interrupt
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    // Interrupt mode 
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    // Triggers on rising and falling edge
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    // Add to EXTI
    EXTI_Init(&EXTI_InitStruct);

    // Add IRQ vector to NVIC
    // PB0 is connected to EXTI_Line2, which has EXTI2_IRQn vector
    NVIC_InitStruct.NVIC_IRQChannel = EXTI2_IRQn;
    // Set priority
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x03;
    // Set sub priority
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x05;
    // Enable interrupt
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    // Add to NVIC
    NVIC_Init(&NVIC_InitStruct);
 
    // Tell system that you will use PA2 for EXTI_Line2
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2);
 
    // PD0 is connected to EXTI_Line2
    EXTI_InitStruct.EXTI_Line = EXTI_Line2;
    // Enable interrupt
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    // Interrupt mode
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    // Triggers on rising and falling edge
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    // Add to EXTI
    EXTI_Init(&EXTI_InitStruct);
}



// Interrupts upon pulse from geiger counter on pin PB1
void geiger_counter_input_init()
{
    // Set variables used
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    // Enable clock for AFIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // Set pin as input
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
 
    // Add IRQ vector to NVIC
    // PB0 is connected to EXTI_Line0, which has EXTI0_IRQn vector
    NVIC_InitStruct.NVIC_IRQChannel = EXTI1_IRQn;
    // Set priority 
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
    // Set sub priority
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
    // Enable interrupt
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    // Add to NVIC
    NVIC_Init(&NVIC_InitStruct);

    // Tell system that you will use PB1 for EXTI_Line1
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);

    // PD0 is connected to EXTI_Line1
    EXTI_InitStruct.EXTI_Line = EXTI_Line1;
    // Enable interrupt
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    // Interrupt mode
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    // Triggers on rising and falling edge
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    // Add to EXTI
    EXTI_Init(&EXTI_InitStruct);
}

/////////////////////////////////////////////////////////////////////////////////////////////
//
//    Pushbuttons actions
//
/////////////////////////////////////////////////////////////////////////////////////////////

// Toggles background correction and resets errors estimation
void subtract_background(void)
{
    if(background_nrh) background_nrh = 0;
    else               background_nrh = NRH_PER_CPS * counts / seconds;

    background_err = invsqrt(counts);
    seconds = 0;
    counts = 0;
    prev_counts = 0;
}

// Short button 1 press, subtracts background
void EXTI0_IRQHandler(void) 
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) 
    {
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) != 0) 
        {
            subtract_background();
        }
        // Clear interrupt flag
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

// Short button2 press, toggle LCD backlight
void EXTI2_IRQHandler(void) 
{
    if (EXTI_GetITStatus(EXTI_Line2) != RESET)
    {
        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) != 0) 
        {
            backlight ^= 0xff;
            LCD5110_Led(backlight); 
        }
        // Clear interrupt flag
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}

//
//         Actions upon long pressing
//

void shutdown(void)
{
    PWR_EnterSTANDBYMode();
}

void reset_count(void)
{
    seconds = 0;
    counts = 0;
    prev_counts = 0;
    background_nrh = 0;
}

void button_1_shutdown(int is_pressed)
{
    static uint8_t cnt = 0;

    if(is_pressed) cnt++;
    else           cnt = 0;

    if(cnt > BUTTON_LONG_PRESS_DELAY) reset_count();
    if(cnt > BUTTON_VERY_LONG_PRESS_DELAY) shutdown();
}

void button_2_toggle_beeping(int is_pressed)
{
    static uint8_t cnt = 0;

    if(is_pressed) cnt++;
    else           cnt = 0;

    if(cnt > BUTTON_LONG_PRESS_DELAY) is_beeping ^= 0xff;
}



/////////////////////////////////////////////////////////////////////////////////////////////
//
//    Main program flow
//
/////////////////////////////////////////////////////////////////////////////////////////////



// Called upon each particle detection
void EXTI1_IRQHandler(void)
{
    // Make sure that interrupt flag is set
    if(EXTI_GetITStatus(EXTI_Line1) != RESET) 
    {
       // if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) != 0)
        {
            ++counts; // Total particles detected

            if(is_beeping) beep();
            led_blink();
        }
        // Clear interrupt flag
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

// 300/sqrt(n), 3-sigma error estimation with range conditioning
uint32_t invsqrt(uint32_t n)
{
    uint32_t ret = 2;
    if(n < 10) return 99; 
    if(n > 89999) return 1;
    for(uint8_t i = 0; i < 16; i++) // Newton's iteration for 1/ret^2 - n/90000 == 0
    {
        ret = (1 + 3*ret - (n*ret*ret*ret)/90000);
        ret >>= 1;
    }
    return ret;
}

void compute_and_display_values(void)
{
    uint32_t tmp;
    uint32_t err;

    uint32_t fast_nrh;
    uint32_t fast_error;

    tmp = NRH_PER_CPS * counts / seconds; // compute averaged intensity

    if(tmp < background_nrh) nrh = 0;
    else nrh = tmp - background_nrh;      // subtract background if it is set

    if(background_err == 0) err = invsqrt(counts); // compute error
    else
    {
        if(nrh == 0) err = 99;
        else err = (background_err * background_nrh + invsqrt(counts)*tmp) / (nrh); 
        if(err > 99) err = 99;
    }

    fast_nrh = median_filter(NRH_PER_CPS*(counts-prev_counts)); // median filtered 3 last readings
    fast_error = invsqrt(fast_nrh / NRH_PER_CPS);

    LCD5110_clear();
    update_display(is_beeping, batt_charge, nrh, err, fast_nrh, fast_error);
}

// Updates display reading once per second and measures battery charge
void TIM4_IRQHandler(void)
{
    uint32_t voltage;
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        ++seconds;

        voltage = vcc_voltage();
        batt_charge = (voltage - MIN_VOLTAGE) / ((MAX_VOLTAGE - MIN_VOLTAGE) / 100);

        button_1_shutdown(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0));
        button_2_toggle_beeping(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2));

        compute_and_display_values();

        prev_counts = counts;
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}

int main(void)
{
    SetSysClockTo72();

    pushbuttons_init();
    vcc_voltage_monitor_init();
    display_init();
    beep_init();
    led_init();

    init_display_timer();

    LCD5110_init();
    LCD5110_Led(0); 
    LCD5110_set_XY(0,0);
    LCD5110_write_string("Initializing..");

    geiger_counter_input_init();

    // ENABLE Wake Up Pin
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
    PWR_WakeUpPinCmd(ENABLE);
 
    while(1)
    {
        display_number(counts, 0);	
    }
}
