#include "stm32_lib/stm32f10x.h"
#include "stm32_lib/stm32f10x_conf.h"
#include "stm32_lib/system_stm32f10x.h"

#define VCC_DIV 4096 // VCC_DIV = k * 4096
#define VREF 1210    // in millivolts


//Initialize internal ADC, internal voltage reference
void adc1_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    ADC_InitTypeDef ADC_InitStructure;
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;       // single channel
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // single cycle measurement
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Cmd(ADC1, ENABLE);
    ADC_TempSensorVrefintCmd(ENABLE);
    // channel select
    ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 1, ADC_SampleTime_239Cycles5);

    // calibration
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));
}

//Initialize internal ADC, PB1 pin
void adc2_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

    ADC_InitTypeDef ADC_InitStructure;
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;       // single channel
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // single cycle measurement
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC2, &ADC_InitStructure);
    ADC_Cmd(ADC2, ENABLE);

    // channel select
    ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5);

    // calibration
    ADC_ResetCalibration(ADC2);
    while (ADC_GetResetCalibrationStatus(ADC2));
    ADC_StartCalibration(ADC2);
    while (ADC_GetCalibrationStatus(ADC2));

}

uint16_t get_adc1_value(void)
{
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    return ADC_GetConversionValue(ADC1);
}

uint16_t get_adc2_value(void)
{
    ADC_SoftwareStartConvCmd(ADC2, ENABLE);
    while(ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET);
    return ADC_GetConversionValue(ADC2);
}


TIM_OCInitTypeDef timerPWM;

// Generate PWM for forming reference voltage for HV power supply
void vref_out_init(void) 
{
    GPIO_InitTypeDef port;
    TIM_TimeBaseInitTypeDef timer;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 
    GPIO_StructInit(&port);
    port.GPIO_Mode = GPIO_Mode_AF_PP;
    port.GPIO_Pin = GPIO_Pin_7;
    port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &port);
 
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 1;
    timer.TIM_Period = 4096;            // To match ADC range; pulse count == ADC readings
    timer.TIM_ClockDivision = 0;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &timer);
 
    TIM_OCStructInit(&timerPWM);
    timerPWM.TIM_OCMode = TIM_OCMode_PWM1;
    timerPWM.TIM_OutputState = TIM_OutputState_Enable;
    timerPWM.TIM_Pulse = 0;
    timerPWM.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(TIM3, &timerPWM);
 
    TIM_Cmd(TIM3, ENABLE);
}



void vcc_voltage_monitor_init(void)
{
    adc1_init();
    adc2_init();
    vref_out_init();
}

uint16_t vcc_voltage(void) // in millivolts
{
    uint32_t val = get_adc2_value() * 2; // Compensate input divider
    uint32_t vref = get_adc1_value();

    val *= VREF;
    val /= vref;

    timerPWM.TIM_Pulse = vref; // Set DTC = Vcc/Vref
    TIM_OC2Init(TIM3, &timerPWM);

    return val;
}
