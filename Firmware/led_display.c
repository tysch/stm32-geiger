#include "led_display.h"

// Initialize display shift registers driver
// B12 - SCLK; B13 - RCLK; B14 - DIN

void display_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = (GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;            //Push-Pull output
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void display_number(int number, int ppos)
{
    int digit;
    int serdata = 0;
    for(int i = 0; i < 8; i++)
    {
        digit = number % 10;
        number /= 10;
        switch (digit)
        {
        case 0:
            serdata = 252;
            break;
        case 1:
            serdata = 96;
            break;
        case 2:
            serdata = 218;
            break;
        case 3:
            serdata = 242;
            break;
        case 4:
            serdata = 102;
            break;
        case 5:
            serdata = 182;
            break;
        case 6:
            serdata = 62;
            break;
        case 7:
            serdata = 224;
            break;
        case 8:
            serdata = 254;
            break;
        case 9:
            serdata = 230;
            break;
        }
        if(ppos == i) serdata |= 1;
        serdata = (~serdata) & 255;
        serdata |= (1<<(15-i));

        for(int j = 0; j < 16; j++)
        {
            if(serdata & 0x1)  GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_SET);
            GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);
            GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET);
            GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_RESET);
            serdata >>= 1;
        }
        GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_SET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_RESET);
    }
}
