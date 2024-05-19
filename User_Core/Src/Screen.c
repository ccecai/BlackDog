//
// Created by 1 on 2024-05-04.
//
#include "Screen.h"

int Correct_Password[4] = {0,4,2,3};
uint8_t Screen_Data[Screen_Length] = {0};

void Screen_DataProcess(void)
{
    static int Password[4];

    for (int i = 0; i < 4; ++i) {
        Password[i] = Screen_Data[1 + i] - 48;
    }

    if(Screen_Data[0] == 0x0b && strcmp((char *)Correct_Password,(char *)Password) == 0)
    {

        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,250);
        //´ò¿ª¶æ»ú
    }
    else if(Screen_Data[0] == 0x0c)
    {
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
    }
}