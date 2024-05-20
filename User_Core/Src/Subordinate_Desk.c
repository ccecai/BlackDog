//
// Created by 1 on 2024-05-20.
//
#include "Subordinate_Desk.h"

uint8_t Desk_Data[Length_of_Desk] = {0};

void Process(void )
{
   if(Desk_Data[0] == 0x0b && Desk_Data[1] == 1)
       __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,250);
   else if(Desk_Data[0] == 0x0c && Desk_Data[1] == 0)
       __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
}