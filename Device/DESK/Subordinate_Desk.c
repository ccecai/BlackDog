//
// Created by 1 on 2024-05-20.
//
#include "Subordinate_Desk.h"

uint8_t Desk_Data[Length_of_Desk] = {0};
uint16_t TOF_Distance = 0;

void Process(void )
{
   if(Desk_Data[0] == 0x0b && Desk_Data[1] == 1 && Desk_Data[2] == 0xac)
       __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,250);
   else if(Desk_Data[0] == 0x0c && Desk_Data[1] == 0 && Desk_Data[2] == 0xac)
       __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);

   if(Desk_Data[3] == 0xa3 && Desk_Data[6] == 0xb4)
       TOF_Distance = Desk_Data[4] | Desk_Data[5] << 8;
}