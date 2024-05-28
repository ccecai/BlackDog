//
// Created by 1 on 2023-11-02.
//
#include "Mymain.h"
#include "bsp_delay.h"

void Mymain_Init(void)
{
    HAL_TIM_Base_Start(&htim5);
    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_Base_Start(&htim3);
    HAL_TIM_Base_Start(&htim1);

    can1_filter_init();
    can2_filter_init();

    //start pwm channel
    //开启PWM通道
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);//舵机

    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);//蜂鸣器
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3); //5V电源输出调节

    __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,80);
    osDelay(500);
    __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,0);

    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);

    delay_init();
}