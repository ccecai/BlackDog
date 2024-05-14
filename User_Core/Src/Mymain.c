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
    //start pwm channel
    //����PWMͨ��
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);

    can1_filter_init();
    can2_filter_init();
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);//������
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3); //5V��Դ�������
    HAL_TIM_Base_Start_IT(&htim2);
    delay_init();
}