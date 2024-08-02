//
// Created by 1 on 2023-11-07.
//
#include "Attitude_Task.h"

float TargetAngle1 = 0,TargetAngle2 = 0;
enum GPStates gpstate = STOP;
enum DPStates dpstate = NONE;
float NewHeartbeat = 0;//心跳值
uint8_t LieDown_flag = 0;

void StandUp_Posture(void)
{
    LieDown_flag = 0;
    ChangeGainOfPID(15.0f,5.0f,0.6f,0);
    AllMotorSpeedLimit(SpeedNormal);
    Get_Target(0,4);
    SetCoupledThetaPositionAll();
}

void StandUp_Posture_sway(void)
{
    static int flag = 'f';
    LieDown_flag = 0;
    ChangeGainOfPID(10.0f,0.8f,0.6f,0);
    AllMotorSpeedLimit(6.0f);

    switch (flag) {
        case 'f':
            Get_Target(0,4);
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);

            Get_Target(0 - 1.2f,4 - 1.2f);
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
            break;
        case 'b':
            Get_Target(0,4);
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);

            Get_Target(0 - 1.2f,4 - 1.2f);
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
            break;
        default:
            break;
    }

    if(flag == 'f')
    {
        osDelay(800);
        flag = 'b';
    }
    else if(flag == 'b')
    {
        osDelay(800);
        flag = 'f';
    }
}

void StandUp_Posture_UpDown(float vel)
{
    LieDown_flag = 0;
    ChangeGainOfPID(10.0f,0.8f,0.6f,0);
    AllMotorSpeedLimit(SpeedNormal);

    Get_Target(0 + vel,4 + vel);
    SetCoupledThetaPositionAll();
}

void StandUp_Posture_LeftRight(float vel,int flag)
{
    LieDown_flag = 0;
    ChangeGainOfPID(6.0f,0.8f,0.6f,0);
    AllMotorSpeedLimit(2.0f);

    switch (flag) {
        case 'l':
            Get_Target(0 + vel/1.6f,4 + vel/1.6f);
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(1);
            Get_Target(0,4);
            SetCoupledThetaPosition(2);
            SetCoupledThetaPosition(3);
            break;
        case 'r':
            Get_Target(0 + vel/1.6f,4 + vel/1.6f);
            SetCoupledThetaPosition(2);
            SetCoupledThetaPosition(3);
            Get_Target(0,4);
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(1);
            break;
        default:
            break;
    }

}

void LieDown_Posture(void)
{
    LieDown_flag = 1;
    ChangeGainOfPID(8.0f,0.5f,0.0f,0.0f);//输出化PID
    AllMotorSpeedLimit(SpeedNormal);
    for(int i = 1;i < 9;i ++)
    {
        AngleWant_MotorX[i] = begin_pos[i];
    }
}
void MarkingTime(void)
{
    AllMotorSpeedLimit(SpeedFast);
    NewHeartbeat = 5;
    ChangeGainOfPID(10.0f,0.8f,0.6f,0);
    gait_detached(state_detached_params[2],0.0f, 0.5f, 0.5f, 0.0f,
                  1.0f,1.0f,1.0f,1.0f);
}
//实际运行Trot步态
void Trot(float direction,int8_t kind)
{
    switch(kind)
    {
        case 0://小步Trot
            AllMotorSpeedLimit(SpeedNormal);
            NewHeartbeat = 4;
            ChangeGainOfPID(15.0f,3.0f,0.6f,0);
//            ChangeYawOfPID(200.0f,2.0f,3000.0f,10.0f);
//            YawControl(yawwant, &state_detached_params[4], direction);
            gait_detached(state_detached_params[0],0.0f, 0.5f, 0.5f, 0.0f,
                          direction,direction,direction,direction);
            break;
        case 1://大步Trot
            AllMotorSpeedLimit(SpeedNormal);
            NewHeartbeat = 4;
            ChangeGainOfPID(15.0f,0.8f,0.6f,0);
//            ChangeYawOfPID(1000.0f,10.0f,4000.0f,15.0f);
//            YawControl(yawwant, &state_detached_params[1], direction);
            gait_detached(state_detached_params[1],0.0f, 0.5f, 0.5f, 0.0f,
                          direction,direction,direction,direction);
            break;
        default:
            break;
    }
}

//慢步
void Walk(float direction,uint8_t speed)
{
    NewHeartbeat = 5;
    AllMotorSpeedLimit(SpeedFast);
    ChangeGainOfPID(10.0f,1.0f,0.6f,0);
//    ChangeYawOfPID(100.0f,0.5f,2500.0f,10.0f);
//    YawControl(yawwant, &state_detached_params[3], direction);
    gait_detached(state_detached_params[3],0.0f,0.75f,0.5f,0.25f,direction,direction,direction,direction);
}
//转弯步态
void Turn(int state_flag,int speed_flag)
{
    float length;

    if(speed_flag == 'f')
    {
        length = 15.0f;
        state_detached_params[4].detached_params_0.freq = 5.0f;
        state_detached_params[4].detached_params_1.freq = 5.0f;
        state_detached_params[4].detached_params_2.freq = 5.0f;
        state_detached_params[4].detached_params_3.freq = 5.0f;
    }
    else if(speed_flag == 's')
    {
        length = 8.0f;
        state_detached_params[4].detached_params_0.freq = 3.0f;
        state_detached_params[4].detached_params_1.freq = 3.0f;
        state_detached_params[4].detached_params_2.freq = 3.0f;
        state_detached_params[4].detached_params_3.freq = 3.0f;
    }

    NewHeartbeat = 5;
    AllMotorSpeedLimit(SpeedFast);
    ChangeGainOfPID(11.0f,0.8f,0.6f,0);
    switch (state_flag) {
        case 'l':
            state_detached_params[4].detached_params_0.step_length = -length;
            state_detached_params[4].detached_params_1.step_length = -length;
            state_detached_params[4].detached_params_2.step_length = length;
            state_detached_params[4].detached_params_3.step_length = length;
            break;
        case 'r':
            state_detached_params[4].detached_params_0.step_length = length;
            state_detached_params[4].detached_params_1.step_length = length;
            state_detached_params[4].detached_params_2.step_length = -length;
            state_detached_params[4].detached_params_3.step_length = -length;
            break;
        default:
            break;
    }
    gait_detached(state_detached_params[4],  0.0f, 0.5f, 0.5f, 0.0f,
                  1.0f, 1.0f, 1.0f,1.0f);
}

void Up_and_Down(float val)
{
    Get_Target(0,PI);
    AngleWant_MotorX[1]=TargetAngle1-offset_front_0 + val;
    AngleWant_MotorX[2]=TargetAngle2-offset_front_1 + val;//+10.0f
    AngleWant_MotorX[3]=TargetAngle1-offset_back_0 + val;//+5.0f
    AngleWant_MotorX[4]=TargetAngle2-offset_back_1 + val;
    AngleWant_MotorX[5]=-TargetAngle2+offset_front_1 + val;//-4.0f
    AngleWant_MotorX[6]=-TargetAngle1+offset_front_0 + val;
    AngleWant_MotorX[7]=-TargetAngle2+offset_back_1 + val;
    AngleWant_MotorX[8]=-TargetAngle1+offset_back_0 + val;

}

void Handshake(void)
{
    //整体限制
    AllMotorSpeedLimit(SpeedNormal);
    ChangeGainOfPID(18.0f,1.0f,0.8f,0);
    //左前腿
    x =  LegLenthMax*cos((30)*PI/180);
    y = -LegLenthMax*sin((30)*PI/180);
    CartesianToTheta();
    SetCoupledThetaPosition(0);
    //其它腿
    x =  LegLenthMin*cos(60*PI/180);
    y =  LegLenthMin*sin(60*PI/180);
    CartesianToTheta();
    SetCoupledThetaPosition(1);
    SetCoupledThetaPosition(2);
    SetCoupledThetaPosition(3);
    gpstate = STOP;

}

void StretchPosture(void)
{
    AllMotorSpeedLimit(SpeedNormal);
    ChangeGainOfPID(13.0f,0.5f,0.8f,0);
    x = -(LegStandLenth + 5) * cos(60 * PI / 180);
    y = (LegStandLenth + 5) * sin(60 * PI / 180);
    CartesianToTheta();
    SetCoupledThetaPosition(1);
    SetCoupledThetaPosition(3);
    osDelay(800);
    x = LegLenthMax * cos(30 * PI / 180);
    y = LegLenthMax * sin(30 * PI / 180);
    CartesianToTheta();
    SetCoupledThetaPosition(0);
    osDelay(1500);
    SetCoupledThetaPosition(2);
    gpstate = STOP;
}