/**
 * @file crt_chassis.cpp
 * @brief Chassis kinematics and wheel control (4x M3508 + C620).
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_chassis.h"

/* Function definitions ------------------------------------------------------*/

void Class_Chassis::Init(CAN_HandleTypeDef *hcan)
{
    // Conservative defaults; tune for your chassis.
    PID_Velocity_X.Init(1.0f, 0.0f, 0.0f, 0.0f, 2.0f, 4.0f, 0.002f);
    PID_Velocity_Y.Init(1.0f, 0.0f, 0.0f, 0.0f, 2.0f, 4.0f, 0.002f);
    PID_Omega.Init(1.0f, 0.0f, 0.0f, 0.0f, 2.0f, 4.0f, 0.002f);

    for (int i = 0; i < 4; i++)
    {
        //Motor_Wheel[i].PID_Omega.Init(5.0f, 0.2f, 0.0f, 0.0f, 10.0f, 20.0f, 0.002f);
        Motor_Wheel[i].PID_Omega.Init(3.0f, 0.05f, 0.1f, 0.0f, 5.0f, 10.0f, 0.002f);
    }

    Motor_Wheel[Chassis_Wheel_Front_Left].Init(hcan, Motor_DJI_ID_0x201, Motor_DJI_Control_Method_OMEGA);
    Motor_Wheel[Chassis_Wheel_Front_Right].Init(hcan, Motor_DJI_ID_0x202, Motor_DJI_Control_Method_OMEGA);
    Motor_Wheel[Chassis_Wheel_Rear_Left].Init(hcan, Motor_DJI_ID_0x203, Motor_DJI_Control_Method_OMEGA);
    Motor_Wheel[Chassis_Wheel_Rear_Right].Init(hcan, Motor_DJI_ID_0x204, Motor_DJI_Control_Method_OMEGA);
}

void Class_Chassis::TIM_100ms_Alive_PeriodElapsedCallback()
{
    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].TIM_100ms_Alive_PeriodElapsedCallback();
    }
}

void Class_Chassis::TIM_2ms_Resolution_PeriodElapsedCallback()
{
    Self_Resolution();
}

void Class_Chassis::TIM_2ms_Control_PeriodElapsedCallback()
{
    float vx_cmd = Target_Velocity_X;
    float vy_cmd = Target_Velocity_Y;
    float omega_cmd = Target_Omega;

    switch (Chassis_Control_Type)
    {
    case (Chassis_Control_Type_DISABLE):
    {
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_Control_Method(Motor_DJI_Control_Method_CURRENT);
            Motor_Wheel[i].Set_Target_Current(0.0f);
            Motor_Wheel[i].PID_Omega.Set_Integral_Error(0.0f);
        }
        return;
    }
    case (Chassis_Control_Type_NORMAL):
    default:
        break;
    }

    PID_Velocity_X.Set_Target(Target_Velocity_X);
    PID_Velocity_X.Set_Now(Now_Velocity_X);
    PID_Velocity_X.TIM_Calculate_PeriodElapsedCallback();
    vx_cmd = Target_Velocity_X + PID_Velocity_X.Get_Out();

    PID_Velocity_Y.Set_Target(Target_Velocity_Y);
    PID_Velocity_Y.Set_Now(Now_Velocity_Y);
    PID_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();
    vy_cmd = Target_Velocity_Y + PID_Velocity_Y.Get_Out();

    PID_Omega.Set_Target(Target_Omega);
    PID_Omega.Set_Now(Get_Now_AHRS_Omega());
    PID_Omega.TIM_Calculate_PeriodElapsedCallback();
    omega_cmd = Target_Omega + PID_Omega.Get_Out();

    Kinematics_Inverse_Resolution(vx_cmd, vy_cmd, omega_cmd);

    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
        Motor_Wheel[i].Set_Target_Omega(Target_Wheel_Omega[i]);
        Motor_Wheel[i].TIM_Calculate_PeriodElapsedCallback();
    }
}

void Class_Chassis::Self_Resolution()
{
    const float chassis_radius = (Wheelbase + Wheeltrack) * 0.5f;

    float w_fl = Motor_Wheel[Chassis_Wheel_Front_Left].Get_Now_Omega() * Wheel_Direction[Chassis_Wheel_Front_Left];
    float w_fr = Motor_Wheel[Chassis_Wheel_Front_Right].Get_Now_Omega() * Wheel_Direction[Chassis_Wheel_Front_Right];
    float w_rl = Motor_Wheel[Chassis_Wheel_Rear_Left].Get_Now_Omega() * Wheel_Direction[Chassis_Wheel_Rear_Left];
    float w_rr = Motor_Wheel[Chassis_Wheel_Rear_Right].Get_Now_Omega() * Wheel_Direction[Chassis_Wheel_Rear_Right];

    Now_Velocity_X = (Wheel_Radius / 4.0f) * (w_fl + w_fr + w_rl + w_rr);
    Now_Velocity_Y = (Wheel_Radius / 4.0f) * (-w_fl + w_fr + w_rl - w_rr);
    Now_Omega = (Wheel_Radius / (4.0f * chassis_radius)) * (-w_fl + w_fr - w_rl + w_rr);
}

void Class_Chassis::Kinematics_Inverse_Resolution(float vx_cmd, float vy_cmd, float omega_cmd)
{
    const float chassis_radius = (Wheelbase + Wheeltrack) * 0.5f;
    const float inv_r = 1.0f / Wheel_Radius;

    Target_Wheel_Omega[Chassis_Wheel_Front_Left] = (vx_cmd - vy_cmd - omega_cmd * chassis_radius) * inv_r;
    Target_Wheel_Omega[Chassis_Wheel_Front_Right] = (vx_cmd + vy_cmd + omega_cmd * chassis_radius) * inv_r;
    Target_Wheel_Omega[Chassis_Wheel_Rear_Left] = (vx_cmd + vy_cmd - omega_cmd * chassis_radius) * inv_r;
    Target_Wheel_Omega[Chassis_Wheel_Rear_Right] = (vx_cmd - vy_cmd + omega_cmd * chassis_radius) * inv_r;

    for (int i = 0; i < 4; i++)
    {
        Target_Wheel_Omega[i] *= Wheel_Direction[i];
    }
}
