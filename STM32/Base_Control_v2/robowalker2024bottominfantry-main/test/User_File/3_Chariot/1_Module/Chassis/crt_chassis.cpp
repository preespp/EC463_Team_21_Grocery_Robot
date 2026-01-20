/**
 * @file crt_chassis.cpp
 * @brief Mecanum chassis control (C620 + M3508)
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_chassis.h"

/* Function prototypes -------------------------------------------------------*/

void Class_Chassis::Init()
{
    PID_Velocity_X.Init(600.0f, 0.0f, 0.0f, 0.0f, 150.0f, 3000.0f, 0.002f);
    PID_Velocity_Y.Init(600.0f, 0.0f, 0.0f, 0.0f, 150.0f, 3000.0f, 0.002f);
    PID_Omega.Init(25.0f, 0.0f, 0.0f, 0.0f, 10.0f, 400.0f, 0.002f);

    const float gearbox_rate = 3591.0f / 187.0f;

    Motor_Wheel[0].Init(&hcan1, Motor_DJI_ID_0x201, Motor_DJI_Control_Method_CURRENT, gearbox_rate, Motor_DJI_Power_Limit_Status_ENABLE);
    Motor_Wheel[1].Init(&hcan1, Motor_DJI_ID_0x202, Motor_DJI_Control_Method_CURRENT, gearbox_rate, Motor_DJI_Power_Limit_Status_ENABLE);
    Motor_Wheel[2].Init(&hcan1, Motor_DJI_ID_0x203, Motor_DJI_Control_Method_CURRENT, gearbox_rate, Motor_DJI_Power_Limit_Status_ENABLE);
    Motor_Wheel[3].Init(&hcan1, Motor_DJI_ID_0x204, Motor_DJI_Control_Method_CURRENT, gearbox_rate, Motor_DJI_Power_Limit_Status_ENABLE);
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
    Kinematics_Inverse_Resolution();
    Output_To_Dynamics();
    Dynamics_Inverse_Resolution();
    Output_To_Motor();
}

void Class_Chassis::Self_Resolution()
{
    float w_fl = Motor_Wheel[0].Get_Now_Omega();
    float w_fr = Motor_Wheel[1].Get_Now_Omega();
    float w_rl = Motor_Wheel[2].Get_Now_Omega();
    float w_rr = Motor_Wheel[3].Get_Now_Omega();

    float base_sum = Wheel_Base_Half_Length + Wheel_Base_Half_Width;

    Now_Velocity_X = Wheel_Radius * (w_fl + w_fr + w_rl + w_rr) * 0.25f;
    Now_Velocity_Y = Wheel_Radius * (-w_fl + w_fr + w_rl - w_rr) * 0.25f;
    Now_Omega = Wheel_Radius * (-w_fl + w_fr - w_rl + w_rr) / (4.0f * base_sum);

    Angle_Pitch = 0.0f;
    Angle_Roll = 0.0f;
    Slope_Direction_X = 0.0f;
    Slope_Direction_Y = 0.0f;
    Slope_Direction_Z = 1.0f;

    Now_Motor_Power = 0.0f;
    Now_Steer_Motor_Power = 0.0f;
    Now_Wheel_Motor_Power = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        Now_Motor_Power += Motor_Wheel[i].Get_Now_Power();
        Now_Wheel_Motor_Power += Motor_Wheel[i].Get_Now_Power();
    }
}

void Class_Chassis::Kinematics_Inverse_Resolution()
{
    float base_sum = Wheel_Base_Half_Length + Wheel_Base_Half_Width;

    // Wheel order: 0=FL, 1=FR, 2=RL, 3=RR.
    Target_Wheel_Omega[0] = (Target_Velocity_X - Target_Velocity_Y - Target_Omega * base_sum) / Wheel_Radius;
    Target_Wheel_Omega[1] = (Target_Velocity_X + Target_Velocity_Y + Target_Omega * base_sum) / Wheel_Radius;
    Target_Wheel_Omega[2] = (Target_Velocity_X + Target_Velocity_Y - Target_Omega * base_sum) / Wheel_Radius;
    Target_Wheel_Omega[3] = (Target_Velocity_X - Target_Velocity_Y + Target_Omega * base_sum) / Wheel_Radius;
}

void Class_Chassis::Output_To_Dynamics()
{
    switch (Chassis_Control_Type)
    {
    case (Chassis_Control_Type_DISABLE):
    {
        PID_Velocity_X.Set_Integral_Error(0.0f);
        PID_Velocity_Y.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
        break;
    }
    case (Chassis_Control_Type_NORMAL):
    {
        PID_Velocity_X.Set_Target(Target_Velocity_X);
        PID_Velocity_X.Set_Now(Now_Velocity_X);
        PID_Velocity_X.TIM_Calculate_PeriodElapsedCallback();

        PID_Velocity_Y.Set_Target(Target_Velocity_Y);
        PID_Velocity_Y.Set_Now(Now_Velocity_Y);
        PID_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();

        PID_Omega.Set_Target(Target_Omega);
        PID_Omega.Set_Now(Now_Omega);
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();
        break;
    }
    }
}

void Class_Chassis::Dynamics_Inverse_Resolution()
{
    float force_x = PID_Velocity_X.Get_Out();
    float force_y = PID_Velocity_Y.Get_Out();
    float torque_omega = PID_Omega.Get_Out();
    float base_sum = Wheel_Base_Half_Length + Wheel_Base_Half_Width;

    float tmp_force[4];
    tmp_force[0] = force_x - force_y - torque_omega / base_sum;
    tmp_force[1] = force_x + force_y + torque_omega / base_sum;
    tmp_force[2] = force_x + force_y - torque_omega / base_sum;
    tmp_force[3] = force_x - force_y + torque_omega / base_sum;

    for (int i = 0; i < 4; i++)
    {
        Target_Wheel_Current[i] = tmp_force[i] * Wheel_Radius + Wheel_Speed_Limit_Factor * (Target_Wheel_Omega[i] - Motor_Wheel[i].Get_Now_Omega());

        if (Target_Wheel_Omega[i] > Wheel_Resistance_Omega_Threshold)
        {
            Target_Wheel_Current[i] += Dynamic_Resistance_Wheel_Current[i];
        }
        else if (Target_Wheel_Omega[i] < -Wheel_Resistance_Omega_Threshold)
        {
            Target_Wheel_Current[i] -= Dynamic_Resistance_Wheel_Current[i];
        }
        else
        {
            Target_Wheel_Current[i] += Motor_Wheel[i].Get_Now_Omega() / Wheel_Resistance_Omega_Threshold * Dynamic_Resistance_Wheel_Current[i];
        }
    }
}

void Class_Chassis::Output_To_Motor()
{
    switch (Chassis_Control_Type)
    {
    case (Chassis_Control_Type_DISABLE):
    {
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_Control_Method(Motor_DJI_Control_Method_CURRENT);
            Motor_Wheel[i].Set_Target_Current(0.0f);
        }
        break;
    }
    case (Chassis_Control_Type_NORMAL):
    {
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_Control_Method(Motor_DJI_Control_Method_CURRENT);
            Motor_Wheel[i].Set_Target_Current(Target_Wheel_Current[i]);
        }
        break;
    }
    }

    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].TIM_Calculate_PeriodElapsedCallback();
    }

    _Power_Limit_Control();

    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].TIM_Power_Limit_After_Calculate_PeriodElapsedCallback();
    }
}

void Class_Chassis::_Power_Limit_Control()
{
    float available_power = Power_Limit_Max;
    float wheel_consume_power = 0.0f;
    float wheel_power_single[4];

    for (int i = 0; i < 4; i++)
    {
        wheel_power_single[i] = Motor_Wheel[i].Get_Power_Estimate();
        if (wheel_power_single[i] > 0)
        {
            wheel_consume_power += wheel_power_single[i];
        }
        else
        {
            available_power += -wheel_power_single[i];
        }
    }

    Steer_Factor = 1.0f;
    Wheel_Factor = __Wheel_Power_Limit_Control(available_power, wheel_consume_power);

    for (int i = 0; i < 4; i++)
    {
        if (wheel_power_single[i] > 0)
        {
            Motor_Wheel[i].Set_Power_Factor(Wheel_Factor);
        }
        else
        {
            Motor_Wheel[i].Set_Power_Factor(1.0f);
        }
    }
}

float Class_Chassis::__Wheel_Power_Limit_Control(float wheel_available_power, float wheel_consume_power)
{
    if (wheel_consume_power <= wheel_available_power)
    {
        return (1.0f);
    }
    return (wheel_available_power / wheel_consume_power);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
