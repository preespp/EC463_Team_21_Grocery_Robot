/**
 * @file ita_robot.cpp
 * @brief PC-controlled chassis logic (no gimbal/supercap/referee/IMU)
 */

/* Includes ------------------------------------------------------------------*/

#include "ita_robot.h"
#include "1_Middleware/1_Driver/Math/drv_math.h"
#include "usart.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

void Class_Robot::Init()
{
    // speed planning slopes
    Slope_Speed_X.Init(2.5f / 1000.0f, 3.0f / 1000.0f);
    Slope_Speed_Y.Init(2.5f / 1000.0f, 5.0f / 1000.0f);
    Slope_Omega.Init(4.0f * PI / 1000.0f, 4.0f * PI / 1000.0f);

    PC.Init(&huart2);
    Chassis.Init();
}

void Class_Robot::Loop()
{
}

void Class_Robot::TIM_1000ms_Alive_PeriodElapsedCallback()
{
}

void Class_Robot::TIM_100ms_Alive_PeriodElapsedCallback()
{
    PC.TIM_100ms_Alive_PeriodElapsedCallback();
    Chassis.TIM_100ms_Alive_PeriodElapsedCallback();
}

void Class_Robot::TIM_10ms_Calculate_PeriodElapsedCallback()
{
}

void Class_Robot::TIM_2ms_Calculate_PeriodElapsedCallback()
{
    Chassis.TIM_2ms_Resolution_PeriodElapsedCallback();
    Chassis.TIM_2ms_Control_PeriodElapsedCallback();
}

void Class_Robot::TIM_1ms_Calculate_Callback()
{
    PC.TIM_1ms_Calculate_PeriodElapsedCallback();
    _Chassis_Control();
}

void Class_Robot::_Chassis_Control()
{
    float tmp_chassis_velocity_max = 3.0f;
    float tmp_chassis_omega_max = 4.0f * PI;

    if (PC.Get_Keyboard_Key_Shift() == DR16_Key_Status_PRESSED)
    {
        tmp_chassis_velocity_max = 4.0f;
        tmp_chassis_omega_max = 6.0f * PI;
        Slope_Speed_X.Set_Increase_Value(5.0f / 1000.0f);
    }
    else
    {
        Slope_Speed_X.Set_Increase_Value(3.0f / 1000.0f);
    }

    Chassis.Set_Power_Limit_Max(45.0f);

    if (PC.Get_Status() == DR16_Status_DISABLE || PC.Get_Left_Switch() == DR16_Switch_Status_DOWN)
    {
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        return;
    }

    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_NORMAL);

    float pc_left_x = PC.Get_Left_X();
    float pc_left_y = PC.Get_Left_Y();
    float pc_yaw = PC.Get_Yaw();

    pc_left_x = Math_Abs(pc_left_x) > PC_Rocker_Dead_Zone ? pc_left_x : 0.0f;
    pc_left_y = Math_Abs(pc_left_y) > PC_Rocker_Dead_Zone ? pc_left_y : 0.0f;
    pc_yaw = Math_Abs(pc_yaw) > PC_Rocker_Dead_Zone ? pc_yaw : 0.0f;

    float tmp_expect_direction_velocity_x = pc_left_y * tmp_chassis_velocity_max;
    float tmp_expect_direction_velocity_y = -pc_left_x * tmp_chassis_velocity_max;
    // Invert yaw to keep Q=CCW, E=CW with the default PC script.
    float tmp_expect_direction_omega = -pc_yaw * tmp_chassis_omega_max;

    Slope_Speed_X.Set_Now_Real(Chassis.Get_Now_Velocity_X());
    Slope_Speed_Y.Set_Now_Real(Chassis.Get_Now_Velocity_Y());
    Slope_Speed_X.Set_Target(tmp_expect_direction_velocity_x);
    Slope_Speed_Y.Set_Target(tmp_expect_direction_velocity_y);
    Slope_Speed_X.TIM_Calculate_PeriodElapsedCallback();
    Slope_Speed_Y.TIM_Calculate_PeriodElapsedCallback();

    float tmp_planning_chassis_velocity_x = Slope_Speed_X.Get_Out();
    float tmp_planning_chassis_velocity_y = Slope_Speed_Y.Get_Out();

    Slope_Omega.Set_Target(tmp_expect_direction_omega);
    Slope_Omega.Set_Now_Real(Chassis.Get_Now_Omega());
    Slope_Omega.TIM_Calculate_PeriodElapsedCallback();

    float tmp_planning_chassis_omega = Slope_Omega.Get_Out();

    Chassis.Set_Target_Velocity_X(tmp_planning_chassis_velocity_x);
    Chassis.Set_Target_Velocity_Y(tmp_planning_chassis_velocity_y);
    Chassis.Set_Target_Omega(tmp_planning_chassis_omega);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
