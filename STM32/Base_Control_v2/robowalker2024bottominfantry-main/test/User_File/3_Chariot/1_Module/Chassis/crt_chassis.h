/**
 * @file crt_chassis.h
 * @brief Mecanum chassis control (C620 + M3508)
 */

#ifndef CRT_CHASSIS_H
#define CRT_CHASSIS_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"

/* Exported types ------------------------------------------------------------*/

enum Enum_Chassis_Control_Type
{
    Chassis_Control_Type_DISABLE = 0,
    Chassis_Control_Type_NORMAL,
};

class Class_Chassis
{
public:
    Class_PID PID_Velocity_X;
    Class_PID PID_Velocity_Y;
    Class_PID PID_Omega;

    Class_Motor_DJI_C620 Motor_Wheel[4];

    void Init();

    inline float Get_Now_Motor_Power();
    inline float Get_Now_Steer_Motor_Power();
    inline float Get_Now_Wheel_Motor_Power();
    inline float Get_Steer_Factor();
    inline float Get_Wheel_Factor();
    inline float Get_Now_Velocity_X();
    inline float Get_Now_Velocity_Y();
    inline float Get_Now_Omega();
    inline float Get_Now_AHRS_Omega();
    inline float Get_Angle_Pitch();
    inline float Get_Angle_Roll();
    inline float Get_Slope_Direction_X();
    inline float Get_Slope_Direction_Y();
    inline float Get_Slope_Direction_Z();
    inline Enum_Chassis_Control_Type Get_Chassis_Control_Type();
    inline float Get_Target_Velocity_X();
    inline float Get_Target_Velocity_Y();
    inline float Get_Target_Omega();

    inline void Set_Power_Limit_Max(float __Power_Limit_Max);
    inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);
    inline void Set_Target_Velocity_X(float __Target_Velocity_X);
    inline void Set_Target_Velocity_Y(float __Target_Velocity_Y);
    inline void Set_Target_Omega(float __Target_Omega);

    void TIM_100ms_Alive_PeriodElapsedCallback();
    void TIM_2ms_Resolution_PeriodElapsedCallback();
    void TIM_2ms_Control_PeriodElapsedCallback();

protected:
    // wheel geometry
    const float Wheel_Radius = 0.0762f;
    const float Wheel_Base_Half_Length = 0.1905f;
    const float Wheel_Base_Half_Width = 0.3175f;

    // kinematics outputs
    float Target_Wheel_Omega[4];
    float Target_Wheel_Current[4];
    // wheel direction (1=matches chassis math, -1=reversed motor mounting)
    static constexpr int8_t Wheel_Direction[4] = {1, -1, 1, -1};

    // friction compensation
    float Static_Resistance_Wheel_Current[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float Dynamic_Resistance_Wheel_Current[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float Wheel_Resistance_Omega_Threshold = 1.0f;
    float Wheel_Speed_Limit_Factor = 0.5f;

    // power estimates
    float Now_Motor_Power = 0.0f;
    float Now_Steer_Motor_Power = 0.0f;
    float Now_Wheel_Motor_Power = 0.0f;
    float Steer_Factor = 1.0f;
    float Wheel_Factor = 1.0f;

    // current state
    float Now_Velocity_X = 0.0f;
    float Now_Velocity_Y = 0.0f;
    float Now_Omega = 0.0f;

    float Angle_Pitch = 0.0f;
    float Angle_Roll = 0.0f;

    float Slope_Direction_X = 0.0f;
    float Slope_Direction_Y = 0.0f;
    float Slope_Direction_Z = 1.0f;

    // limits and targets
    float Power_Limit_Max = 45.0f;
    Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_DISABLE;
    float Target_Velocity_X = 0.0f;
    float Target_Velocity_Y = 0.0f;
    float Target_Omega = 0.0f;

    void Self_Resolution();
    void Kinematics_Inverse_Resolution();
    void Output_To_Dynamics();
    void Dynamics_Inverse_Resolution();
    void Output_To_Motor();
    void _Power_Limit_Control();
    float __Wheel_Power_Limit_Control(float wheel_available_power, float wheel_consume_power);
};

/* Inline getters/setters ----------------------------------------------------*/

inline float Class_Chassis::Get_Now_Motor_Power()
{
    return (Now_Motor_Power);
}

inline float Class_Chassis::Get_Now_Steer_Motor_Power()
{
    return (Now_Steer_Motor_Power);
}

inline float Class_Chassis::Get_Now_Wheel_Motor_Power()
{
    return (Now_Wheel_Motor_Power);
}

inline float Class_Chassis::Get_Steer_Factor()
{
    return (Steer_Factor);
}

inline float Class_Chassis::Get_Wheel_Factor()
{
    return (Wheel_Factor);
}

inline float Class_Chassis::Get_Now_Velocity_X()
{
    return (Now_Velocity_X);
}

inline float Class_Chassis::Get_Now_Velocity_Y()
{
    return (Now_Velocity_Y);
}

inline float Class_Chassis::Get_Now_Omega()
{
    return (Now_Omega);
}

inline float Class_Chassis::Get_Now_AHRS_Omega()
{
    return (Now_Omega);
}

inline float Class_Chassis::Get_Angle_Pitch()
{
    return (Angle_Pitch);
}

inline float Class_Chassis::Get_Angle_Roll()
{
    return (Angle_Roll);
}

inline float Class_Chassis::Get_Slope_Direction_X()
{
    return (Slope_Direction_X);
}

inline float Class_Chassis::Get_Slope_Direction_Y()
{
    return (Slope_Direction_Y);
}

inline float Class_Chassis::Get_Slope_Direction_Z()
{
    return (Slope_Direction_Z);
}

inline Enum_Chassis_Control_Type Class_Chassis::Get_Chassis_Control_Type()
{
    return (Chassis_Control_Type);
}

inline float Class_Chassis::Get_Target_Velocity_X()
{
    return (Target_Velocity_X);
}

inline float Class_Chassis::Get_Target_Velocity_Y()
{
    return (Target_Velocity_Y);
}

inline float Class_Chassis::Get_Target_Omega()
{
    return (Target_Omega);
}

inline void Class_Chassis::Set_Power_Limit_Max(float __Power_Limit_Max)
{
    Power_Limit_Max = __Power_Limit_Max;
}

inline void Class_Chassis::Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Chassis_Control_Type = __Chassis_Control_Type;
}

inline void Class_Chassis::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Velocity_X = __Target_Velocity_X;
}

inline void Class_Chassis::Set_Target_Velocity_Y(float __Target_Velocity_Y)
{
    Target_Velocity_Y = __Target_Velocity_Y;
}

inline void Class_Chassis::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
