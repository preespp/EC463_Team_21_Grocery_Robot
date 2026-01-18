/**
 * @file crt_chassis.h
 * @brief Chassis kinematics and wheel control (4x M3508 + C620).
 */

#ifndef CRT_CHASSIS_H
#define CRT_CHASSIS_H

/* Includes ------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif
#include "2_Device/IMU/MPU6500/dvc_imu_mpu6500.h"
#ifdef __cplusplus
}
#endif
#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "1_Middleware/2_Algorithm/PID/alg_pid.h"

/* Exported types ------------------------------------------------------------*/

enum Enum_Chassis_Control_Type
{
    Chassis_Control_Type_DISABLE = 0,
    Chassis_Control_Type_NORMAL,
};

enum Enum_Chassis_Wheel_Index
{
    Chassis_Wheel_Front_Left = 0,
    Chassis_Wheel_Front_Right,
    Chassis_Wheel_Rear_Left,
    Chassis_Wheel_Rear_Right,
};

class Class_Chassis
{
public:
    Class_PID PID_Velocity_X;
    Class_PID PID_Velocity_Y;
    Class_PID PID_Omega;

    Class_Motor_DJI_C620 Motor_Wheel[4];

    void Init(CAN_HandleTypeDef *hcan);

    inline float Get_Now_Velocity_X();
    inline float Get_Now_Velocity_Y();
    inline float Get_Now_Omega();
    inline float Get_Now_AHRS_Omega();

    inline Enum_Chassis_Control_Type Get_Chassis_Control_Type();
    inline float Get_Target_Velocity_X();
    inline float Get_Target_Velocity_Y();
    inline float Get_Target_Omega();

    inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);
    inline void Set_Target_Velocity_X(float __Target_Velocity_X);
    inline void Set_Target_Velocity_Y(float __Target_Velocity_Y);
    inline void Set_Target_Omega(float __Target_Omega);

    void TIM_100ms_Alive_PeriodElapsedCallback();
    void TIM_2ms_Resolution_PeriodElapsedCallback();
    void TIM_2ms_Control_PeriodElapsedCallback();

protected:
    // Geometry (update to match your chassis)
    const float Wheel_Radius = 0.076f;
    const float Wheelbase = 0.36f;
    const float Wheeltrack = 0.30f;

    // Direction sign for each wheel (adjust to wiring)
    float Wheel_Direction[4] = {1.0f, -1.0f, 1.0f, -1.0f};

    float Target_Wheel_Omega[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    float Now_Velocity_X = 0.0f;
    float Now_Velocity_Y = 0.0f;
    float Now_Omega = 0.0f;

    Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_DISABLE;

    float Target_Velocity_X = 0.0f;
    float Target_Velocity_Y = 0.0f;
    float Target_Omega = 0.0f;

    void Self_Resolution();
    void Kinematics_Inverse_Resolution(float vx_cmd, float vy_cmd, float omega_cmd);
};

/* Inline definitions --------------------------------------------------------*/

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
    if (IMU_MPU6500_Get_Status() == IMU_MPU6500_Status_ENABLE)
    {
        return (-IMU_MPU6500_Get_Omega_Z());
    }
    return (Now_Omega);
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
