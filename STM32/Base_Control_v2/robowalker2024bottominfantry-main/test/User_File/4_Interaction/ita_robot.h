/**
 * @file ita_robot.h
 * @brief PC-controlled chassis logic (no gimbal/supercap/referee)
 */

#ifndef ITA_ROBOT_H
#define ITA_ROBOT_H

/* Includes ------------------------------------------------------------------*/

#include "3_Chariot/1_Module/Chassis/crt_chassis.h"
#include "2_Device/PC/dvc_pc.h"
#include "1_Middleware/2_Algorithm/Slope/alg_slope.h"

/* Exported types ------------------------------------------------------------*/

enum Enum_Robot_Gyroscope_Type
{
    Robot_Gyroscope_Type_DISABLE = 0,
    Robot_Gyroscope_Type_CLOCKWISE,
    Robot_Gyroscope_Type_COUNTERCLOCKWISE,
};

class Class_Robot
{
public:
    Class_PC PC;
    Class_Chassis Chassis;

    void Init();
    void Loop();

    void TIM_1000ms_Alive_PeriodElapsedCallback();
    void TIM_100ms_Alive_PeriodElapsedCallback();
    void TIM_10ms_Calculate_PeriodElapsedCallback();
    void TIM_2ms_Calculate_PeriodElapsedCallback();
    void TIM_1ms_Calculate_Callback();

protected:
    // input parameters
    float PC_Rocker_Dead_Zone = 0.03f;

    // speed planning
    Class_Slope Slope_Speed_X;
    Class_Slope Slope_Speed_Y;
    Class_Slope Slope_Omega;

    // chassis modes
    Enum_Robot_Gyroscope_Type Chassis_Gyroscope_Mode_Status = Robot_Gyroscope_Type_DISABLE;

    void _Chassis_Control();
};

#endif

