/**
 * @file tsk_config_and_callback.cpp
 * @brief Task scheduler and callbacks for chassis-only control
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"

#include "4_Interaction/ita_robot.h"
#include "2_Device/Serialplot/dvc_serialplot.h"
#include "1_Middleware/1_Driver/BSP/drv_djiboarda.h"
#include "1_Middleware/1_Driver/TIM/drv_tim.h"
#include "1_Middleware/1_Driver/WDG/drv_wdg.h"

/* Private variables ---------------------------------------------------------*/

bool init_finished = false;
uint32_t flag = 0;

Class_Robot robot;
Class_Serialplot serialplot;

/* Callback functions --------------------------------------------------------*/

void Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    case (0x201):
    {
        robot.Chassis.Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
        break;
    }
    case (0x202):
    {
        robot.Chassis.Motor_Wheel[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
        break;
    }
    case (0x203):
    {
        robot.Chassis.Motor_Wheel[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
        break;
    }
    case (0x204):
    {
        robot.Chassis.Motor_Wheel[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
        break;
    }
    }
}

void PC_UART2_Callback(uint8_t *Buffer, uint16_t Length)
{
    // UART2 is shared: try PC frame parse first, then serialplot parse.
    if (!robot.PC.UART_RxCpltCallback(Buffer, Length))
    {
        serialplot.UART_RxCpltCallback(Buffer, Length);
    }
}

void Task100us_TIM4_Callback()
{
}

void Task1ms_TIM5_Callback()
{
    static int alive_mod100 = 0;
    alive_mod100++;
    if (alive_mod100 == 100)
    {
        alive_mod100 = 0;
        robot.TIM_100ms_Alive_PeriodElapsedCallback();
    }

    static int alive_mod1000 = 0;
    alive_mod1000++;
    if (alive_mod1000 == 1000)
    {
        alive_mod1000 = 0;
        robot.TIM_1000ms_Alive_PeriodElapsedCallback();
    }

    static int interaction_mod10 = 0;
    interaction_mod10++;
    if (interaction_mod10 == 10)
    {
        interaction_mod10 = 0;
        robot.TIM_10ms_Calculate_PeriodElapsedCallback();
    }

    static int interaction_mod2 = 0;
    interaction_mod2++;
    if (interaction_mod2 == 2)
    {
        interaction_mod2 = 0;
        robot.TIM_2ms_Calculate_PeriodElapsedCallback();
    }

    robot.TIM_1ms_Calculate_Callback();

    // UART2 telemetry throttled to 200 Hz:
    // frame size = 1(header) + 6*4(float) + 1(checksum) = 26 bytes
    // 26 bytes * 10 bits/byte * 200 Hz = 52 kbps (safe at 115200 bps)
    static uint8_t telemetry_mod5 = 0;
    telemetry_mod5++;
    if (telemetry_mod5 >= 5)
    {
        telemetry_mod5 = 0;

        float target_vx = robot.Chassis.Get_Target_Velocity_X();
        float target_vy = robot.Chassis.Get_Target_Velocity_Y();
        float target_omega = robot.Chassis.Get_Target_Omega();
        float now_vx = robot.Chassis.Get_Now_Velocity_X();
        float now_vy = robot.Chassis.Get_Now_Velocity_Y();
        float now_omega = robot.Chassis.Get_Now_Omega();
        // Serialplot channels: target vx/vy/omega, now vx/vy/omega.
        serialplot.Set_Data(6, &target_vx, &target_vy, &target_omega,
                            &now_vx, &now_vy, &now_omega);
        serialplot.TIM_1ms_Write_PeriodElapsedCallback();
    }

    TIM_1ms_CAN_PeriodElapsedCallback();
    TIM_1ms_UART_PeriodElapsedCallback();
    TIM_1ms_IWDG_PeriodElapsedCallback();
    flag++;
}

/* Task entry points ---------------------------------------------------------*/

void Task_Init()
{
    BSP_Init(BSP_LED_R_ON | BSP_LED_G_ON);

    CAN_Init(&hcan1, Device_CAN1_Callback);
    UART_Init(&huart2, PC_UART2_Callback, SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH);

    TIM_Init(&htim4, Task100us_TIM4_Callback);
    TIM_Init(&htim5, Task1ms_TIM5_Callback);
    IWDG_Independent_Feed();

    serialplot.Init(&huart2, Serialplot_Checksum_8_ENABLE);

    robot.Init();

    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
    init_finished = true;

    HAL_Delay(2000);
}

void Task_Loop()
{
    robot.Loop();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
