/**
 * @file dvc_host_uart.cpp
 * @brief Host UART protocol for vx/vy/w command + IMU telemetry.
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_host_uart.h"

/* Function definitions ------------------------------------------------------*/

void Class_Host_UART::Init(UART_HandleTypeDef *huart, uint16_t __Frame_Header_Rx, uint16_t __Frame_Header_Tx, Enum_Host_Checksum_8 __Checksum_8)
{
    if (huart->Instance == USART1)
    {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (huart->Instance == USART2)
    {
        UART_Manage_Object = &UART2_Manage_Object;
    }
    else if (huart->Instance == USART3)
    {
        UART_Manage_Object = &UART3_Manage_Object;
    }
    else if (huart->Instance == UART4)
    {
        UART_Manage_Object = &UART4_Manage_Object;
    }
    else if (huart->Instance == UART5)
    {
        UART_Manage_Object = &UART5_Manage_Object;
    }
    else if (huart->Instance == USART6)
    {
        UART_Manage_Object = &UART6_Manage_Object;
    }
    else if (huart->Instance == UART7)
    {
        UART_Manage_Object = &UART7_Manage_Object;
    }
    else if (huart->Instance == UART8)
    {
        UART_Manage_Object = &UART8_Manage_Object;
    }

    Frame_Header_Rx = __Frame_Header_Rx;
    Frame_Header_Tx = __Frame_Header_Tx;
    Checksum_8 = __Checksum_8;
}

void Class_Host_UART::UART_RxCpltCallback(uint8_t *Rx_Data_Buffer, uint16_t Length)
{
    const uint16_t frame_len = sizeof(Struct_Host_UART_Rx_Data);

    if (Length < frame_len)
    {
        return;
    }

    for (uint16_t offset = 0; offset + frame_len <= Length; offset++)
    {
        uint16_t header = (uint16_t) Rx_Data_Buffer[offset] | ((uint16_t) Rx_Data_Buffer[offset + 1] << 8);
        if (header != Frame_Header_Rx)
        {
            continue;
        }

        if (Checksum_8 == Host_Checksum_8_ENABLE)
        {
            uint8_t sum = Math_Sum_8(&Rx_Data_Buffer[offset], frame_len - 1);
            if (sum != Rx_Data_Buffer[offset + frame_len - 1])
            {
                continue;
            }
        }

        Struct_Host_UART_Rx_Data tmp_data;
        memcpy(&tmp_data, &Rx_Data_Buffer[offset], frame_len);
        Rx_Data = tmp_data;
        Flag++;
        break;
    }
}

void Class_Host_UART::TIM_100ms_Alive_PeriodElapsedCallback()
{
    if (Flag == Pre_Flag)
    {
        Host_Status = Host_Status_DISABLE;
        Rx_Data.Velocity_X = 0.0f;
        Rx_Data.Velocity_Y = 0.0f;
        Rx_Data.Omega = 0.0f;
    }
    else
    {
        Host_Status = Host_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

void Class_Host_UART::Set_Tx_IMU(float yaw, float pitch, float roll, float omega_z)
{
    Tx_Data.Angle_Yaw = yaw;
    Tx_Data.Angle_Pitch = pitch;
    Tx_Data.Angle_Roll = roll;
    Tx_Data.Omega_Z = omega_z;
}

void Class_Host_UART::Set_Tx_Chassis(float vx, float vy, float omega)
{
    Tx_Data.Velocity_X = vx;
    Tx_Data.Velocity_Y = vy;
    Tx_Data.Omega = omega;
}

void Class_Host_UART::TIM_10ms_Send_PeriodElapsedCallback()
{
    if (UART_Manage_Object == nullptr)
    {
        return;
    }

    Output();
}

void Class_Host_UART::Output()
{
    Tx_Data.Frame_Header = Frame_Header_Tx;

    if (Checksum_8 == Host_Checksum_8_ENABLE)
    {
        uint8_t *raw = reinterpret_cast<uint8_t *>(&Tx_Data);
        Tx_Data.CRC_8 = Math_Sum_8(raw, sizeof(Struct_Host_UART_Tx_Data) - 1);
    }
    else
    {
        Tx_Data.CRC_8 = 0;
    }

    UART_Send_Data(UART_Manage_Object->UART_Handler, reinterpret_cast<uint8_t *>(&Tx_Data), sizeof(Struct_Host_UART_Tx_Data));
}
