/**
 * @file dvc_host_uart.h
 * @brief Host UART protocol for vx/vy/w command + IMU telemetry.
 */

#ifndef DVC_HOST_UART_H
#define DVC_HOST_UART_H

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/1_Driver/UART/drv_uart.h"
#include "1_Middleware/1_Driver/Math/drv_math.h"
#include <string.h>

/* Exported types ------------------------------------------------------------*/

enum Enum_Host_Status
{
    Host_Status_DISABLE = 0,
    Host_Status_ENABLE,
};

enum Enum_Host_Checksum_8
{
    Host_Checksum_8_DISABLE = 0,
    Host_Checksum_8_ENABLE,
};

struct Struct_Host_UART_Rx_Data
{
    uint16_t Frame_Header;
    float Velocity_X;
    float Velocity_Y;
    float Omega;
    uint8_t CRC_8;
} __attribute__((packed));

struct Struct_Host_UART_Tx_Data
{
    uint16_t Frame_Header;
    float Angle_Yaw;
    float Angle_Pitch;
    float Angle_Roll;
    float Omega_Z;
    float Velocity_X;
    float Velocity_Y;
    float Omega;
    uint8_t CRC_8;
} __attribute__((packed));

class Class_Host_UART
{
public:
    void Init(UART_HandleTypeDef *huart, uint16_t __Frame_Header_Rx = 0xA55A, uint16_t __Frame_Header_Tx = 0x5AA5, Enum_Host_Checksum_8 __Checksum_8 = Host_Checksum_8_ENABLE);

    inline Enum_Host_Status Get_Status();
    inline float Get_Target_Velocity_X();
    inline float Get_Target_Velocity_Y();
    inline float Get_Target_Omega();

    void Set_Tx_IMU(float yaw, float pitch, float roll, float omega_z);
    void Set_Tx_Chassis(float vx, float vy, float omega);

    void UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);
    void TIM_100ms_Alive_PeriodElapsedCallback();
    void TIM_10ms_Send_PeriodElapsedCallback();

protected:
    Struct_UART_Manage_Object *UART_Manage_Object = nullptr;
    Enum_Host_Checksum_8 Checksum_8 = Host_Checksum_8_ENABLE;
    uint16_t Frame_Header_Rx = 0xA55A;
    uint16_t Frame_Header_Tx = 0x5AA5;

    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;

    Enum_Host_Status Host_Status = Host_Status_DISABLE;

    Struct_Host_UART_Rx_Data Rx_Data = {0};
    Struct_Host_UART_Tx_Data Tx_Data = {0};

    void Output();
};

/* Inline definitions --------------------------------------------------------*/

inline Enum_Host_Status Class_Host_UART::Get_Status()
{
    return (Host_Status);
}

inline float Class_Host_UART::Get_Target_Velocity_X()
{
    return (Rx_Data.Velocity_X);
}

inline float Class_Host_UART::Get_Target_Velocity_Y()
{
    return (Rx_Data.Velocity_Y);
}

inline float Class_Host_UART::Get_Target_Omega()
{
    return (Rx_Data.Omega);
}

#endif

