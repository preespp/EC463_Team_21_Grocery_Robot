/**
 * @file dvc_pc.h
 * @author
 * @brief PC control input over UART (frame + checksum)
 * @version 0.1
 * @date 2026-01-20
 */

#ifndef DVC_PC_H
#define DVC_PC_H

/* Includes ------------------------------------------------------------------*/

#include "2_Device/DR16/dvc_dr16.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief PC control frame header
 */
#define PC_CONTROL_FRAME_HEADER (0xAC)

/**
 * @brief PC control frame payload size (bytes)
 * Right_X, Right_Y, Left_X, Left_Y, Yaw (5 floats)
 * Keyboard mask (uint16)
 * Left/Right switch (uint8 each)
 */
#define PC_CONTROL_PAYLOAD_SIZE (24)

/**
 * @brief PC control frame total size (bytes)
 * Header + payload + checksum
 */
#define PC_CONTROL_FRAME_SIZE (26)

/**
 * @brief Specialized, PC control input (DR16-like data interface)
 */
class Class_PC
{
public:
    void Init(UART_HandleTypeDef *huart);

    inline Enum_DR16_Status Get_Status();

    inline float Get_Right_X();
    inline float Get_Right_Y();
    inline float Get_Left_X();
    inline float Get_Left_Y();
    inline float Get_Yaw();

    inline Enum_DR16_Switch_Status Get_Left_Switch();
    inline Enum_DR16_Switch_Status Get_Right_Switch();

    inline Enum_DR16_Key_Status Get_Keyboard_Key_W();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_S();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_A();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_D();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_Shift();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_Ctrl();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_Q();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_E();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_R();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_F();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_G();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_Z();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_X();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_C();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_V();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_B();

    bool UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_1ms_Calculate_PeriodElapsedCallback();

protected:
    // bound UART
    Struct_UART_Manage_Object *UART_Manage_Object = NULL;

    // last frame counters
    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;

    // raw inputs
    uint16_t Raw_Keyboard_Mask = 0;
    uint8_t Raw_Switch_Left = DR16_SWITCH_UP;
    uint8_t Raw_Switch_Right = DR16_SWITCH_UP;

    // previous raw inputs
    uint16_t Pre_Keyboard_Mask = 0;
    uint8_t Pre_Switch_Left = DR16_SWITCH_UP;
    uint8_t Pre_Switch_Right = DR16_SWITCH_UP;

    // output data (DR16-like)
    Enum_DR16_Status PC_Status = DR16_Status_DISABLE;
    Struct_DR16_Data Data;

    // stream parser cache to support fragmented UART DMA callbacks
    static constexpr uint16_t STREAM_CACHE_SIZE = 256;
    uint8_t Stream_Cache[STREAM_CACHE_SIZE] = {0};
    uint16_t Stream_Cache_Length = 0;

    bool Parse_Frame(uint8_t *Rx_Data, uint16_t Length);

    void _Judge_Switch(Enum_DR16_Switch_Status *Switch, uint8_t Status, uint8_t Pre_Status);
    void _Judge_Key(Enum_DR16_Key_Status *Key, uint8_t Status, uint8_t Pre_Status);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

inline Enum_DR16_Status Class_PC::Get_Status()
{
    return (PC_Status);
}

inline float Class_PC::Get_Right_X()
{
    return (Data.Right_X);
}

inline float Class_PC::Get_Right_Y()
{
    return (Data.Right_Y);
}

inline float Class_PC::Get_Left_X()
{
    return (Data.Left_X);
}

inline float Class_PC::Get_Left_Y()
{
    return (Data.Left_Y);
}

inline float Class_PC::Get_Yaw()
{
    return (Data.Yaw);
}

inline Enum_DR16_Switch_Status Class_PC::Get_Left_Switch()
{
    return (Data.Left_Switch);
}

inline Enum_DR16_Switch_Status Class_PC::Get_Right_Switch()
{
    return (Data.Right_Switch);
}

inline Enum_DR16_Key_Status Class_PC::Get_Keyboard_Key_W()
{
    return (Data.Keyboard_Key[DR16_KEY_W]);
}

inline Enum_DR16_Key_Status Class_PC::Get_Keyboard_Key_S()
{
    return (Data.Keyboard_Key[DR16_KEY_S]);
}

inline Enum_DR16_Key_Status Class_PC::Get_Keyboard_Key_A()
{
    return (Data.Keyboard_Key[DR16_KEY_A]);
}

inline Enum_DR16_Key_Status Class_PC::Get_Keyboard_Key_D()
{
    return (Data.Keyboard_Key[DR16_KEY_D]);
}

inline Enum_DR16_Key_Status Class_PC::Get_Keyboard_Key_Shift()
{
    return (Data.Keyboard_Key[DR16_KEY_SHIFT]);
}

inline Enum_DR16_Key_Status Class_PC::Get_Keyboard_Key_Ctrl()
{
    return (Data.Keyboard_Key[DR16_KEY_CTRL]);
}

inline Enum_DR16_Key_Status Class_PC::Get_Keyboard_Key_Q()
{
    return (Data.Keyboard_Key[DR16_KEY_Q]);
}

inline Enum_DR16_Key_Status Class_PC::Get_Keyboard_Key_E()
{
    return (Data.Keyboard_Key[DR16_KEY_E]);
}

inline Enum_DR16_Key_Status Class_PC::Get_Keyboard_Key_R()
{
    return (Data.Keyboard_Key[DR16_KEY_R]);
}

inline Enum_DR16_Key_Status Class_PC::Get_Keyboard_Key_F()
{
    return (Data.Keyboard_Key[DR16_KEY_F]);
}

inline Enum_DR16_Key_Status Class_PC::Get_Keyboard_Key_G()
{
    return (Data.Keyboard_Key[DR16_KEY_G]);
}

inline Enum_DR16_Key_Status Class_PC::Get_Keyboard_Key_Z()
{
    return (Data.Keyboard_Key[DR16_KEY_Z]);
}

inline Enum_DR16_Key_Status Class_PC::Get_Keyboard_Key_X()
{
    return (Data.Keyboard_Key[DR16_KEY_X]);
}

inline Enum_DR16_Key_Status Class_PC::Get_Keyboard_Key_C()
{
    return (Data.Keyboard_Key[DR16_KEY_C]);
}

inline Enum_DR16_Key_Status Class_PC::Get_Keyboard_Key_V()
{
    return (Data.Keyboard_Key[DR16_KEY_V]);
}

inline Enum_DR16_Key_Status Class_PC::Get_Keyboard_Key_B()
{
    return (Data.Keyboard_Key[DR16_KEY_B]);
}

#endif
