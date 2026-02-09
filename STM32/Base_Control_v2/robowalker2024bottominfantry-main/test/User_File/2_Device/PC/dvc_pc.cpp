/**
 * @file dvc_pc.cpp
 * @brief PC control input over UART (frame + checksum)
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_pc.h"
#include "1_Middleware/1_Driver/Math/drv_math.h"
#include <string.h>

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

void Class_PC::Init(UART_HandleTypeDef *huart)
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

    Data.Right_X = 0.0f;
    Data.Right_Y = 0.0f;
    Data.Left_X = 0.0f;
    Data.Left_Y = 0.0f;
    Data.Yaw = 0.0f;
    Data.Left_Switch = DR16_Switch_Status_UP;
    Data.Right_Switch = DR16_Switch_Status_UP;
    Data.Mouse_X = 0.0f;
    Data.Mouse_Y = 0.0f;
    Data.Mouse_Z = 0.0f;
    Data.Mouse_Left_Key = DR16_Key_Status_FREE;
    Data.Mouse_Right_Key = DR16_Key_Status_FREE;
    for (int i = 0; i < 16; i++)
    {
        Data.Keyboard_Key[i] = DR16_Key_Status_FREE;
    }
}

bool Class_PC::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    if (Parse_Frame(Rx_Data, Length))
    {
        Flag += 1;
        return true;
    }

    return false;
}

void Class_PC::TIM_100ms_Alive_PeriodElapsedCallback()
{
    if (Flag == Pre_Flag)
    {
        PC_Status = DR16_Status_DISABLE;
        UART_Reinit(UART_Manage_Object->UART_Handler);
    }
    else
    {
        PC_Status = DR16_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

void Class_PC::TIM_1ms_Calculate_PeriodElapsedCallback()
{
    _Judge_Switch(&Data.Left_Switch, Raw_Switch_Left, Pre_Switch_Left);
    _Judge_Switch(&Data.Right_Switch, Raw_Switch_Right, Pre_Switch_Right);

    for (int i = 0; i < 16; i++)
    {
        uint8_t now_key = (Raw_Keyboard_Mask >> i) & 0x1;
        uint8_t pre_key = (Pre_Keyboard_Mask >> i) & 0x1;
        _Judge_Key(&Data.Keyboard_Key[i], now_key, pre_key);
    }

    Pre_Switch_Left = Raw_Switch_Left;
    Pre_Switch_Right = Raw_Switch_Right;
    Pre_Keyboard_Mask = Raw_Keyboard_Mask;
}

bool Class_PC::Parse_Frame(uint8_t *Rx_Data, uint16_t Length)
{
    if (Length == 0)
    {
        return false;
    }

    // Append incoming bytes into local stream cache.
    if (Length >= STREAM_CACHE_SIZE)
    {
        memcpy(Stream_Cache, Rx_Data + (Length - STREAM_CACHE_SIZE), STREAM_CACHE_SIZE);
        Stream_Cache_Length = STREAM_CACHE_SIZE;
    }
    else
    {
        uint16_t copy_len = Length;
        if ((Stream_Cache_Length + copy_len) > STREAM_CACHE_SIZE)
        {
            uint16_t drop = Stream_Cache_Length + copy_len - STREAM_CACHE_SIZE;
            if (drop >= Stream_Cache_Length)
            {
                Stream_Cache_Length = 0;
            }
            else
            {
                memmove(Stream_Cache, Stream_Cache + drop, Stream_Cache_Length - drop);
                Stream_Cache_Length -= drop;
            }
        }
        memcpy(Stream_Cache + Stream_Cache_Length, Rx_Data, copy_len);
        Stream_Cache_Length += copy_len;
    }

    bool parsed_any = false;
    while (Stream_Cache_Length >= PC_CONTROL_FRAME_SIZE)
    {
        uint16_t header_index = 0;
        while (header_index < Stream_Cache_Length && Stream_Cache[header_index] != PC_CONTROL_FRAME_HEADER)
        {
            header_index++;
        }

        if (header_index >= Stream_Cache_Length)
        {
            Stream_Cache_Length = 0;
            break;
        }

        if (header_index > 0)
        {
            memmove(Stream_Cache, Stream_Cache + header_index, Stream_Cache_Length - header_index);
            Stream_Cache_Length -= header_index;
        }

        if (Stream_Cache_Length < PC_CONTROL_FRAME_SIZE)
        {
            break;
        }

        uint8_t checksum = Math_Sum_8(Stream_Cache + 1, PC_CONTROL_PAYLOAD_SIZE);
        uint8_t rx_checksum = Stream_Cache[1 + PC_CONTROL_PAYLOAD_SIZE];
        if (checksum != rx_checksum)
        {
            // Bad frame at current header, drop one byte and re-sync.
            memmove(Stream_Cache, Stream_Cache + 1, Stream_Cache_Length - 1);
            Stream_Cache_Length -= 1;
            continue;
        }

        const uint8_t *payload = Stream_Cache + 1;
        memcpy(&Data.Right_X, payload + 0, sizeof(float));
        memcpy(&Data.Right_Y, payload + 4, sizeof(float));
        memcpy(&Data.Left_X, payload + 8, sizeof(float));
        memcpy(&Data.Left_Y, payload + 12, sizeof(float));
        memcpy(&Data.Yaw, payload + 16, sizeof(float));

        Raw_Keyboard_Mask = (uint16_t) (payload[20] | (payload[21] << 8));
        Raw_Switch_Left = payload[22];
        Raw_Switch_Right = payload[23];

        // Consume parsed frame and keep scanning so the freshest frame wins.
        memmove(Stream_Cache, Stream_Cache + PC_CONTROL_FRAME_SIZE, Stream_Cache_Length - PC_CONTROL_FRAME_SIZE);
        Stream_Cache_Length -= PC_CONTROL_FRAME_SIZE;
        parsed_any = true;
    }

    return parsed_any;
}

void Class_PC::_Judge_Switch(Enum_DR16_Switch_Status *Switch, uint8_t Status, uint8_t Pre_Status)
{
    switch (Pre_Status)
    {
    case (DR16_SWITCH_UP):
    {
        switch (Status)
        {
        case (DR16_SWITCH_UP):
            *Switch = DR16_Switch_Status_UP;
            break;
        case (DR16_SWITCH_DOWN):
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_DOWN;
            break;
        case (DR16_SWITCH_MIDDLE):
            *Switch = DR16_Switch_Status_TRIG_UP_MIDDLE;
            break;
        }
        break;
    }
    case (DR16_SWITCH_DOWN):
    {
        switch (Status)
        {
        case (DR16_SWITCH_UP):
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_UP;
            break;
        case (DR16_SWITCH_DOWN):
            *Switch = DR16_Switch_Status_DOWN;
            break;
        case (DR16_SWITCH_MIDDLE):
            *Switch = DR16_Switch_Status_TRIG_DOWN_MIDDLE;
            break;
        }
        break;
    }
    case (DR16_SWITCH_MIDDLE):
    {
        switch (Status)
        {
        case (DR16_SWITCH_UP):
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_UP;
            break;
        case (DR16_SWITCH_DOWN):
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_DOWN;
            break;
        case (DR16_SWITCH_MIDDLE):
            *Switch = DR16_Switch_Status_MIDDLE;
            break;
        }
        break;
    }
    }
}

void Class_PC::_Judge_Key(Enum_DR16_Key_Status *Key, uint8_t Status, uint8_t Pre_Status)
{
    switch (Pre_Status)
    {
    case (DR16_KEY_FREE):
    {
        switch (Status)
        {
        case (DR16_KEY_FREE):
            *Key = DR16_Key_Status_FREE;
            break;
        case (DR16_KEY_PRESSED):
            *Key = DR16_Key_Status_TRIG_FREE_PRESSED;
            break;
        }
        break;
    }
    case (DR16_KEY_PRESSED):
    {
        switch (Status)
        {
        case (DR16_KEY_FREE):
            *Key = DR16_Key_Status_TRIG_PRESSED_FREE;
            break;
        case (DR16_KEY_PRESSED):
            *Key = DR16_Key_Status_PRESSED;
            break;
        }
        break;
    }
    }
}
