/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "1_Middleware/1_Driver/BSP/drv_djiboarda.h"
#include "1_Middleware/1_Driver/CAN/drv_can.h"
#include "1_Middleware/1_Driver/TIM/drv_tim.h"
#include "1_Middleware/1_Driver/UART/drv_uart.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "2_Device/IMU/MPU6500/dvc_imu_mpu6500.h"
#ifdef __cplusplus
}
#endif
#include "2_Device/Host/dvc_host_uart.h"
#include "2_Device/Serialplot/dvc_serialplot.h"

#include "3_Chariot/1_Module/Chassis/crt_chassis.h"

#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

bool init_finished = false;
static bool can2_inited = false;

Class_Serialplot serialplot;
Class_Host_UART host_uart;
Class_Chassis chassis;

static float debug_target_vx = 0.0f;
static float debug_target_vy = 0.0f;
static float debug_target_w = 0.0f;
static float debug_now_vx = 0.0f;
static float debug_now_vy = 0.0f;
static float debug_now_w = 0.0f;
static float debug_yaw = 0.0f;
static float serialplot_target_angle = 0.0f;
static float serialplot_now_angle = 0.0f;
static float serialplot_target_omega = 0.0f;
static float serialplot_now_omega = 0.0f;
static float serialplot_target_current = 0.0f;
static float serialplot_now_current = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static const bool kLedActiveLow = true;
static const bool kLedGreenActiveLow = true;
static const bool kLedGreenDiagBlink = true;

static Enum_BSP_LED_Status BoardA_LedStatus(bool on, bool active_low)
{
  return active_low
           ? (on ? BSP_LED_Status_ENABLED : BSP_LED_Status_DISABLED)
           : (on ? BSP_LED_Status_DISABLED : BSP_LED_Status_ENABLED);
}

static void BoardA_Set_All_LEDs(bool on)
{
  Enum_BSP_LED_Status status = BoardA_LedStatus(on, kLedActiveLow);
  BSP_Set_LED_R(status);
  BSP_Set_LED_1(status);
  BSP_Set_LED_2(status);
  BSP_Set_LED_3(status);
  BSP_Set_LED_4(status);
  BSP_Set_LED_5(status);
  BSP_Set_LED_6(status);
  BSP_Set_LED_7(status);
  BSP_Set_LED_8(status);
  BSP_Set_LED_G(BoardA_LedStatus(on, kLedGreenActiveLow));
}

static void BoardA_Set_Only_LED_G(void)
{
  Enum_BSP_LED_Status on_status = BoardA_LedStatus(true, kLedGreenActiveLow);
  Enum_BSP_LED_Status off_status = BoardA_LedStatus(false, kLedActiveLow);
  BSP_Set_LED_R(off_status);
  BSP_Set_LED_G(on_status);
  BSP_Set_LED_1(off_status);
  BSP_Set_LED_2(off_status);
  BSP_Set_LED_3(off_status);
  BSP_Set_LED_4(off_status);
  BSP_Set_LED_5(off_status);
  BSP_Set_LED_6(off_status);
  BSP_Set_LED_7(off_status);
  BSP_Set_LED_8(off_status);
}

static void BoardA_Update_LED_By_DC24(void)
{
  static bool led_all_on = true;
  bool dc24_detected = (BSP_Get_DC24_LU() == BSP_DC24_Status_ENABLED) ||
                       (BSP_Get_DC24_LD() == BSP_DC24_Status_ENABLED) ||
                       (BSP_Get_DC24_RU() == BSP_DC24_Status_ENABLED) ||
                       (BSP_Get_DC24_RD() == BSP_DC24_Status_ENABLED);
  bool want_all_on = !dc24_detected;

  if (want_all_on == led_all_on)
  {
    return;
  }

  led_all_on = want_all_on;
  if (led_all_on)
  {
    BoardA_Set_All_LEDs(true);
  }
  else
  {
    BoardA_Set_Only_LED_G();
  }
}

static void BoardA_Debug_Blink_LedG_10ms(void)
{
  static uint32_t tick_10ms = 0;
  tick_10ms++;
  if (tick_10ms >= 50)
  {
    tick_10ms = 0;
    HAL_GPIO_TogglePin(BoardA_LED_G_GPIO_Port, BoardA_LED_G_Pin);
  }
}

/**
 * @brief CAN2 Rx callback for chassis motors
 *
 * @param Rx_Buffer CAN接收的信息结构体
 */
void Device_CAN2_Callback(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
  switch (Rx_Buffer->Header.StdId)
  {
  case (0x201):
  {
    chassis.Motor_Wheel[Chassis_Wheel_Front_Left].CAN_RxCpltCallback(Rx_Buffer->Data);
    break;
  }
  case (0x202):
  {
    chassis.Motor_Wheel[Chassis_Wheel_Front_Right].CAN_RxCpltCallback(Rx_Buffer->Data);
    break;
  }
  case (0x203):
  {
    chassis.Motor_Wheel[Chassis_Wheel_Rear_Left].CAN_RxCpltCallback(Rx_Buffer->Data);
    break;
  }
  case (0x204):
  {
    chassis.Motor_Wheel[Chassis_Wheel_Rear_Right].CAN_RxCpltCallback(Rx_Buffer->Data);
    break;
  }
  }
}

/**
 * @brief UART2 serialplot callback
 *
 * @param Buffer UART2收到的消息
 * @param Length 长度
 */
void Serialplot_UART2_Callback(uint8_t *Buffer, uint16_t Length)
{
  serialplot.UART_RxCpltCallback(Buffer, Length);
}

/**
 * @brief UART3 host callback
 *
 * @param Buffer UART3收到的消息
 * @param Length 长度
 */
void Host_UART3_Callback(uint8_t *Buffer, uint16_t Length)
{
  host_uart.UART_RxCpltCallback(Buffer, Length);
}

/**
 * @brief UART7 chassis IMU callback
 *
 * @param Buffer UART7收到的消息
 * @param Length 长度
 */
/**
 * @brief TIM4 100us task
 */
void Task100us_TIM4_Callback()
{

}

/**
 * @brief TIM5 1ms task
 */
void Task1ms_TIM5_Callback()
{
  static uint32_t tick_1ms = 0;
  tick_1ms++;

  float cmd_vx = host_uart.Get_Target_Velocity_X();
  float cmd_vy = host_uart.Get_Target_Velocity_Y();
  float cmd_w = host_uart.Get_Target_Omega();
  if (host_uart.Get_Status() == Host_Status_DISABLE)
  {
    cmd_vx = 0.0f;
    cmd_vy = 0.0f;
    cmd_w = 0.0f;
  }

  chassis.Set_Target_Velocity_X(cmd_vx);
  chassis.Set_Target_Velocity_Y(cmd_vy);
  chassis.Set_Target_Omega(cmd_w);

  if ((tick_1ms % 2) == 0)
  {
    IMU_MPU6500_Update();
    chassis.TIM_2ms_Resolution_PeriodElapsedCallback();
    chassis.TIM_2ms_Control_PeriodElapsedCallback();
  }

  if ((tick_1ms % 10) == 0)
  {
    debug_target_vx = cmd_vx;
    debug_target_vy = cmd_vy;
    debug_target_w = cmd_w;
    debug_now_vx = chassis.Get_Now_Velocity_X();
    debug_now_vy = chassis.Get_Now_Velocity_Y();
    debug_now_w = chassis.Get_Now_Omega();

    float imu_yaw = 0.0f;
    float imu_pitch = 0.0f;
    float imu_roll = 0.0f;
    float imu_wz = 0.0f;
    if (IMU_MPU6500_Get_Status() == IMU_MPU6500_Status_ENABLE)
    {
      imu_yaw = IMU_MPU6500_Get_Angle_Yaw();
      imu_pitch = IMU_MPU6500_Get_Angle_Pitch();
      imu_roll = IMU_MPU6500_Get_Angle_Roll();
      imu_wz = IMU_MPU6500_Get_Omega_Z();
    }
    if (isnan(imu_yaw))
    {
      imu_yaw = 0.0f;
    }
    debug_yaw = imu_yaw;

    Class_Motor_DJI_C620 *motor = &chassis.Motor_Wheel[Chassis_Wheel_Front_Left];
    serialplot_target_angle = motor->Get_Target_Angle();
    serialplot_now_angle = motor->Get_Now_Angle();
    serialplot_target_omega = motor->Get_Target_Omega() + motor->Get_Feedforward_Omega();
    serialplot_now_omega = motor->Get_Now_Omega();
    serialplot_target_current = motor->Get_Target_Current() + motor->Get_Feedforward_Current();
    serialplot_now_current = motor->Get_Now_Current();

    if (kLedGreenDiagBlink)
    {
      if (can2_inited)
      {
        BoardA_Debug_Blink_LedG_10ms();
      }
    }
    else
    {
      BoardA_Update_LED_By_DC24();
    }

    serialplot.TIM_1ms_Write_PeriodElapsedCallback();

    host_uart.Set_Tx_IMU(imu_yaw,
                         imu_pitch,
                         imu_roll,
                         imu_wz);
    host_uart.Set_Tx_Chassis(debug_now_vx, debug_now_vy, debug_now_w);
    host_uart.TIM_10ms_Send_PeriodElapsedCallback();
  }

  if ((tick_1ms % 100) == 0)
  {
    chassis.TIM_100ms_Alive_PeriodElapsedCallback();
    host_uart.TIM_100ms_Alive_PeriodElapsedCallback();
  }

  TIM_1ms_CAN_PeriodElapsedCallback();
  TIM_1ms_UART_PeriodElapsedCallback();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM12_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_ADC1_Init();
  MX_SPI5_Init();
  /* USER CODE BEGIN 2 */

  BSP_Init(BSP_DC24_LU_ON | BSP_DC24_LD_ON | BSP_DC24_RU_ON | BSP_DC24_RD_ON);
  BoardA_Set_All_LEDs(true);
  IMU_MPU6500_Init();
  CAN_Init(&hcan2, Device_CAN2_Callback);
  can2_inited = true;
  UART_Init(&huart2, Serialplot_UART2_Callback, SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH);
  UART_Init(&huart3, Host_UART3_Callback, sizeof(Struct_Host_UART_Rx_Data));

  serialplot.Init(&huart2, Serialplot_Checksum_8_ENABLE, 0, NULL);
  serialplot.Set_Data(6, &serialplot_target_angle, &serialplot_now_angle, &serialplot_target_omega,
                      &serialplot_now_omega, &serialplot_target_current, &serialplot_now_current);

  host_uart.Init(&huart3);
  chassis.Init(&hcan2);
  chassis.Set_Chassis_Control_Type(Chassis_Control_Type_NORMAL);

  TIM_Init(&htim4, Task100us_TIM4_Callback);
  TIM_Init(&htim5, Task1ms_TIM5_Callback);

  init_finished = true;

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim5);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    __WFI();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = BoardA_LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BoardA_LED_R_GPIO_Port, &GPIO_InitStruct);
  while (1)
  {
    HAL_GPIO_TogglePin(BoardA_LED_R_GPIO_Port, BoardA_LED_R_Pin);
    for (volatile uint32_t i = 0; i < 300000; ++i)
    {
      __NOP();
    }
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
