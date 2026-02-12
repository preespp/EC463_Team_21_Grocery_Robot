/**
 * @file dvc_imu_mpu6500.h
 * @brief On-board MPU6500 + IST8310 IMU driver (SPI5).
 */

#ifndef DVC_IMU_MPU6500_H
#define DVC_IMU_MPU6500_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    IMU_MPU6500_Status_DISABLE = 0,
    IMU_MPU6500_Status_ENABLE,
} Enum_IMU_MPU6500_Status;

uint8_t IMU_MPU6500_Init(void);
void IMU_MPU6500_Update(void);
Enum_IMU_MPU6500_Status IMU_MPU6500_Get_Status(void);

float IMU_MPU6500_Get_Omega_X(void);
float IMU_MPU6500_Get_Omega_Y(void);
float IMU_MPU6500_Get_Omega_Z(void);

float IMU_MPU6500_Get_Angle_Roll(void);
float IMU_MPU6500_Get_Angle_Pitch(void);
float IMU_MPU6500_Get_Angle_Yaw(void);

#ifdef __cplusplus
}
#endif

#endif
