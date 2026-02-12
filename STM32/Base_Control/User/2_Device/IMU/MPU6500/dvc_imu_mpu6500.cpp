/**
 * @file dvc_imu_mpu6500.cpp
 * @brief On-board MPU6500 + IST8310 IMU driver (SPI5).
 */

#include "dvc_imu_mpu6500.h"

#include "ist8310_reg.h"
#include "mpu6500_reg.h"

#include "main.h"
#include "spi.h"

#include <math.h>
#include <string.h>

#define BOARD_DOWN (1)
#define IST8310 (1)

#define MPU_DELAY(x) HAL_Delay(x)
#define MPU_HSPI hspi5
#define MPU_NSS_LOW HAL_GPIO_WritePin(BoardA_MPU6500_CS_GPIO_Port, BoardA_MPU6500_CS_Pin, GPIO_PIN_RESET)
#define MPU_NSS_HIGH HAL_GPIO_WritePin(BoardA_MPU6500_CS_GPIO_Port, BoardA_MPU6500_CS_Pin, GPIO_PIN_SET)

#define Kp 2.0f
#define Ki 0.01f

struct Struct_MPU6500_Raw
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t mx;
    int16_t my;
    int16_t mz;
    int16_t temp;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t ax_offset;
    int16_t ay_offset;
    int16_t az_offset;
    int16_t gx_offset;
    int16_t gy_offset;
    int16_t gz_offset;
};

struct Struct_MPU6500_Output
{
    float ax;
    float ay;
    float az;
    float mx;
    float my;
    float mz;
    float temp;
    float wx;
    float wy;
    float wz;
    float rol;
    float pit;
    float yaw;
};

static Enum_IMU_MPU6500_Status g_status = IMU_MPU6500_Status_DISABLE;

static Struct_MPU6500_Raw g_mpu_data = {};
static Struct_MPU6500_Output g_imu = {};

static uint8_t g_mpu_buff[14] = {0};
static uint8_t g_ist_buff[6] = {0};

static uint8_t g_tx = 0;
static uint8_t g_rx = 0;
static uint8_t g_tx_buff[14] = {0xff};

static volatile float q0 = 1.0f;
static volatile float q1 = 0.0f;
static volatile float q2 = 0.0f;
static volatile float q3 = 0.0f;
static volatile float exInt = 0.0f;
static volatile float eyInt = 0.0f;
static volatile float ezInt = 0.0f;
static volatile uint32_t last_update = 0;
static volatile uint32_t now_update = 0;

static float inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;

    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));

    return y;
}

static uint8_t mpu_write_byte(uint8_t reg, uint8_t data)
{
    MPU_NSS_LOW;
    g_tx = reg & 0x7F;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &g_tx, &g_rx, 1, 55);
    g_tx = data;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &g_tx, &g_rx, 1, 55);
    MPU_NSS_HIGH;
    return 0;
}

static uint8_t mpu_read_byte(uint8_t reg)
{
    MPU_NSS_LOW;
    g_tx = reg | 0x80;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &g_tx, &g_rx, 1, 55);
    HAL_SPI_TransmitReceive(&MPU_HSPI, &g_tx, &g_rx, 1, 55);
    MPU_NSS_HIGH;
    return g_rx;
}

static uint8_t mpu_read_bytes(uint8_t reg, uint8_t *data, uint8_t len)
{
    MPU_NSS_LOW;
    g_tx = reg | 0x80;
    g_tx_buff[0] = g_tx;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &g_tx, &g_rx, 1, 55);
    HAL_SPI_TransmitReceive(&MPU_HSPI, g_tx_buff, data, len, 55);
    MPU_NSS_HIGH;
    return 0;
}

static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data)
{
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, addr);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, data);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    MPU_DELAY(10);
}

static uint8_t ist_reg_read_by_mpu(uint8_t addr)
{
    uint8_t retval = 0;
    mpu_write_byte(MPU6500_I2C_SLV4_REG, addr);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x80);
    MPU_DELAY(10);
    retval = mpu_read_byte(MPU6500_I2C_SLV4_DI);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);
    return retval;
}

static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, device_address);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
    MPU_DELAY(2);

    mpu_write_byte(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV0_REG, reg_base_addr);
    MPU_DELAY(2);

    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x03);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    MPU_DELAY(6);
    mpu_write_byte(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
    MPU_DELAY(2);
}

static uint8_t ist8310_init(void)
{
    mpu_write_byte(MPU6500_USER_CTRL, 0x30);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_MST_CTRL, 0x0d);
    MPU_DELAY(10);

    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
    MPU_DELAY(10);

    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
    MPU_DELAY(10);
    if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
    {
        return 1;
    }

    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
    MPU_DELAY(10);

    ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00);
    if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
    {
        return 2;
    }
    MPU_DELAY(10);

    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00);
    if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
    {
        return 3;
    }
    MPU_DELAY(10);

    ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24);
    if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
    {
        return 4;
    }
    MPU_DELAY(10);

    ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);
    if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
    {
        return 5;
    }
    MPU_DELAY(10);

    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);

    mpu_master_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
    MPU_DELAY(100);
    return 0;
}

static void ist8310_get_data(uint8_t *buff)
{
    mpu_read_bytes(MPU6500_EXT_SENS_DATA_00, buff, 6);
}

static void mpu_get_data(void)
{
    mpu_read_bytes(MPU6500_ACCEL_XOUT_H, g_mpu_buff, 14);

    g_mpu_data.ax = (int16_t)(g_mpu_buff[0] << 8 | g_mpu_buff[1]);
    g_mpu_data.ay = (int16_t)(g_mpu_buff[2] << 8 | g_mpu_buff[3]);
    g_mpu_data.az = (int16_t)(g_mpu_buff[4] << 8 | g_mpu_buff[5]);
    g_mpu_data.temp = (int16_t)(g_mpu_buff[6] << 8 | g_mpu_buff[7]);

    g_mpu_data.gx = (int16_t)((g_mpu_buff[8] << 8 | g_mpu_buff[9]) - g_mpu_data.gx_offset);
    g_mpu_data.gy = (int16_t)((g_mpu_buff[10] << 8 | g_mpu_buff[11]) - g_mpu_data.gy_offset);
    g_mpu_data.gz = (int16_t)((g_mpu_buff[12] << 8 | g_mpu_buff[13]) - g_mpu_data.gz_offset);

    ist8310_get_data(g_ist_buff);
    memcpy(&g_mpu_data.mx, g_ist_buff, 6);

    g_imu.ax = (float)g_mpu_data.ax;
    g_imu.ay = (float)g_mpu_data.ay;
    g_imu.az = (float)g_mpu_data.az;
    g_imu.mx = (float)g_mpu_data.mx;
    g_imu.my = (float)g_mpu_data.my;
    g_imu.mz = (float)g_mpu_data.mz;

    g_imu.temp = 21.0f + g_mpu_data.temp / 333.87f;

    g_imu.wx = g_mpu_data.gx / 16.384f / 57.3f;
    g_imu.wy = g_mpu_data.gy / 16.384f / 57.3f;
    g_imu.wz = g_mpu_data.gz / 16.384f / 57.3f;
}

static uint8_t mpu_set_gyro_fsr(uint8_t fsr)
{
    return mpu_write_byte(MPU6500_GYRO_CONFIG, fsr << 3);
}

static uint8_t mpu_set_accel_fsr(uint8_t fsr)
{
    return mpu_write_byte(MPU6500_ACCEL_CONFIG, fsr << 3);
}

static void mpu_offset_call(void)
{
    g_mpu_data.ax_offset = 0;
    g_mpu_data.ay_offset = 0;
    g_mpu_data.az_offset = 0;
    g_mpu_data.gx_offset = 0;
    g_mpu_data.gy_offset = 0;
    g_mpu_data.gz_offset = 0;

    for (int i = 0; i < 300; i++)
    {
        mpu_read_bytes(MPU6500_ACCEL_XOUT_H, g_mpu_buff, 14);

        g_mpu_data.ax_offset += (int16_t)(g_mpu_buff[0] << 8 | g_mpu_buff[1]);
        g_mpu_data.ay_offset += (int16_t)(g_mpu_buff[2] << 8 | g_mpu_buff[3]);
        g_mpu_data.az_offset += (int16_t)(g_mpu_buff[4] << 8 | g_mpu_buff[5]);

        g_mpu_data.gx_offset += (int16_t)(g_mpu_buff[8] << 8 | g_mpu_buff[9]);
        g_mpu_data.gy_offset += (int16_t)(g_mpu_buff[10] << 8 | g_mpu_buff[11]);
        g_mpu_data.gz_offset += (int16_t)(g_mpu_buff[12] << 8 | g_mpu_buff[13]);

        MPU_DELAY(5);
    }

    g_mpu_data.ax_offset = (int16_t)(g_mpu_data.ax_offset / 300);
    g_mpu_data.ay_offset = (int16_t)(g_mpu_data.ay_offset / 300);
    g_mpu_data.az_offset = (int16_t)(g_mpu_data.az_offset / 300);
    g_mpu_data.gx_offset = (int16_t)(g_mpu_data.gx_offset / 300);
    g_mpu_data.gy_offset = (int16_t)(g_mpu_data.gy_offset / 300);
    g_mpu_data.gz_offset = (int16_t)(g_mpu_data.gz_offset / 300);
}

static uint8_t mpu_device_init(void)
{
    MPU_DELAY(100);

    const uint8_t init_data[][2] = {
        {MPU6500_PWR_MGMT_1, 0x80},
        {MPU6500_PWR_MGMT_1, 0x03},
        {MPU6500_PWR_MGMT_2, 0x00},
        {MPU6500_CONFIG, 0x04},
        {MPU6500_GYRO_CONFIG, 0x18},
        {MPU6500_ACCEL_CONFIG, 0x10},
        {MPU6500_ACCEL_CONFIG_2, 0x02},
        {MPU6500_USER_CTRL, 0x20},
    };

    for (size_t i = 0; i < (sizeof(init_data) / sizeof(init_data[0])); i++)
    {
        mpu_write_byte(init_data[i][0], init_data[i][1]);
        MPU_DELAY(1);
    }

    mpu_set_gyro_fsr(3);
    mpu_set_accel_fsr(2);

    ist8310_init();
    mpu_offset_call();
    return 0;
}

static void init_quaternion(void)
{
    int16_t hx = (int16_t)g_imu.mx;
    int16_t hy = (int16_t)g_imu.my;

#ifdef BOARD_DOWN
    if (hx < 0 && hy < 0)
    {
        if (fabs(hx / (float)hy) >= 1)
        {
            q0 = -0.005f;
            q1 = -0.199f;
            q2 = 0.979f;
            q3 = -0.0089f;
        }
        else
        {
            q0 = -0.008f;
            q1 = -0.555f;
            q2 = 0.83f;
            q3 = -0.002f;
        }
    }
    else if (hx < 0 && hy > 0)
    {
        if (fabs(hx / (float)hy) >= 1)
        {
            q0 = 0.005f;
            q1 = -0.199f;
            q2 = -0.978f;
            q3 = 0.012f;
        }
        else
        {
            q0 = 0.005f;
            q1 = -0.553f;
            q2 = -0.83f;
            q3 = -0.0023f;
        }
    }
    else if (hx > 0 && hy > 0)
    {
        if (fabs(hx / (float)hy) >= 1)
        {
            q0 = 0.0012f;
            q1 = -0.978f;
            q2 = -0.199f;
            q3 = -0.005f;
        }
        else
        {
            q0 = 0.0023f;
            q1 = -0.83f;
            q2 = -0.553f;
            q3 = 0.0023f;
        }
    }
    else if (hx > 0 && hy < 0)
    {
        if (fabs(hx / (float)hy) >= 1)
        {
            q0 = 0.0025f;
            q1 = 0.978f;
            q2 = -0.199f;
            q3 = 0.008f;
        }
        else
        {
            q0 = 0.0025f;
            q1 = 0.83f;
            q2 = -0.56f;
            q3 = 0.0045f;
        }
    }
#else
    if (hx < 0 && hy < 0)
    {
        if (fabs(hx / (float)hy) >= 1)
        {
            q0 = 0.195f;
            q1 = -0.015f;
            q2 = 0.0043f;
            q3 = 0.979f;
        }
        else
        {
            q0 = 0.555f;
            q1 = -0.015f;
            q2 = 0.006f;
            q3 = 0.829f;
        }
    }
    else if (hx < 0 && hy > 0)
    {
        if (fabs(hx / (float)hy) >= 1)
        {
            q0 = -0.193f;
            q1 = -0.009f;
            q2 = -0.006f;
            q3 = 0.979f;
        }
        else
        {
            q0 = -0.552f;
            q1 = -0.0048f;
            q2 = -0.0115f;
            q3 = 0.8313f;
        }
    }
    else if (hx > 0 && hy > 0)
    {
        if (fabs(hx / (float)hy) >= 1)
        {
            q0 = -0.9785f;
            q1 = 0.008f;
            q2 = -0.02f;
            q3 = 0.195f;
        }
        else
        {
            q0 = -0.9828f;
            q1 = 0.002f;
            q2 = -0.0167f;
            q3 = 0.5557f;
        }
    }
    else if (hx > 0 && hy < 0)
    {
        if (fabs(hx / (float)hy) >= 1)
        {
            q0 = -0.979f;
            q1 = 0.0116f;
            q2 = -0.0167f;
            q3 = -0.195f;
        }
        else
        {
            q0 = -0.83f;
            q1 = 0.014f;
            q2 = -0.012f;
            q3 = -0.556f;
        }
    }
#endif
}

static void imu_ahrs_update(void)
{
    float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez, halfT;
    float tempq0, tempq1, tempq2, tempq3;

    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    float gx = g_imu.wx;
    float gy = g_imu.wy;
    float gz = g_imu.wz;
    float ax = g_imu.ax;
    float ay = g_imu.ay;
    float az = g_imu.az;
    float mx = g_imu.mx;
    float my = g_imu.my;
    float mz = g_imu.mz;

    now_update = HAL_GetTick();
    halfT = ((float)(now_update - last_update) / 2000.0f);
    last_update = now_update;

    norm = inv_sqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

#ifdef IST8310
    norm = inv_sqrt(mx * mx + my * my + mz * mz);
    mx *= norm;
    my *= norm;
    mz *= norm;
#else
    mx = 0.0f;
    my = 0.0f;
    mz = 0.0f;
#endif

    hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3) + 2.0f * mz * (q1q3 + q0q2);
    hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3) + 2.0f * mz * (q2q3 - q0q1);
    hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
    bx = sqrtf((hx * hx) + (hy * hy));
    bz = hz;

    vx = 2.0f * (q1q3 - q0q2);
    vy = 2.0f * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
    wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
    wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);

    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

    if (ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
        exInt = exInt + ex * Ki * halfT;
        eyInt = eyInt + ey * Ki * halfT;
        ezInt = ezInt + ez * Ki * halfT;

        gx = gx + Kp * ex + exInt;
        gy = gy + Kp * ey + eyInt;
        gz = gz + Kp * ez + ezInt;
    }

    tempq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    tempq1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    tempq2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    tempq3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

    norm = inv_sqrt(tempq0 * tempq0 + tempq1 * tempq1 + tempq2 * tempq2 + tempq3 * tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;
}

static void imu_attitude_update(void)
{
    g_imu.yaw = -atan2f(2.0f * q1 * q2 + 2.0f * q0 * q3, -2.0f * q2 * q2 - 2.0f * q3 * q3 + 1.0f) * 57.3f;
    g_imu.pit = -asinf(-2.0f * q1 * q3 + 2.0f * q0 * q2) * 57.3f;
    g_imu.rol = atan2f(2.0f * q2 * q3 + 2.0f * q0 * q1, -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1.0f) * 57.3f;
}

uint8_t IMU_MPU6500_Init(void)
{
    uint8_t id = mpu_read_byte(MPU6500_WHO_AM_I);
    if (id != MPU6500_ID && id != 0x70)
    {
        g_status = IMU_MPU6500_Status_DISABLE;
        return 1;
    }

    mpu_device_init();
    mpu_get_data();
    init_quaternion();
    exInt = 0.0f;
    eyInt = 0.0f;
    ezInt = 0.0f;
    last_update = HAL_GetTick();
    g_status = IMU_MPU6500_Status_ENABLE;
    return 0;
}

void IMU_MPU6500_Update(void)
{
    if (g_status != IMU_MPU6500_Status_ENABLE)
    {
        return;
    }

    mpu_get_data();
    imu_ahrs_update();
    imu_attitude_update();
}

Enum_IMU_MPU6500_Status IMU_MPU6500_Get_Status(void)
{
    return g_status;
}

float IMU_MPU6500_Get_Omega_X(void)
{
    return g_imu.wx;
}

float IMU_MPU6500_Get_Omega_Y(void)
{
    return g_imu.wy;
}

float IMU_MPU6500_Get_Omega_Z(void)
{
    return g_imu.wz;
}

float IMU_MPU6500_Get_Angle_Roll(void)
{
    return g_imu.rol;
}

float IMU_MPU6500_Get_Angle_Pitch(void)
{
    return g_imu.pit;
}

float IMU_MPU6500_Get_Angle_Yaw(void)
{
    return g_imu.yaw;
}
