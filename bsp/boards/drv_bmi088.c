/****************************************************************************
 *  Copyright (C) 2020 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "main.h"
#include "spi.h"
#include "sys.h"
#include "drv_imu.h"
#include "drv_io.h"
#include "BMI088driver.h"
#include "ahrs_lib.h"
#include "pid.h"
#include "os_timer.h"
#include "easyflash.h"

#define ABS_F(x) (x) < 0 ? -(x) : (x)

#define BMI088_BOARD_INSTALL_SPIN_MATRIX \
  {0.0f, 1.0f, 0.0f},                    \
      {-1.0f, 0.0f, 0.0f},               \
  {                                      \
    0.0f, 0.0f, 1.0f                     \
  }

bmi088_real_data_t bmi088_real_data;
fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};

fp32 gyro_offset[3];
fp32 gyro_cali_offset[3];

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];
fp32 accel_cali_offset[3];

fp32 temperate = 0;

fp32 gyro[3], accel[3], mag[3];
static fp32 ins_quat[4];
fp32 ins_angle[3];

static void bmi088_cali_slove(fp32 gyro[3], fp32 accel[3], bmi088_real_data_t *bmi088);

void bmi088_get_data(struct ahrs_sensor *sensor)
{

    BMI088_Read(bmi088_real_data.gyro, bmi088_real_data.accel, &temperate);

    bmi088_cali_slove(gyro, accel, &bmi088_real_data);

    sensor->ax = accel[0];
    sensor->ay = accel[1];
    sensor->az = accel[2];

    sensor->gx = gyro[0];
    sensor->gy = gyro[1];
    sensor->gz = gyro[2];
}

void bmi088_get_temp(float *tmp)
{
    *tmp = temperate;
}

/**
  * @brief  bmi088 init
  * @param
  * @retval error code
  */
uint8_t bmi088_device_init(void)
{
    BMI088_Init();

    BMI088_Read(bmi088_real_data.gyro, bmi088_real_data.accel, &temperate);

    bmi088_cali_slove(gyro, accel, &bmi088_real_data);

    AHRS_init(ins_quat, accel, mag);

    get_angle(ins_quat, ins_angle, ins_angle + 1, ins_angle + 2);

    return 0;
}

int ahrs_update(struct ahrs_sensor *sensor, uint8_t period_ms)
{
    BMI088_Read(bmi088_real_data.gyro, bmi088_real_data.accel, &temperate);
    bmi088_cali_slove(gyro, accel, &bmi088_real_data);

    sensor->ax = accel[0];
    sensor->ay = accel[1];
    sensor->az = accel[2];

    sensor->gx = gyro[0];
    sensor->gy = gyro[1];
    sensor->gz = gyro[2];

    AHRS_update(ins_quat, period_ms / 1000.0f, gyro, accel, mag);
    get_angle(ins_quat, ins_angle, ins_angle + 1, ins_angle + 2);
    sensor->yaw = ins_angle[0];
    sensor->pitch = ins_angle[1];
    sensor->roll = ins_angle[2];

    return 0;
}

/* temperature control, using pid */
struct pid pid_imu_tmp;

int32_t imu_temp_keep(void *argc)
{
    float temp;
    bmi088_get_temp(&temp);
    pid_calculate(&pid_imu_tmp, temp, DEFAULT_IMU_TEMP);
    if (pid_imu_tmp.out < 0)
    {
        pid_imu_tmp.out = 0;
    }
    mpu_heat_output(pid_imu_tmp.out);
    return 0;
}

void imu_temp_ctrl_init(void)
{
    pid_struct_init(&pid_imu_tmp, 20000, 8000, 800, 10, 0);
    soft_timer_register(imu_temp_keep, (void *)NULL, 5);
}

/**
  * @brief  bmi088 get gyrp offset
  * @param
  * @retval error code
  */
uint8_t bmi088_set_offset(void)
{
    imu_temp_ctrl_init();
    /* need adjust */
    while (ABS_F(DEFAULT_IMU_TEMP - temperate) > 1.0f)
    {
        BMI088_Read(bmi088_real_data.gyro, bmi088_real_data.accel, &temperate);

        imu_temp_keep(NULL);
        HAL_Delay(2);

        LED_R_ON();
        LED_B_ON();
        LED_G_ON();
    }

    fp32 gyro[3], accel[3];

    for (int i = 0; i < 300; i++)
    {
        BMI088_Read(gyro, accel, &temperate);

        gyro_offset[0] += gyro[0];
        gyro_offset[1] += gyro[1];
        gyro_offset[2] += gyro[2];

        imu_temp_keep(NULL);
        HAL_Delay(2);
    }

    LED_R_ON();
    LED_B_ON();
    LED_G_ON();

    gyro_offset[0] = gyro_offset[0] / 300;
    gyro_offset[1] = gyro_offset[1] / 300;
    gyro_offset[2] = gyro_offset[2] / 300;

    ef_set_env_blob(BMI088_PARAM_KEY, gyro_offset, sizeof(gyro_offset));

    __disable_irq();
    NVIC_SystemReset();

    return 0;
}

uint8_t bmi088_get_offset(void)
{
    size_t read_len = 0;
    ef_get_env_blob(BMI088_PARAM_KEY, gyro_offset, sizeof(gyro_offset), &read_len);

    if (read_len == sizeof(gyro_offset))
    {
        /* read ok */
        return 0;
    }
    else
    {
        bmi088_set_offset();
    }

    return 0;
}

static void bmi088_cali_slove(fp32 gyro[3], fp32 accel[3], bmi088_real_data_t *bmi088)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] - gyro_offset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] - accel_offset[i];
    }
}
