/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
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
/** @file bsp_imu.c
 *  @version 1.0
 *  @date Apr 2017
 *
 *  @brief Configurate MPU6500 and Read the Accelerator
 *         and Gyrometer data using SPI interface
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "bsp_imu.h"
#include "imu_task.h"
#include "calibrate.h"
#include "ist8310_reg.h"
#include "mpu6500_reg.h"
#include "cmsis_os.h"
#include "spi.h"
#include "string.h"

#define MPU_INIT_DELAY(x) HAL_Delay(x)

#define MPU_HSPI hspi5
#define MPU_NSS_LOW  HAL_GPIO_WritePin(GPIOF, SPI5_NSS_Pin, GPIO_PIN_RESET)
#define MPU_NSS_HIGH HAL_GPIO_WritePin(GPIOF, SPI5_NSS_Pin, GPIO_PIN_SET)
#define ENABLE_IST   HAL_GPIO_WritePin(GPIOE, IST_SET_Pin, GPIO_PIN_SET)

static uint8_t tx, rx;
static uint8_t tx_buff[14];
static uint8_t mpu_buff[14];

uint8_t mpu_write_reg(uint8_t const reg, uint8_t const data)
{
  MPU_NSS_LOW;
  tx = reg & 0x7F;
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
  tx = data;
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
  MPU_NSS_HIGH;
  return 0;
}

uint8_t mpu_read_reg(uint8_t const reg)
{
  MPU_NSS_LOW;
  tx = reg | 0x80;
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
  MPU_NSS_HIGH;
  return rx;
}

uint8_t mpu_read_regs(uint8_t const regAddr, uint8_t *pData, uint8_t len)
{
  MPU_NSS_LOW;
  tx = regAddr | 0x80;
  tx_buff[0] = tx;
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
  HAL_SPI_TransmitReceive(&MPU_HSPI, tx_buff, pData, len, 55);
  MPU_NSS_HIGH;
  return 0;
}

static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data)
{
  //turn off slave 1 at first
  mpu_write_reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  MPU_INIT_DELAY(2);
  mpu_write_reg(MPU6500_I2C_SLV1_REG, addr);
  MPU_INIT_DELAY(2);
  mpu_write_reg(MPU6500_I2C_SLV1_DO, data);
  MPU_INIT_DELAY(2);
  //turn on slave 1 with one byte transmitting
  mpu_write_reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  MPU_INIT_DELAY(10);
}

static uint8_t ist_reg_read_by_mpu(uint8_t addr)
{
  uint8_t retval;
  mpu_write_reg(MPU6500_I2C_SLV4_REG, addr);
  MPU_INIT_DELAY(10);
  mpu_write_reg(MPU6500_I2C_SLV4_CTRL, 0x80);
  MPU_INIT_DELAY(10);
  retval = mpu_read_reg(MPU6500_I2C_SLV4_DI);
  //turn off slave4 after read
  mpu_write_reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  MPU_INIT_DELAY(10);
  return retval;
}

static void mpu_mst_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
  //configure the device address of the IST8310
  //use slave1,auto transmit single measure mode.
  mpu_write_reg(MPU6500_I2C_SLV1_ADDR, device_address);
  MPU_INIT_DELAY(2);
  mpu_write_reg(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
  MPU_INIT_DELAY(2);
  mpu_write_reg(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
  MPU_INIT_DELAY(2);

  //use slave0,auto read data
  mpu_write_reg(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
  MPU_INIT_DELAY(2);
  mpu_write_reg(MPU6500_I2C_SLV0_REG, reg_base_addr);
  MPU_INIT_DELAY(2);

  //every eight mpu6500 internal samples one i2c master read
  mpu_write_reg(MPU6500_I2C_SLV4_CTRL, 0x03);
  MPU_INIT_DELAY(2);
  //enable slave 0 and 1 access delay
  mpu_write_reg(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
  MPU_INIT_DELAY(2);
  //enable slave 1 auto transmit
  mpu_write_reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  MPU_INIT_DELAY(6); //Wait 6ms (minimum waiting time for 16 times internal average setup)
  //enable slave 0 with data_num bytes reading
  mpu_write_reg(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
  MPU_INIT_DELAY(2);
}

uint8_t ist8310_init(void)
{
  //Enable I2C master mode, Reset I2C Slave module
  mpu_write_reg(MPU6500_USER_CTRL, 0x30); 
  MPU_INIT_DELAY(10);
  //I2C master clock 400kHz
  mpu_write_reg(MPU6500_I2C_MST_CTRL, 0x0d);
  MPU_INIT_DELAY(10);

  //turn on slave 1 for ist write and slave 4 for ist read
  mpu_write_reg(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS); //write ist
  MPU_INIT_DELAY(10);
  mpu_write_reg(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS); //read ist
  MPU_INIT_DELAY(10);

  //reset ist8310
  ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
  MPU_INIT_DELAY(10);

  if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
      return 1;

  ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
  MPU_INIT_DELAY(10);

  //config as ready mode to access reg
  ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00); 
  if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
      return 2;
  MPU_INIT_DELAY(10);

  //normal state, no int
  ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00); 
  if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
      return 3;
  MPU_INIT_DELAY(10);

  //config  low noise mode, x,y,z axis 16 time 1 avg,
  ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24); //100100
  if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
      return 4;
  MPU_INIT_DELAY(10);

  //Set/Reset pulse duration setup, normal mode
  ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);
  if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
      return 5;
  MPU_INIT_DELAY(10);

  //turn off slave1 & slave 4
  mpu_write_reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  MPU_INIT_DELAY(10);
  mpu_write_reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  MPU_INIT_DELAY(10);

  //configure and turn on slave 0
  mpu_mst_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
  MPU_INIT_DELAY(100);
  return 0;
}

void ist8310_get_data(uint8_t* buff)
{
  mpu_read_regs(MPU6500_EXT_SENS_DATA_00, buff, 6);
}

void mpu_get_data(void)
{
  mpu_read_regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

  mpu_data.ax   = mpu_buff[0] << 8 | mpu_buff[1];
  mpu_data.ay   = mpu_buff[2] << 8 | mpu_buff[3];
  mpu_data.az   = mpu_buff[4] << 8 | mpu_buff[5];
  mpu_data.temp = mpu_buff[6] << 8 | mpu_buff[7];

  mpu_data.gx = ((mpu_buff[8] << 8 | mpu_buff[9])   - mpu_data.gx_offset);
  mpu_data.gy = ((mpu_buff[10] << 8 | mpu_buff[11]) - mpu_data.gy_offset);
  mpu_data.gz = ((mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset);

  //ist8310_get_data((uint8_t*)&mpu_data.mx);

  memcpy(&imu.ax, &mpu_data.ax, 6 * sizeof(int16_t));
  imu.temp = 21 + mpu_data.temp / 333.87f;
  imu.wx   = mpu_data.gx / 16.384f / 57.3f; //2000dps -> rad/s
  imu.wy   = mpu_data.gy / 16.384f / 57.3f; //2000dps -> rad/s
  imu.wz   = mpu_data.gz / 16.384f / 57.3f; //2000dps -> rad/s

  imu_cali_hook(CALI_GYRO, &mpu_data.gx);
  imu_cali_hook(CALI_ACC, &mpu_data.ax);
  imu_cali_hook(CALI_MAG, &mpu_data.mx);
}

uint8_t mpu_device_init(void)
{
  ENABLE_IST;
  // Reset the internal registers
  mpu_write_reg(MPU6500_PWR_MGMT_1, 0x80);
  MPU_INIT_DELAY(100);
  // Reset gyro/accel/temp digital signal path
  mpu_write_reg(MPU6500_SIGNAL_PATH_RESET, 0x07);
  MPU_INIT_DELAY(100);

  if (MPU6500_ID != mpu_read_reg(MPU6500_WHO_AM_I))
    return 1;

  uint8_t MPU6500_Init_Data[7][2] = {
    { MPU6500_PWR_MGMT_1,     0x03 }, // Auto selects Clock Source
    { MPU6500_PWR_MGMT_2,     0x00 }, // all enable
    { MPU6500_CONFIG,         0x04 }, // gyro bandwidth 184Hz 01
    { MPU6500_GYRO_CONFIG,    0x18 }, // +-2000dps
    { MPU6500_ACCEL_CONFIG,   0x10 }, // +-8G
    { MPU6500_ACCEL_CONFIG_2, 0x04 }, // acc bandwidth 20Hz
    { MPU6500_USER_CTRL,      0x20 }, // Enable the I2C Master I/F module
                                      // pins ES_DA and ES_SCL are isolated from 
                                      // pins SDA/SDI and SCL/SCLK.
  };
  uint8_t i = 0;
  for (i = 0; i < 7; i++)
  {
      mpu_write_reg(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
      MPU_INIT_DELAY(1);
  }
  
  ist8310_init();
  mpu_offset_cal();
  return 0;
}

void mpu_offset_cal(void)
{
  int i;
  for (i = 0; i < 300; i++)
  {
    mpu_read_regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

    mpu_data.ax_offset += mpu_buff[0] << 8 | mpu_buff[1];
    mpu_data.ay_offset += mpu_buff[2] << 8 | mpu_buff[3];
    mpu_data.az_offset += mpu_buff[4] << 8 | mpu_buff[5];

    mpu_data.gx_offset += mpu_buff[8] << 8 | mpu_buff[9];
    mpu_data.gy_offset += mpu_buff[10] << 8 | mpu_buff[11];
    mpu_data.gz_offset += mpu_buff[12] << 8 | mpu_buff[13];

    MPU_INIT_DELAY(5);
  }
  mpu_data.ax_offset=mpu_data.ax_offset / 300;
  mpu_data.ay_offset=mpu_data.ay_offset / 300;
  mpu_data.az_offset=mpu_data.az_offset / 300;
  mpu_data.gx_offset=mpu_data.gx_offset / 300;
  mpu_data.gy_offset=mpu_data.gx_offset / 300;
  mpu_data.gz_offset=mpu_data.gz_offset / 300;
}

