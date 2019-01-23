/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
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

#include "ist8310_reg.h"
#include "mpu6500_reg.h"
#include "main.h"
#include "spi.h"
#include "sys.h"
#include "drv_imu.h"

#define MPU_DELAY(x) HAL_Delay(x)

#define MPU_HSPI hspi5
#define MPU_NSS_LOW() HAL_GPIO_WritePin(GPIOF, SPI5_NSS_Pin, GPIO_PIN_RESET)
#define MPU_NSS_HIGH() HAL_GPIO_WritePin(GPIOF, SPI5_NSS_Pin, GPIO_PIN_SET)
#define IST_ENABLE() HAL_GPIO_WritePin(GPIOE, IST_RESET_Pin, GPIO_PIN_SET)

static uint8_t tx, rx;
static uint8_t tx_buff[14];
static uint8_t mpu_buff[14];

static struct mpu_data_info mpu_data;
static struct mpu_calibrate imu_cali = {1, 0, 0};

static void get_mpu_gyro_offset(void);
static void get_mpu_acc_offset(void);
static void get_ist_mag_offset(void);

uint8_t mpu_write_reg(uint8_t const reg, uint8_t const data)
{
  MPU_NSS_LOW();
  tx = reg & 0x7F;
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
  tx = data;
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
  MPU_NSS_HIGH();
  return 0;
}

uint8_t mpu_read_reg(uint8_t const reg)
{
  MPU_NSS_LOW();
  tx = reg | 0x80;
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
  MPU_NSS_HIGH();
  return rx;
}

uint8_t mpu_read_regs(uint8_t const regAddr, uint8_t *pData, uint8_t len)
{
  MPU_NSS_LOW();
  tx = regAddr | 0x80;
  tx_buff[0] = tx;
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
  HAL_SPI_TransmitReceive(&MPU_HSPI, tx_buff, pData, len, 55);
  MPU_NSS_HIGH();
  return 0;
}

static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data)
{
  //turn off slave 1 at first
  mpu_write_reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  MPU_DELAY(2);
  mpu_write_reg(MPU6500_I2C_SLV1_REG, addr);
  MPU_DELAY(2);
  mpu_write_reg(MPU6500_I2C_SLV1_DO, data);
  MPU_DELAY(2);
  //turn on slave 1 with one byte transmitting
  mpu_write_reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  MPU_DELAY(10);
}

static uint8_t ist_reg_read_by_mpu(uint8_t addr)
{
  uint8_t retval;
  mpu_write_reg(MPU6500_I2C_SLV4_REG, addr);
  MPU_DELAY(10);
  mpu_write_reg(MPU6500_I2C_SLV4_CTRL, 0x80);
  MPU_DELAY(10);
  retval = mpu_read_reg(MPU6500_I2C_SLV4_DI);
  //turn off slave4 after read
  mpu_write_reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  MPU_DELAY(10);
  return retval;
}

static void mpu_mst_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
  //configure the device address of the IST8310
  //use slave1,auto transmit single measure mode.
  mpu_write_reg(MPU6500_I2C_SLV1_ADDR, device_address);
  MPU_DELAY(2);
  mpu_write_reg(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
  MPU_DELAY(2);
  mpu_write_reg(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
  MPU_DELAY(2);

  //use slave0,auto read data
  mpu_write_reg(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
  MPU_DELAY(2);
  mpu_write_reg(MPU6500_I2C_SLV0_REG, reg_base_addr);
  MPU_DELAY(2);

  //every eight mpu6500 internal samples one i2c master read
  mpu_write_reg(MPU6500_I2C_SLV4_CTRL, 0x03);
  MPU_DELAY(2);
  //enable slave 0 and 1 access delay
  mpu_write_reg(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
  MPU_DELAY(2);
  //enable slave 1 auto transmit
  mpu_write_reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  MPU_DELAY(6); //Wait 6ms (minimum waiting time for 16 times internal average setup)
  //enable slave 0 with data_num bytes reading
  mpu_write_reg(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
  MPU_DELAY(2);
}

uint8_t ist8310_init(void)
{
  //Enable I2C master mode, Reset I2C Slave module
  mpu_write_reg(MPU6500_USER_CTRL, 0x30);
  MPU_DELAY(10);
  //I2C master clock 400kHz
  mpu_write_reg(MPU6500_I2C_MST_CTRL, 0x0d);
  MPU_DELAY(10);

  //turn on slave 1 for ist write and slave 4 for ist read
  mpu_write_reg(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS); //write ist
  MPU_DELAY(10);
  mpu_write_reg(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS); //read ist
  MPU_DELAY(10);

  //reset ist8310
  ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
  MPU_DELAY(10);

  if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
    return 1;

  ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
  MPU_DELAY(10);

  //config as ready mode to access reg
  ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00);
  if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
    return 2;
  MPU_DELAY(10);

  //normal state, no int
  ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00);
  if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
    return 3;
  MPU_DELAY(10);

  //config  low noise mode, x,y,z axis 16 time 1 avg,
  ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24); //100100
  if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
    return 4;
  MPU_DELAY(10);

  //Set/Reset pulse duration setup, normal mode
  ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);
  if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
    return 5;
  MPU_DELAY(10);

  //turn off slave1 & slave 4
  mpu_write_reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  MPU_DELAY(10);
  mpu_write_reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  MPU_DELAY(10);

  //configure and turn on slave 0
  mpu_mst_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
  MPU_DELAY(100);
  return 0;
}

void ist8310_get_data(uint8_t *buff)
{
  mpu_read_regs(MPU6500_EXT_SENS_DATA_00, buff, 6);
}

//this function takes 24.6us.(42M spi)
void mpu_get_data(struct ahrs_sensor *sensor)
{
  MPU_IO_PROBE();

  mpu_read_regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

  mpu_data.ax = (mpu_buff[0] << 8 | mpu_buff[1]) - mpu_data.ax_offset;
  mpu_data.ay = (mpu_buff[2] << 8 | mpu_buff[3]) - mpu_data.ay_offset;
  mpu_data.az = (mpu_buff[4] << 8 | mpu_buff[5]) - mpu_data.az_offset;
  mpu_data.temp = mpu_buff[6] << 8 | mpu_buff[7];

  mpu_data.gx = ((mpu_buff[8] << 8 | mpu_buff[9]) - mpu_data.gx_offset);
  mpu_data.gy = ((mpu_buff[10] << 8 | mpu_buff[11]) - mpu_data.gy_offset);
  mpu_data.gz = ((mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset);

  ist8310_get_data((uint8_t *)&mpu_data.mx);

  sensor->ax = mpu_data.ax / (4096.0f / 9.80665f); //8g -> m/s^2
  sensor->ay = mpu_data.ay / (4096.0f / 9.80665f); //8g -> m/s^2
  sensor->az = mpu_data.az / (4096.0f / 9.80665f); //8g -> m/s^2

  sensor->wx = mpu_data.gx / 16.384f / 57.3f; //2000dps -> rad/s
  sensor->wy = mpu_data.gy / 16.384f / 57.3f; //2000dps -> rad/s
  sensor->wz = mpu_data.gz / 16.384f / 57.3f; //2000dps -> rad/s

  sensor->mx = (mpu_data.my - mpu_data.my_offset);
  sensor->my = -(mpu_data.mx - mpu_data.mx_offset);
  sensor->mz = -(mpu_data.mz - mpu_data.mz_offset);

  MPU_IO_PROBE();
}

void mpu_get_temp(float *tmp)
{
  *tmp = 21 + mpu_data.temp / 333.87f;;
}

uint8_t mpu_device_init(void)
{
  // Reset the internal registers
  IST_ENABLE();

  mpu_write_reg(MPU6500_PWR_MGMT_1, 0x80);
  MPU_DELAY(100);
  // Reset gyro/accel/temp digital signal path
  mpu_write_reg(MPU6500_SIGNAL_PATH_RESET, 0x07);
  MPU_DELAY(100);

  if (MPU6500_ID != mpu_read_reg(MPU6500_WHO_AM_I))
    return 1;
  //0: 250hz; 1: 184hz; 2: 92hz; 3: 41hz; 4: 20hz; 5: 10hz; 6: 5hz; 7: 3600hz
  uint8_t MPU6500_Init_Data[7][2] = {
      {MPU6500_PWR_MGMT_1, 0x03},     // Auto selects Clock Source
      {MPU6500_PWR_MGMT_2, 0x00},     // all enable
      {MPU6500_CONFIG, 0x02},         // gyro bandwidth 0x00:250Hz 0x04:20Hz
      {MPU6500_GYRO_CONFIG, 0x18},    // gyro range 0x10:+-1000dps 0x18:+-2000dps
      {MPU6500_ACCEL_CONFIG, 0x10},   // acc range 0x10:+-8G
      {MPU6500_ACCEL_CONFIG_2, 0x00}, // acc bandwidth 0x00:250Hz 0x04:20Hz
      {MPU6500_USER_CTRL, 0x20},      // Enable the I2C Master I/F module
                                      // pins ES_DA and ES_SCL are isolated from
                                      // pins SDA/SDI and SCL/SCLK.
  };

  for (int i = 0; i < 7; i++)
  {
    mpu_write_reg(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
    MPU_DELAY(1);
  }

  ist8310_init();

  if (imu_cali.gyro_flag == 1)
  {
    get_mpu_gyro_offset();
  }

  if (imu_cali.acc_flag == 1)
  {
    get_mpu_acc_offset();
  }

  if (imu_cali.mag_flag == 1)
  {
    get_ist_mag_offset();
  }

  return 0;
}

static void get_mpu_gyro_offset(void)
{
  int i;
  for (i = 0; i < 300; i++)
  {
    mpu_read_regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

    mpu_data.gx_offset += mpu_buff[8] << 8 | mpu_buff[9];
    mpu_data.gy_offset += mpu_buff[10] << 8 | mpu_buff[11];
    mpu_data.gz_offset += mpu_buff[12] << 8 | mpu_buff[13];

    MPU_DELAY(2);
  }

  mpu_data.gx_offset = mpu_data.gx_offset / 300;
  mpu_data.gy_offset = mpu_data.gy_offset / 300;
  mpu_data.gz_offset = mpu_data.gz_offset / 300;
  imu_cali.gyro_flag = 0;
}

static void get_mpu_acc_offset(void)
{
  int i;
  for (i = 0; i < 300; i++)
  {
    mpu_read_regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

    mpu_data.ax_offset += mpu_buff[0] << 8 | mpu_buff[1];
    mpu_data.ay_offset += mpu_buff[2] << 8 | mpu_buff[3];
    mpu_data.az_offset += mpu_buff[4] << 8 | mpu_buff[5] - 4096;

    MPU_DELAY(2);
  }

  mpu_data.ax_offset = mpu_data.ax_offset / 300;
  mpu_data.ay_offset = mpu_data.ay_offset / 300;
  mpu_data.az_offset = mpu_data.az_offset / 300;

  imu_cali.acc_flag = 0;
}

static void get_ist_mag_offset(void)
{
  int16_t mag_max[3], mag_min[3];
  int i;
  for (i = 0; i < 5000; i++)
  {
    ist8310_get_data((uint8_t *)&mpu_data.mx);
    if ((abs(mpu_data.mx) < 400) && (abs(mpu_data.my) < 400) && (abs(mpu_data.mz) < 400))
    {
      mag_max[0] = VAL_MAX(mag_max[0], mpu_data.mx);
      mag_min[0] = VAL_MIN(mag_min[0], mpu_data.mx);

      mag_max[1] = VAL_MAX(mag_max[1], mpu_data.my);
      mag_min[1] = VAL_MIN(mag_min[1], mpu_data.my);

      mag_max[2] = VAL_MAX(mag_max[2], mpu_data.mz);
      mag_min[2] = VAL_MIN(mag_min[2], mpu_data.mz);
    }
    MPU_DELAY(2);
  }
  mpu_data.mx_offset = (int16_t)((mag_max[0] + mag_min[0]) * 0.5f);
  mpu_data.my_offset = (int16_t)((mag_max[1] + mag_min[1]) * 0.5f);
  mpu_data.mz_offset = (int16_t)((mag_max[2] + mag_min[2]) * 0.5f);

  imu_cali.mag_flag = 0;
}
