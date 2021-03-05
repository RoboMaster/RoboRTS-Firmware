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

#include "chassis.h"
#include "device.h"

#define LOG_TAG "chassis"
#include "log.h"

/**
  * @brief     chassis pid param initialize
  * @param[in]
  * @retval    error code
  */
int32_t chassis_pid_init(struct chassis *chassis, const char *name, struct pid_param param, enum device_can can)
{
    char motor_name[4][OBJECT_NAME_MAX_LEN] = {0};
    uint8_t name_len;

    int32_t err;

    name_len = strlen(name);

    for (int i = 0; i < 4; i++)
    {
        memcpy(&motor_name[i], name, name_len);
        chassis->motor[i].can_periph = can;
        chassis->motor[i].can_id = 0x201 + i;
        chassis->motor[i].init_offset_f = 1;

        pid_struct_init(&chassis->motor_pid[i], param.max_out, param.integral_limit, param.p, param.i, param.d);
    }

    chassis->mecanum.param.wheel_perimeter = PERIMETER;
    chassis->mecanum.param.wheeltrack = WHEELTRACK;
    chassis->mecanum.param.wheelbase = WHEELBASE;
    chassis->mecanum.param.rotate_x_offset = ROTATE_X_OFFSET;
    chassis->mecanum.param.rotate_y_offset = ROTATE_Y_OFFSET;

    memcpy(&motor_name[0][name_len], "_FR\0", 4);
    memcpy(&motor_name[1][name_len], "_FL\0", 4);
    memcpy(&motor_name[2][name_len], "_BL\0", 4);
    memcpy(&motor_name[3][name_len], "_BR\0", 4);

    for (int i = 0; i < 4; i++)
    {
        err = motor_register(&(chassis->motor[i]), motor_name[i]);
        if (err != E_OK)
        {
            goto end;
        }
    }

    return E_OK;
end:
    return err;
}

/**
  * @brief     pid calcualte
  * @param[in]
  * @retval    error code
  */
int32_t chassis_pid_calculate(struct chassis *chassis)
{
    float motor_out;
    struct motor_data *pdata;
    struct mecanum_motor_fdb wheel_fdb[4];

    static uint8_t init_f = 0;
    static float last_time, period;

    device_assert(chassis != NULL);

    period  = get_time_ms_us() - last_time;

    if (!init_f)
    {
        period = 0;
        last_time = get_time_ms_us();
        init_f = 1;
    }
    else
    {
        last_time = get_time_ms_us();

        chassis->mecanum.speed.vx += chassis->acc.ax / 1000.0f * period;
        chassis->mecanum.speed.vy += chassis->acc.ay / 1000.0f * period;
        chassis->mecanum.speed.vw += chassis->acc.wz / 1000.0f * period;
    }

    mecanum_calculate(&(chassis->mecanum));

    for (int i = 0; i < 4; i++)
    {
        pdata = motor_get_data(&(chassis->motor[i]));

        wheel_fdb[i].total_ecd = pdata->total_ecd;
        wheel_fdb[i].speed_rpm = pdata->speed_rpm;

        motor_out = pid_calculate(&chassis->motor_pid[i], chassis->motor[i].data.speed_rpm, chassis->mecanum.wheel_rpm[i]);

        motor_set_current(&chassis->motor[i], (int16_t)motor_out);
    }

    mecanum_position_measure(&(chassis->mecanum), wheel_fdb);

    return E_OK;
}

/**
  * @brief     update chassis gyro angle
  * @param[in]
  * @retval    error code
  */
int32_t chassis_gyro_updata(struct chassis *chassis, float yaw_angle, float yaw_rate)
{
    device_assert(chassis != NULL);

    chassis->mecanum.gyro.yaw_gyro_angle = yaw_angle;
    chassis->mecanum.gyro.yaw_gyro_rate = yaw_rate;
    return E_OK;
}

/**
  * @brief     set chassis vx vy vz, vx:forward, vy:left, vz:anticlockwise
  * @param[in]
  * @retval    error code
  */
int32_t chassis_set_speed(struct chassis *chassis, float vx, float vy, float vw)
{
    if (chassis == NULL)
    {
        return -E_INVAL;
    }
    chassis->mecanum.speed.vx = vx;
    chassis->mecanum.speed.vy = vy;
    chassis->mecanum.speed.vw = vw;
    return E_OK;
}

int32_t chassis_set_acc(struct chassis *chassis, float ax, float ay, float wz)
{
    if (chassis == NULL)
    {
        return -E_INVAL;
    }
    chassis->acc.ax = ax;
    chassis->acc.ay = ay;
    chassis->acc.wz = wz;
    return E_OK;
}

int32_t chassis_set_vw(struct chassis *chassis, float vw)
{
    if (chassis == NULL)
    {
        return -E_INVAL;
    }
    chassis->mecanum.speed.vw = vw;
    return E_OK;
}

int32_t chassis_set_vx_vy(struct chassis *chassis, float vx, float vy)
{
    if (chassis == NULL)
    {
        return -E_INVAL;
    }
    chassis->mecanum.speed.vx = vx;
    chassis->mecanum.speed.vy = vy;
    return E_OK;
}

int32_t chassis_set_offset(struct chassis *chassis, float offset_x, float offset_y)
{
    if (chassis == NULL)
    {
        return -E_INVAL;
    }

    chassis->mecanum.param.rotate_x_offset = offset_x;
    chassis->mecanum.param.rotate_y_offset = offset_y;

    return E_OK;
}

/**
  * @brief     measure chassis odometry
  * @param[in]
  * @retval    error code
  */
int32_t chassis_get_info(struct chassis *chassis, struct chassis_info *info)
{
    if (chassis == NULL)
    {
        return -E_INVAL;
    }

    memcpy(info, &(chassis->mecanum.position), sizeof(struct mecanum_position));
    ANGLE_LIMIT_180(info->angle_deg, chassis->mecanum.position.angle_deg);
    ANGLE_LIMIT_180(info->yaw_gyro_angle, chassis->mecanum.gyro.yaw_gyro_angle);
    info->yaw_gyro_rate = chassis->mecanum.gyro.yaw_gyro_rate;

    for (int i = 0; i < 4; i++)
    {
        info->wheel_rpm[i] = chassis->mecanum.wheel_rpm[i] * MOTOR_DECELE_RATIO;
    }

    return E_OK;
}

int32_t chassis_enable(struct chassis *chassis)
{
    if (chassis == NULL)
    {
        return -E_INVAL;
    }

    for (int i = 0; i < 4; i++)
    {
        chassis->motor_pid[i].enable = 1;
    }

    return E_OK;
}

int32_t chassis_disable(struct chassis *chassis)
{
    if (chassis == NULL)
    {
        return -E_INVAL;
    }

    for (int i = 0; i < 4; i++)
    {
        chassis->motor_pid[i].enable = 0;
    }

    return E_OK;
}

