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

#include "gimbal.h"
#define LOG_TAG "gimbal"
#include "log.h"

static int32_t gimbal_set_yaw_gyro_angle(struct gimbal *gimbal, float yaw, uint8_t mode);
static int16_t gimbal_get_ecd_angle(int16_t raw_ecd, int16_t center_offset);

/**
  * @brief    gimbal cascade param init
  * @param[in]
  * @retval    error code
  */
int32_t gimbal_cascade_init(struct gimbal *gimbal, const char *name,
                            struct pid_param yaw_inter_param,
                            struct pid_param yaw_outer_param,
                            struct pid_param pitch_inter_param,
                            struct pid_param pitch_outer_param,
                            enum device_can can)
{
    char motor_name[2][OBJECT_NAME_MAX_LEN] = {0};
    uint8_t name_len;

    device_assert(gimbal != NULL);

    name_len = strlen(name);

    if (name_len > OBJECT_NAME_MAX_LEN / 2)
    {
        name_len = OBJECT_NAME_MAX_LEN / 2;
    }

    for (int i = 0; i < 2; i++)
    {
        memcpy(&motor_name[i], name, name_len);
    }
    gimbal->yaw_motor.can_periph = can;
    gimbal->yaw_motor.can_id = 0x205;
    gimbal->pitch_motor.can_periph = can;
    gimbal->pitch_motor.can_id = 0x206;

    memcpy(&motor_name[YAW_MOTOR_INDEX][name_len], "_YAW\0", 5);
    memcpy(&motor_name[PITCH_MOTOR_INDEX][name_len], "_PIT\0", 5);

    motor_register(&(gimbal->yaw_motor), motor_name[YAW_MOTOR_INDEX]);
    motor_register(&(gimbal->pitch_motor), motor_name[PITCH_MOTOR_INDEX]);

    memcpy(&motor_name[YAW_MOTOR_INDEX][name_len], "_CTL_Y\0", 7);
    memcpy(&motor_name[PITCH_MOTOR_INDEX][name_len], "_CTL_P\0", 7);

    gimbal->mode.bit.yaw_mode = ENCODER_MODE;
    pid_struct_init(&(gimbal->yaw_outer_pid), yaw_outer_param.max_out, yaw_outer_param.integral_limit, yaw_outer_param.p, yaw_outer_param.i, yaw_outer_param.d);
    pid_struct_init(&(gimbal->yaw_inter_pid), yaw_inter_param.max_out, yaw_inter_param.integral_limit, yaw_inter_param.p, yaw_inter_param.i, yaw_outer_param.d);

    gimbal->mode.bit.pitch_mode = ENCODER_MODE;
    pid_struct_init(&(gimbal->pitch_outer_pid), pitch_inter_param.max_out, pitch_outer_param.integral_limit, pitch_outer_param.p, pitch_outer_param.i, pitch_outer_param.d);
    pid_struct_init(&(gimbal->pitch_inter_pid), pitch_inter_param.max_out, pitch_inter_param.integral_limit, pitch_inter_param.p, pitch_inter_param.i, pitch_inter_param.d);

    return E_OK;
}

/**
  * @brief     set pitch angle relative angle
  * @param[in]
  * @retval    error code
  */
int32_t gimbal_set_pitch_delta(struct gimbal *gimbal, float pitch)
{
    device_assert(gimbal != NULL);

    if (gimbal->mode.bit.pitch_mode == GYRO_MODE)
    {
        gimbal_set_pitch_angle(gimbal, gimbal->gyro_target_angle.pitch + pitch);
    }
    else
    {
        gimbal_set_pitch_angle(gimbal, gimbal->ecd_target_angle.pitch + pitch);
    }

    return E_OK;
}

int32_t gimbal_set_yaw_delta(struct gimbal *gimbal, float yaw)
{
    device_assert(gimbal != NULL);

    if (gimbal->mode.bit.yaw_mode == GYRO_MODE)
    {
        gimbal_set_yaw_angle(gimbal, gimbal->gyro_target_angle.yaw + yaw, 0);
    }
    else
    {
        gimbal_set_yaw_angle(gimbal, gimbal->ecd_target_angle.yaw + yaw, 0);
    }

    return E_OK;
}

/**
  * @brief     set pitch turn speed
  * @param[in]
  * @retval    error code
  */
int32_t gimbal_set_pitch_speed(struct gimbal *gimbal, float pitch)
{
    device_assert(gimbal != NULL);

    if (gimbal->mode.bit.pitch_mode == GYRO_MODE)
    {
        gimbal_set_pitch_angle(gimbal, gimbal->sensor.gyro_angle.pitch + pitch);
    }
    else
    {
        gimbal_set_pitch_angle(gimbal, gimbal->ecd_angle.pitch + pitch);
    }

    return E_OK;
}

int32_t gimbal_set_yaw_speed(struct gimbal *gimbal, float yaw)
{
    device_assert(gimbal != NULL);

    if (gimbal->mode.bit.yaw_mode == GYRO_MODE)
    {
        gimbal_set_yaw_angle(gimbal, gimbal->sensor.gyro_angle.yaw + yaw, YAW_FASTEST);
    }
    else
    {
        gimbal_set_yaw_angle(gimbal, gimbal->ecd_angle.yaw + yaw, 0);
    }

    return E_OK;
}

int32_t gimbal_set_pitch_angle(struct gimbal *gimbal, float pitch)
{
    device_assert(gimbal != NULL);

    if (gimbal->mode.bit.pitch_mode == GYRO_MODE)
    {
        float center_offset;

        center_offset = gimbal->sensor.gyro_angle.pitch - gimbal->ecd_angle.pitch;

        VAL_LIMIT(pitch, PITCH_ANGLE_MIN + center_offset, PITCH_ANGLE_MAX + center_offset);
        gimbal->gyro_target_angle.pitch = pitch;
    }
    else
    {
        VAL_LIMIT(pitch, PITCH_ANGLE_MIN, PITCH_ANGLE_MAX);
        gimbal->ecd_target_angle.pitch = pitch;
    }

    return E_OK;
}

int32_t gimbal_set_yaw_angle(struct gimbal *gimbal, float yaw, uint8_t mode)
{
    device_assert(gimbal != NULL);

    if (gimbal->mode.bit.yaw_mode == GYRO_MODE)
    {
        gimbal_set_yaw_gyro_angle(gimbal, yaw, mode);
    }
    else
    {
        VAL_LIMIT(yaw, YAW_ANGLE_MIN, YAW_ANGLE_MAX);
        gimbal->ecd_target_angle.yaw = yaw;
    }

    return E_OK;
}

/**
  * @brief     set pitch mode, GYRO_MORE or ENCODER_MODE
  * @param[in]
  * @retval    error code
  */
int32_t gimbal_set_pitch_mode(struct gimbal *gimbal, uint8_t mode)
{
    device_assert(gimbal != NULL);

    if (mode != gimbal->mode.bit.pitch_mode)
    {
        gimbal->mode.bit.pitch_mode = mode;
        if (mode == GYRO_MODE)
        {
            gimbal_set_pitch_angle(gimbal, gimbal->sensor.gyro_angle.pitch);
        }
        else if (mode == ENCODER_MODE)
        {
            gimbal_set_pitch_angle(gimbal, gimbal->ecd_angle.pitch);
        }
    }

    return E_OK;
}

int32_t gimbal_set_yaw_mode(struct gimbal *gimbal, uint8_t mode)
{
    device_assert(gimbal != NULL);

    if (mode != gimbal->mode.bit.yaw_mode)
    {
        gimbal->mode.bit.yaw_mode = mode;
        if (mode == GYRO_MODE)
        {
            gimbal_set_yaw_angle(gimbal, gimbal->sensor.gyro_angle.yaw, YAW_FASTEST);
        }
        else if (mode == ENCODER_MODE)
        {
            gimbal_set_yaw_angle(gimbal, gimbal->ecd_angle.yaw, 0);
        }
    }

    return E_OK;
}

int32_t gimbal_set_offset(struct gimbal *gimbal, uint16_t yaw_ecd, uint16_t pitch_ecd)
{
    device_assert(gimbal != NULL);

    gimbal->param.yaw_ecd_center = yaw_ecd;
    gimbal->param.pitch_ecd_center = pitch_ecd;

    return E_OK;
}

int32_t gimbal_pitch_enable(struct gimbal *gimbal)
{
    device_assert(gimbal != NULL);

    gimbal->pitch_inter_pid.enable = 1;

    return E_OK;
}

int32_t gimbal_pitch_disable(struct gimbal *gimbal)
{
    device_assert(gimbal != NULL);

    gimbal->pitch_inter_pid.enable = 0;

    return E_OK;
}

int32_t gimbal_yaw_enable(struct gimbal *gimbal)
{
    device_assert(gimbal != NULL);

    gimbal->yaw_inter_pid.enable = 1;

    return E_OK;
}

int32_t gimbal_yaw_disable(struct gimbal *gimbal)
{
    device_assert(gimbal != NULL);

    gimbal->yaw_inter_pid.enable = 0;

    return E_OK;
}

/**
  * @brief     PID calculate
  * @param[in]
  * @retval    error code
  */
int32_t gimbal_cascade_calculate(struct gimbal *gimbal)
{
    float motor_out;
    float outer_out;
    struct motor_data *pdata;
    float yaw_fdb, pitch_fdb;

    device_assert(gimbal != NULL);

    pdata = motor_get_data(&(gimbal->yaw_motor));
    gimbal->ecd_angle.yaw = YAW_MOTOR_POSITIVE_DIR * gimbal_get_ecd_angle(pdata->ecd, gimbal->param.yaw_ecd_center) / ENCODER_ANGLE_RATIO;

    pdata = motor_get_data(&(gimbal->pitch_motor));
    gimbal->ecd_angle.pitch = PITCH_MOTOR_POSITIVE_DIR * gimbal_get_ecd_angle(pdata->ecd, gimbal->param.pitch_ecd_center) / ENCODER_ANGLE_RATIO;

    if (gimbal->mode.bit.yaw_mode == GYRO_MODE)
    {
        float center_offset;

        yaw_fdb = gimbal->sensor.gyro_angle.yaw;
        center_offset = gimbal->sensor.gyro_angle.yaw - gimbal->ecd_angle.yaw;

        VAL_LIMIT(gimbal->gyro_target_angle.yaw, YAW_ANGLE_MIN + center_offset, YAW_ANGLE_MAX + center_offset);

        outer_out = pid_calculate(&(gimbal->yaw_outer_pid), yaw_fdb, gimbal->gyro_target_angle.yaw);
    }
    else
    {
        yaw_fdb = gimbal->ecd_angle.yaw;
        VAL_LIMIT(gimbal->ecd_target_angle.yaw, YAW_ANGLE_MIN, YAW_ANGLE_MAX);

        outer_out = pid_calculate(&(gimbal->yaw_outer_pid), yaw_fdb, gimbal->ecd_target_angle.yaw);
    }

    motor_out = pid_calculate(&(gimbal->yaw_inter_pid), gimbal->sensor.rate.yaw_rate, outer_out);
    motor_set_current(&(gimbal->yaw_motor), (int16_t)YAW_MOTOR_POSITIVE_DIR * motor_out);

    if (gimbal->mode.bit.pitch_mode == GYRO_MODE)
    {
        float center_offset;

        pitch_fdb = gimbal->sensor.gyro_angle.pitch;
        center_offset = gimbal->sensor.gyro_angle.pitch - gimbal->ecd_angle.pitch;

        VAL_LIMIT(gimbal->gyro_target_angle.pitch, PITCH_ANGLE_MIN + center_offset, PITCH_ANGLE_MAX + center_offset);

        outer_out = pid_calculate(&(gimbal->pitch_outer_pid), pitch_fdb, gimbal->gyro_target_angle.pitch);
    }
    else
    {
        pitch_fdb = gimbal->ecd_angle.pitch;
        VAL_LIMIT(gimbal->ecd_target_angle.pitch, PITCH_ANGLE_MIN, PITCH_ANGLE_MAX);

        outer_out = pid_calculate(&(gimbal->pitch_outer_pid), pitch_fdb, gimbal->ecd_target_angle.pitch);
    }

    motor_out = pid_calculate(&(gimbal->pitch_inter_pid), gimbal->sensor.rate.pitch_rate, outer_out);
    motor_set_current(&(gimbal->pitch_motor), (int16_t)PITCH_MOTOR_POSITIVE_DIR * motor_out);

    return E_OK;
}

/**
  * @brief     update gyro rate
  * @param[in]
  * @retval    error code
  */
int32_t gimbal_rate_update(struct gimbal *gimbal, float yaw_rate, float pitch_rate)
{
    device_assert(gimbal != NULL);

    gimbal->sensor.rate.yaw_rate = yaw_rate;
    gimbal->sensor.rate.pitch_rate = pitch_rate;

    return E_OK;
}

/**
  * @brief     update gyro angle
  * @param[in]
  * @retval    error code
  */
int32_t gimbal_yaw_gyro_update(struct gimbal *gimbal, float yaw)
{
    device_assert(gimbal != NULL);

    gimbal->sensor.gyro_angle.yaw = yaw;

    return E_OK;
}

int32_t gimbal_pitch_gyro_update(struct gimbal *gimbal, float pitch)
{
    device_assert(gimbal != NULL);

    gimbal->sensor.gyro_angle.pitch = pitch;

    return E_OK;
}

/**
  * @brief     push gimbal infomation to PC
  * @param[in]
  * @retval    error code
  */
int32_t gimbal_get_info(struct gimbal *gimbal, struct gimbal_info *info)
{
    device_assert(gimbal != NULL);

    info->yaw_ecd_angle = gimbal->ecd_angle.yaw;
    info->pitch_ecd_angle = gimbal->ecd_angle.pitch;

    ANGLE_LIMIT_180(info->yaw_gyro_angle, gimbal->sensor.gyro_angle.yaw);
    info->mode = gimbal->mode.state;
    info->pitch_gyro_angle = gimbal->sensor.gyro_angle.pitch;
    info->yaw_rate = gimbal->sensor.rate.yaw_rate;
    info->pitch_rate = gimbal->sensor.rate.pitch_rate;

    return E_OK;
}

static int16_t gimbal_get_ecd_angle(int16_t raw_ecd, int16_t center_offset)
{
    int16_t tmp = 0;
    if (center_offset >= 4096)
    {
        if (raw_ecd > center_offset - 4096)
        {
            tmp = raw_ecd - center_offset;
        }
        else
        {
            tmp = raw_ecd + 8192 - center_offset;
        }
    }
    else
    {
        if (raw_ecd > center_offset + 4096)
        {
            tmp = raw_ecd - 8192 - center_offset;
        }
        else
        {
            tmp = raw_ecd - center_offset;
        }
    }
    return tmp;
}

/**
  * @brief     yaw range 0~360
  * @param[in]
  * @retval    error code
  */
static int32_t gimbal_set_yaw_gyro_angle(struct gimbal *gimbal, float yaw, uint8_t mode)
{
    device_assert(gimbal != NULL);

    float yaw_offset = 0, yaw_now = 0, yaw_target = 0;

    ANGLE_LIMIT_360(yaw_target, yaw);
    ANGLE_LIMIT_360(yaw_now, gimbal->sensor.gyro_angle.yaw);

    if (mode == YAW_CLOCKWISE)
    {
        if (yaw_now < yaw_target)
        {
            yaw_offset = -(360 - (yaw_target - yaw_now));
        }
        else
        {
            yaw_offset = yaw_target - yaw_now;
        }
    }
    else if (mode == YAW_ANTICLOCKWISE)
    {
        if (yaw_now < yaw_target)
        {
            yaw_offset = yaw_target - yaw_now;
        }
        else
        {
            yaw_offset = 360 + (yaw_target - yaw_now);
        }
    }
    else if (mode == YAW_FASTEST)
    {
        yaw_offset = yaw_target - yaw_now;
        if (yaw_offset > 180)
        {
            yaw_offset = yaw_offset - 360;
        }
        else if (yaw_offset < -180)
        {
            yaw_offset = yaw_offset + 360;
        }
    }

    gimbal->gyro_target_angle.yaw = gimbal->sensor.gyro_angle.yaw + yaw_offset;

    return E_OK;
}
