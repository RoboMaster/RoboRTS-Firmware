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

#include "gimbal.h"

static int32_t yaw_gyro_input_convert(struct controller *ctrl, void *input);
static int32_t pitch_gyro_input_convert(struct controller *ctrl, void *input);
static int32_t yaw_ecd_input_convert(struct controller *ctrl, void *input);
static int32_t pitch_ecd_input_convert(struct controller *ctrl, void *input);
static int32_t gimbal_set_yaw_gyro_angle(struct gimbal *gimbal, float yaw, uint8_t mode);
static int16_t gimbal_get_ecd_angle(int16_t raw_ecd, int16_t center_offset);

int32_t gimbal_cascade_register(struct gimbal *gimbal, const char *name, enum device_can can)
{
  char motor_name[2][OBJECT_NAME_MAX_LEN] = {0};
  uint8_t name_len;
  int32_t err;

  if (gimbal == NULL)
    return -RM_INVAL;
  if (gimbal_find(name) != NULL)
    return -RM_EXISTED;

  object_init(&(gimbal->parent), Object_Class_Gimbal, name);

  name_len = strlen(name);

  if (name_len > OBJECT_NAME_MAX_LEN / 2)
  {
    name_len = OBJECT_NAME_MAX_LEN / 2;
  }

  for (int i = 0; i < 2; i++)
  {
    memcpy(&motor_name[i], name, name_len);
    gimbal->motor[i].can_periph = can;
    gimbal->motor[i].can_id = 0x205 + i;
  }

  memcpy(&motor_name[YAW_MOTOR_INDEX][name_len], "_YAW\0", 5);
  memcpy(&motor_name[PITCH_MOTOR_INDEX][name_len], "_PIT\0", 5);

  for (int i = 0; i < 2; i++)
  {
    err = motor_device_register(&(gimbal->motor[i]), motor_name[i], 0);
    if (err != RM_OK)
      goto end;
  }

  memcpy(&motor_name[YAW_MOTOR_INDEX][name_len], "_CTL_Y\0", 7);
  memcpy(&motor_name[PITCH_MOTOR_INDEX][name_len], "_CTL_P\0", 7);

  gimbal->mode.bit.yaw_mode = ENCODER_MODE;
  gimbal->ctrl[YAW_MOTOR_INDEX].convert_feedback = yaw_ecd_input_convert;
  pid_struct_init(&(gimbal->cascade[YAW_MOTOR_INDEX].outer), 2000, 0, 50, 0, 0);
  pid_struct_init(&(gimbal->cascade[YAW_MOTOR_INDEX].inter), 30000, 3000, 70, 0.2, 0);

  gimbal->mode.bit.pitch_mode = ENCODER_MODE;
  gimbal->ctrl[PITCH_MOTOR_INDEX].convert_feedback = pitch_ecd_input_convert;
  pid_struct_init(&(gimbal->cascade[PITCH_MOTOR_INDEX].outer), 2000, 0, 30, 0, 0);
  pid_struct_init(&(gimbal->cascade[PITCH_MOTOR_INDEX].inter), 30000, 3000, 60, 0.2, 0);

  for (int i = 0; i < 2; i++)
  {
    err = cascade_controller_register(&(gimbal->ctrl[i]), motor_name[i],
                                      &(gimbal->cascade[i]),
                                      &(gimbal->cascade_fdb[i]), 1);
    if (err != RM_OK)
      goto end;
  }
  
  return RM_OK;
end:
  object_detach(&(gimbal->parent));

  return err;
}

int32_t gimbal_set_pitch_delta(struct gimbal *gimbal, float pitch)
{
  if (gimbal == NULL)
    return -RM_INVAL;
  if (gimbal->mode.bit.pitch_mode == GYRO_MODE)
  {
    gimbal_set_pitch_angle(gimbal, gimbal->gyro_target_angle.pitch + pitch);
  }
  else
  {
    gimbal_set_pitch_angle(gimbal, gimbal->ecd_target_angle.pitch + pitch);
  }

  return RM_OK;
}

int32_t gimbal_set_yaw_delta(struct gimbal *gimbal, float yaw)
{
  if (gimbal == NULL)
    return -RM_INVAL;

  if (gimbal->mode.bit.yaw_mode == GYRO_MODE)
  {
    gimbal_set_yaw_angle(gimbal, gimbal->gyro_target_angle.yaw + yaw, 0);
  }
  else
  {
    gimbal_set_yaw_angle(gimbal, gimbal->ecd_target_angle.yaw + yaw, 0);
  }

  return RM_OK;
}

int32_t gimbal_set_pitch_speed(struct gimbal *gimbal, float pitch)
{
  if (gimbal == NULL)
    return -RM_INVAL;

  if (gimbal->mode.bit.pitch_mode == GYRO_MODE)
  {
    gimbal_set_pitch_angle(gimbal, gimbal->sensor.gyro_angle.pitch + pitch);
  }
  else
  {
    gimbal_set_pitch_angle(gimbal, gimbal->ecd_angle.pitch + pitch);
  }

  return RM_OK;
}

int32_t gimbal_set_yaw_speed(struct gimbal *gimbal, float yaw)
{
  if (gimbal == NULL)
    return -RM_INVAL;

  if (gimbal->mode.bit.yaw_mode == GYRO_MODE)
  {
    gimbal_set_yaw_angle(gimbal, gimbal->sensor.gyro_angle.yaw + yaw, YAW_FASTEST);
  }
  else
  {
    gimbal_set_yaw_angle(gimbal, gimbal->ecd_angle.yaw + yaw, 0);
  }

  return RM_OK;
}

int32_t gimbal_set_pitch_angle(struct gimbal *gimbal, float pitch)
{
  if (gimbal == NULL)
    return -RM_INVAL;

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

  return RM_OK;
}

int32_t gimbal_set_yaw_angle(struct gimbal *gimbal, float yaw, uint8_t mode)
{
  if (gimbal == NULL)
    return -RM_INVAL;

  if (gimbal->mode.bit.yaw_mode == GYRO_MODE)
  {
    gimbal_set_yaw_gyro_angle(gimbal, yaw, mode);
  }
  else
  {
    VAL_LIMIT(yaw, YAW_ANGLE_MIN, YAW_ANGLE_MAX);
    gimbal->ecd_target_angle.yaw = yaw;
  }

  return RM_OK;
}

int32_t gimbal_set_pitch_mode(struct gimbal *gimbal, uint8_t mode)
{
  if (gimbal == NULL)
    return -RM_INVAL;

  if (mode != gimbal->mode.bit.pitch_mode)
  {
    gimbal->mode.bit.pitch_mode = mode;
    if (mode == GYRO_MODE)
    {
      gimbal->ctrl[PITCH_MOTOR_INDEX].convert_feedback = pitch_gyro_input_convert;
      gimbal_set_pitch_angle(gimbal, gimbal->sensor.gyro_angle.pitch);
    }
    else if (mode == ENCODER_MODE)
    {
      gimbal->ctrl[PITCH_MOTOR_INDEX].convert_feedback = pitch_ecd_input_convert;
      gimbal_set_pitch_angle(gimbal, gimbal->ecd_angle.pitch);
    }
  }

  return RM_OK;
}

int32_t gimbal_set_yaw_mode(struct gimbal *gimbal, uint8_t mode)
{
  if (gimbal == NULL)
    return -RM_INVAL;

  if (mode != gimbal->mode.bit.yaw_mode)
  {
    gimbal->mode.bit.yaw_mode = mode;
    if (mode == GYRO_MODE)
    {
      gimbal->ctrl[YAW_MOTOR_INDEX].convert_feedback = yaw_gyro_input_convert;
      gimbal_set_yaw_angle(gimbal, gimbal->sensor.gyro_angle.yaw, YAW_FASTEST);
    }
    else if (mode == ENCODER_MODE)
    {
      gimbal->ctrl[YAW_MOTOR_INDEX].convert_feedback = yaw_ecd_input_convert;
      gimbal_set_yaw_angle(gimbal, gimbal->ecd_angle.yaw, 0);
    }
  }

  return RM_OK;
}

int32_t gimbal_set_offset(struct gimbal *gimbal, uint16_t yaw_ecd, uint16_t pitch_ecd)
{
  if (gimbal == NULL)
    return -RM_INVAL;

  gimbal->param.yaw_ecd_center = yaw_ecd;
  gimbal->param.pitch_ecd_center = pitch_ecd;

  return RM_OK;
}

int32_t gimbal_pitch_enable(struct gimbal *gimbal)
{
  if (gimbal == NULL)
    return -RM_INVAL;

  controller_enable(&(gimbal->ctrl[PITCH_MOTOR_INDEX]));

  return RM_OK;
}

int32_t gimbal_pitch_disable(struct gimbal *gimbal)
{
  if (gimbal == NULL)
    return -RM_INVAL;

  controller_disable(&(gimbal->ctrl[PITCH_MOTOR_INDEX]));

  return RM_OK;
}

int32_t gimbal_yaw_enable(struct gimbal *gimbal)
{
  if (gimbal == NULL)
    return -RM_INVAL;

  controller_enable(&(gimbal->ctrl[YAW_MOTOR_INDEX]));

  return RM_OK;
}

int32_t gimbal_yaw_disable(struct gimbal *gimbal)
{
  if (gimbal == NULL)
    return -RM_INVAL;

  controller_disable(&(gimbal->ctrl[YAW_MOTOR_INDEX]));

  return RM_OK;
}

int32_t gimbal_execute(struct gimbal *gimbal)
{
  float motor_out;
  struct motor_data *pdata;

  if (gimbal == NULL)
    return -RM_INVAL;

  if (gimbal->mode.bit.yaw_mode == GYRO_MODE)
  {
    struct controller *ctrl;
    float center_offset;
    float yaw;

    yaw = gimbal->gyro_target_angle.yaw;
    center_offset = gimbal->sensor.gyro_angle.yaw - gimbal->ecd_angle.yaw;
    ctrl = &(gimbal->ctrl[YAW_MOTOR_INDEX]);

    VAL_LIMIT(yaw, YAW_ANGLE_MIN + center_offset, YAW_ANGLE_MAX + center_offset);
    controller_set_input(ctrl, yaw);
  }
  else
  {
    struct controller *ctrl;
    float yaw;
    yaw = gimbal->ecd_target_angle.yaw;
    ctrl = &(gimbal->ctrl[YAW_MOTOR_INDEX]);
    VAL_LIMIT(yaw, YAW_ANGLE_MIN, YAW_ANGLE_MAX);
    controller_set_input(ctrl, yaw);
  }

  if (gimbal->mode.bit.pitch_mode == GYRO_MODE)
  {
    struct controller *ctrl;
    float center_offset;
    float pitch;

    pitch = gimbal->gyro_target_angle.pitch;
    center_offset = gimbal->sensor.gyro_angle.pitch - gimbal->ecd_angle.pitch;
    ctrl = &(gimbal->ctrl[PITCH_MOTOR_INDEX]);

    VAL_LIMIT(pitch, PITCH_ANGLE_MIN + center_offset, PITCH_ANGLE_MAX + center_offset);
    controller_set_input(ctrl, pitch);
  }
  else
  {
    struct controller *ctrl;
    float pitch;
    pitch = gimbal->ecd_target_angle.pitch;
    ctrl = &(gimbal->ctrl[PITCH_MOTOR_INDEX]);
    VAL_LIMIT(pitch, PITCH_ANGLE_MIN, PITCH_ANGLE_MAX);
    controller_set_input(ctrl, pitch);
  }
  
  pdata = motor_device_get_data(&(gimbal->motor[YAW_MOTOR_INDEX]));
  gimbal->ecd_angle.yaw = YAW_MOTOR_POSITIVE_DIR * gimbal_get_ecd_angle(pdata->ecd, gimbal->param.yaw_ecd_center) / ENCODER_ANGLE_RATIO;
  controller_execute(&(gimbal->ctrl[YAW_MOTOR_INDEX]), (void *)gimbal);
  controller_get_output(&(gimbal->ctrl[YAW_MOTOR_INDEX]), &motor_out);
  motor_device_set_current(&(gimbal->motor[YAW_MOTOR_INDEX]), (int16_t)YAW_MOTOR_POSITIVE_DIR * motor_out);

  pdata = motor_device_get_data(&(gimbal->motor[PITCH_MOTOR_INDEX]));
  gimbal->ecd_angle.pitch = PITCH_MOTOR_POSITIVE_DIR * gimbal_get_ecd_angle(pdata->ecd, gimbal->param.pitch_ecd_center) / ENCODER_ANGLE_RATIO;
  controller_execute(&(gimbal->ctrl[PITCH_MOTOR_INDEX]), (void *)gimbal);
  controller_get_output(&(gimbal->ctrl[PITCH_MOTOR_INDEX]), &motor_out);
  motor_device_set_current(&(gimbal->motor[PITCH_MOTOR_INDEX]), (int16_t)PITCH_MOTOR_POSITIVE_DIR * motor_out);

  return RM_OK;
}

int32_t gimbal_rate_update(struct gimbal *gimbal, float yaw_rate, float pitch_rate)
{
  if (gimbal == NULL)
    return -RM_INVAL;

  gimbal->sensor.rate.yaw_rate = yaw_rate;
  gimbal->sensor.rate.pitch_rate = pitch_rate;

  return RM_OK;
}

int32_t gimbal_yaw_gyro_update(struct gimbal *gimbal, float yaw)
{
  if (gimbal == NULL)
    return -RM_INVAL;

  gimbal->sensor.gyro_angle.yaw = yaw;

  return RM_OK;
}

int32_t gimbal_pitch_gyro_update(struct gimbal *gimbal, float pitch)
{
  if (gimbal == NULL)
    return -RM_INVAL;

  gimbal->sensor.gyro_angle.pitch = pitch;

  return RM_OK;
}

int32_t gimbal_get_info(struct gimbal *gimbal, struct gimbal_info *info)
{
  if (gimbal == NULL)
    return -RM_INVAL;

  info->yaw_ecd_angle = gimbal->ecd_angle.yaw;
  info->pitch_ecd_angle = gimbal->ecd_angle.pitch;

  ANGLE_LIMIT_360(info->yaw_gyro_angle, gimbal->sensor.gyro_angle.yaw);
  ANGLE_LIMIT_360_TO_180(info->yaw_gyro_angle);

  info->mode = gimbal->mode.state;
  info->pitch_gyro_angle = gimbal->sensor.gyro_angle.pitch;
  info->yaw_rate = gimbal->sensor.rate.yaw_rate;
  info->pitch_rate = gimbal->sensor.rate.pitch_rate;

  return RM_OK;
}

gimbal_t gimbal_find(const char *name)
{
  struct object *object;

  object = object_find(name, Object_Class_Gimbal);

  return (gimbal_t)object;
}

static int16_t gimbal_get_ecd_angle(int16_t raw_ecd, int16_t center_offset)
{
  int16_t tmp = 0;
  if (center_offset >= 4096)
  {
    if (raw_ecd > center_offset - 4096)
      tmp = raw_ecd - center_offset;
    else
      tmp = raw_ecd + 8192 - center_offset;
  }
  else
  {
    if (raw_ecd > center_offset + 4096)
      tmp = raw_ecd - 8192 - center_offset;
    else
      tmp = raw_ecd - center_offset;
  }
  return tmp;
}

static int32_t gimbal_set_yaw_gyro_angle(struct gimbal *gimbal, float yaw, uint8_t mode)
{
  if (gimbal == NULL)
    return -RM_INVAL;
  float yaw_offset, yaw_now, yaw_target;

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

  return RM_OK;
}

static int32_t yaw_gyro_input_convert(struct controller *ctrl, void *input)
{
  cascade_feedback_t cascade_fdb = (cascade_feedback_t)(ctrl->feedback);
  gimbal_t data = (gimbal_t)input;
  cascade_fdb->outer_fdb = data->sensor.gyro_angle.yaw;
  cascade_fdb->inter_fdb = data->sensor.rate.yaw_rate;
  return RM_OK;
}

static int32_t yaw_ecd_input_convert(struct controller *ctrl, void *input)
{
  cascade_feedback_t cascade_fdb = (cascade_feedback_t)(ctrl->feedback);
  gimbal_t data = (gimbal_t)input;
  cascade_fdb->outer_fdb = data->ecd_angle.yaw;
  cascade_fdb->inter_fdb = data->sensor.rate.yaw_rate;
  return RM_OK;
}

static int32_t pitch_gyro_input_convert(struct controller *ctrl, void *input)
{
  cascade_feedback_t cascade_fdb = (cascade_feedback_t)(ctrl->feedback);
  gimbal_t data = (gimbal_t)input;
  cascade_fdb->outer_fdb = data->sensor.gyro_angle.pitch;
  cascade_fdb->inter_fdb = data->sensor.rate.pitch_rate;
  return RM_OK;
}

static int32_t pitch_ecd_input_convert(struct controller *ctrl, void *input)
{
  cascade_feedback_t cascade_fdb = (cascade_feedback_t)(ctrl->feedback);
  gimbal_t data = (gimbal_t)input;
  cascade_fdb->outer_fdb = data->ecd_angle.pitch;
  cascade_fdb->inter_fdb = data->sensor.rate.pitch_rate;
  return RM_OK;
}
