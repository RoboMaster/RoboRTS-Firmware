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

#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#ifdef GIMBAL_H_GLOBAL
  #define GIMBAL_H_EXTERN 
#else
  #define GIMBAL_H_EXTERN extern
#endif

/* gimbal relevant */
#define PITCH_ANGLE_MAX      20.0f
#define PITCH_ANGLE_MIN      -20.0f
#define YAW_ANGLE_MAX      70.0f
#define YAW_ANGLE_MIN      -70.0f

/************************** gimbal parameter *****************************/
/* the ratio of motor encoder value translate to degree */
#ifndef ENCODER_ANGLE_RATIO
  #define ENCODER_ANGLE_RATIO    (8192.0f/360.0f)
#endif 

#define RAD_TO_DEG 57.3f

/* the deceleration ratio of pitch axis motor */
#define PIT_DECELE_RATIO       1.0f
/* the deceleration ratio of yaw axis motor */
#define YAW_DECELE_RATIO       1.0f
/* the positive direction of pitch axis motor */
#define PITCH_MOTOR_POSITIVE_DIR  1.0f
/* the positive direction of yaw axis motor */
#define YAW_MOTOR_POSITIVE_DIR  1.0f

#include "motor.h"
#include "single_gyro.h"
#include "pid_controller.h"

#define YAW_MOTOR_INDEX 0
#define PITCH_MOTOR_INDEX 1

#define GIMBAL_SET_YAW (1<<0u)
#define GIMBAL_SET_PITCH (1<<1u)
#define GIMBAL_SET_ALL (GIMBAL_SET_YAW | GIMBAL_SET_PITCH)

#define ENCODER_MODE (0u)
#define GYRO_MODE (1u)

#define YAW_FASTEST (0u)
#define YAW_CLOCKWISE (1u)
#define YAW_ANTICLOCKWISE (2u)

typedef struct gimbal *gimbal_t;

struct gimbal_param
{
  int16_t pitch_ecd_center;
  int16_t yaw_ecd_center;
};

struct gimbal_p_y
{
  /* unit: degree */
  float yaw;
  float pitch;  
};

struct gimbal_rate
{
  /* unit: degree/s */
  float yaw_rate;
  float pitch_rate;  
};

struct gimbal_sensor
{
  struct gimbal_p_y gyro_angle;
  struct gimbal_rate rate;
};

struct gimbal
{
  struct object parent;
  struct gimbal_param param;

  union {
    uint8_t state;
    struct
    {
      uint8_t yaw_mode : 1;
      uint8_t pitch_mode : 1;
    } bit;
  } mode;

  struct gimbal_sensor sensor;  
  struct gimbal_p_y ecd_angle;
  
  struct gimbal_p_y gyro_target_angle;
  struct gimbal_p_y ecd_target_angle;

  struct motor_device motor[2];
  struct cascade cascade[2];
  struct cascade_feedback cascade_fdb[2];
  struct controller ctrl[2];
};

struct gimbal_info
{
  uint8_t mode;
  float yaw_ecd_angle;
  float pitch_ecd_angle;
  float yaw_gyro_angle;
  float pitch_gyro_angle;
  float yaw_rate;
  float pitch_rate;
};

gimbal_t gimbal_find(const char *name);
int32_t gimbal_cascade_register(struct gimbal *gimbal, const char *name, enum device_can can);
int32_t gimbal_execute(struct gimbal *gimbal);

int32_t gimbal_yaw_gyro_update(struct gimbal *gimbal, float yaw);
int32_t gimbal_pitch_gyro_update(struct gimbal *gimbal, float pitch);
int32_t gimbal_rate_update(struct gimbal *gimbal, float yaw_rate, float pitch_rate);

int32_t gimbal_set_pitch_mode(struct gimbal *gimbal, uint8_t mode);
int32_t gimbal_set_yaw_mode(struct gimbal *gimbal, uint8_t mode);

int32_t gimbal_set_pitch_delta(struct gimbal *gimbal, float pitch);
int32_t gimbal_set_yaw_delta(struct gimbal *gimbal, float yaw);
int32_t gimbal_set_pitch_speed(struct gimbal *gimbal, float pitch);
int32_t gimbal_set_yaw_speed(struct gimbal *gimbal, float yaw);
int32_t gimbal_set_pitch_angle(struct gimbal *gimbal, float pitch);
int32_t gimbal_set_yaw_angle(struct gimbal *gimbal, float yaw, uint8_t mode);

int32_t gimbal_set_offset(struct gimbal *gimbal, uint16_t yaw_ecd, uint16_t pitch_ecd);

int32_t gimbal_pitch_enable(struct gimbal *gimbal);
int32_t gimbal_pitch_disable(struct gimbal *gimbal);
int32_t gimbal_yaw_enable(struct gimbal *gimbal);
int32_t gimbal_yaw_disable(struct gimbal *gimbal);

int32_t gimbal_get_info(struct gimbal *gimbal, struct gimbal_info *info);

#endif // __GIMBAL_H__
