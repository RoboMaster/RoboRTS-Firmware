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

#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#ifdef CHASSIS_H_GLOBAL
  #define CHASSIS_H_EXTERN 
#else
  #define CHASSIS_H_EXTERN extern
#endif

#include "motor.h"
#include "mecanum.h"
#include "single_gyro.h"
#include "pid_controller.h"

typedef struct chassis *chassis_t;

struct chassis_acc
{
  float ax;
  float ay;
  float wz;
};
  
struct chassis
{
  struct object parent;
  struct mecanum mecanum;

  struct chassis_acc acc;

  struct motor_device motor[4];
  struct pid motor_pid[4];
  struct pid_feedback motor_feedback[4];
  struct controller ctrl[4];
};

struct chassis_info
{
  float v_x_mm;
  float v_y_mm;
  float rate_deg;
  float position_x_mm;
  float position_y_mm;
  float angle_deg;
  float yaw_gyro_angle;
  float yaw_gyro_rate;
  float wheel_rpm[4];
};

chassis_t chassis_find(const char *name);

int32_t chassis_pid_register(struct chassis *chassis, const char *name, enum device_can can);
int32_t chassis_execute(struct chassis *chassis);
int32_t chassis_gyro_updata(struct chassis *chassis, float yaw_angle, float yaw_rate);
int32_t chassis_set_vw(struct chassis *chassis, float vw);
int32_t chassis_set_vx_vy(struct chassis *chassis, float vx, float vy);
int32_t chassis_set_speed(struct chassis *chassis, float vx, float vy, float vw);
int32_t chassis_set_acc(struct chassis *chassis, float ax, float ay, float wz);
int32_t chassis_set_offset(struct chassis *chassis, float offset_x, float offset_y);
int32_t chassis_get_info(struct chassis *chassis, struct chassis_info *info);

int32_t chassis_enable(struct chassis *chassis);
int32_t chassis_disable(struct chassis *chassis);

#endif // __CHASSIS_H__
