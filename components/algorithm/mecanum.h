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

#ifndef __MECANUM_H__
#define __MECANUM_H__

#ifdef MECANUM_H_GLOBAL
    #define MECANUM_H_EXTERN
#else
    #define MECANUM_H_EXTERN extern
#endif

/************************ chassis parameter ****************************/
/* the radius of wheel(mm) */
#define RADIUS 76
/* the perimeter of wheel(mm) */
#define PERIMETER 478

#include "appcfg.h"

#ifdef ICRA2019
    /* wheel track distance(mm) */
    #define WHEELTRACK 394
    /* wheelbase distance(mm) */
    #define WHEELBASE 415
#else
    /* wheel track distance(mm) */
    #define WHEELTRACK 400
    /* wheelbase distance(mm) */
    #define WHEELBASE 376
#endif

/* gimbal is relative to chassis center x axis offset(mm) */
#define ROTATE_X_OFFSET 7
/* gimbal is relative to chassis center y axis offset(mm) */
#define ROTATE_Y_OFFSET 0

/* chassis motor use 3508 */
/* the deceleration ratio of chassis motor */
#define MOTOR_DECELE_RATIO (1.0f / 19.0f)
/* single 3508 motor maximum speed, unit is rpm */
#define MAX_WHEEL_RPM 8500 //8347rpm = 3500mm/s
/* chassis maximum translation speed, unit is mm/s */
#define MAX_CHASSIS_VX_SPEED 3300 //8000rpm
#define MAX_CHASSIS_VY_SPEED 3300
/* chassis maximum rotation speed, unit is degree/s */
#define MAX_CHASSIS_VW_SPEED 300 //5000rpm

#define MOTOR_ENCODER_ACCURACY 8192.0f

/**
  * @brief  infantry structure configuration information
  */
struct mecanum_structure
{
    float wheel_perimeter; /* the perimeter(mm) of wheel */
    float wheeltrack;      /* wheel track distance(mm) */
    float wheelbase;       /* wheelbase distance(mm) */
    float rotate_x_offset; /* rotate offset(mm) relative to the x-axis of the chassis center */
    float rotate_y_offset; /* rotate offset(mm) relative to the y-axis of the chassis center */
};

struct mecanum_position
{
    float v_x_mm;
    float v_y_mm;
    float rate_deg;
    float position_x_mm;
    float position_y_mm;
    float angle_deg;
};

struct mecanum_speed
{
    float vx; // forward/back
    float vy; // left/right
    float vw; // anticlockwise/clockwise
};

struct mecanum_gyro
{
    float yaw_gyro_angle;
    float yaw_gyro_rate;
};

struct mecanum
{
    struct mecanum_structure param;
    struct mecanum_speed speed;
    struct mecanum_position position;
    struct mecanum_gyro gyro;
    float  wheel_rpm[4];
};

struct mecanum_motor_fdb
{
    float total_ecd;
    float speed_rpm;
};

void mecanum_calculate(struct mecanum *mec);
void mecanum_position_measure(struct mecanum *mec, struct mecanum_motor_fdb wheel_fdb[]);

#endif // __MECANUM_H__
