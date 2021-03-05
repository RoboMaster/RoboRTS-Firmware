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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "mecanum.h"

#ifndef RADIAN_COEF
    #define RADIAN_COEF 57.3f
#endif

#define MEC_VAL_LIMIT(val, min, max) \
  do                                 \
  {                                  \
    if ((val) <= (min))              \
    {                                \
      (val) = (min);                 \
    }                                \
    else if ((val) >= (max))         \
    {                                \
      (val) = (max);                 \
    }                                \
  } while (0)

/**
  * @brief mecanum glb_chassis velocity decomposition.F:forword; B:backword; L:left; R:right
  * @param input : ccx=+vx(mm/s)  ccy=+vy(mm/s)  ccw=+vw(deg/s)
  *        output: every wheel speed(rpm)
  * @note  1=FR 2=FL 3=BL 4=BR
  */
void mecanum_calculate(struct mecanum *mec)
{
    static float rotate_ratio_fr;
    static float rotate_ratio_fl;
    static float rotate_ratio_bl;
    static float rotate_ratio_br;
    static float wheel_rpm_ratio;

    rotate_ratio_fr = ((mec->param.wheelbase + mec->param.wheeltrack) / 2.0f - mec->param.rotate_x_offset + mec->param.rotate_y_offset) / RADIAN_COEF;
    rotate_ratio_fl = ((mec->param.wheelbase + mec->param.wheeltrack) / 2.0f - mec->param.rotate_x_offset - mec->param.rotate_y_offset) / RADIAN_COEF;
    rotate_ratio_bl = ((mec->param.wheelbase + mec->param.wheeltrack) / 2.0f + mec->param.rotate_x_offset - mec->param.rotate_y_offset) / RADIAN_COEF;
    rotate_ratio_br = ((mec->param.wheelbase + mec->param.wheeltrack) / 2.0f + mec->param.rotate_x_offset + mec->param.rotate_y_offset) / RADIAN_COEF;

    wheel_rpm_ratio = 60.0f / (mec->param.wheel_perimeter * MOTOR_DECELE_RATIO);

    MEC_VAL_LIMIT(mec->speed.vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED); //mm/s
    MEC_VAL_LIMIT(mec->speed.vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED); //mm/s
    MEC_VAL_LIMIT(mec->speed.vw, -MAX_CHASSIS_VW_SPEED, MAX_CHASSIS_VW_SPEED); //deg/s

    float wheel_rpm[4];
    float max = 0;

    wheel_rpm[0] = (-mec->speed.vx - mec->speed.vy - mec->speed.vw * rotate_ratio_fr) * wheel_rpm_ratio;
    wheel_rpm[1] = (mec->speed.vx - mec->speed.vy - mec->speed.vw * rotate_ratio_fl) * wheel_rpm_ratio;
    wheel_rpm[2] = (mec->speed.vx + mec->speed.vy - mec->speed.vw * rotate_ratio_bl) * wheel_rpm_ratio;
    wheel_rpm[3] = (-mec->speed.vx + mec->speed.vy - mec->speed.vw * rotate_ratio_br) * wheel_rpm_ratio;

    //find max item
    for (uint8_t i = 0; i < 4; i++)
    {
        if (fabs(wheel_rpm[i]) > max)
        {
            max = fabs(wheel_rpm[i]);
        }
    }

    //equal proportion
    if (max > MAX_WHEEL_RPM)
    {
        float rate = MAX_WHEEL_RPM / max;
        for (uint8_t i = 0; i < 4; i++)
        {
            wheel_rpm[i] *= rate;
        }
    }
    memcpy(mec->wheel_rpm, wheel_rpm, 4 * sizeof(float));
}

/**
  * @brief get mecanum chassis odometry, using global yaw gyro angle.
  * @param
  * @note
  */
void mecanum_position_measure(struct mecanum *mec, struct mecanum_motor_fdb wheel_fdb[])
{
    static float rotate_ratio_fr;
    static float rotate_ratio_fl;
    static float rotate_ratio_bl;
    static float rotate_ratio_br;
    static float rpm_ratio;
    static float ecd_ratio;
    static double mecanum_angle;
    static double last_d_x, last_d_y, last_d_w, d_x, d_y, d_w, diff_d_x, diff_d_y, diff_d_w;
    static double position_x, position_y, angle_w;
    static double v_x, v_y, v_w;

    rotate_ratio_fr = ((mec->param.wheelbase + mec->param.wheeltrack) / 2.0f -
                       mec->param.rotate_x_offset + mec->param.rotate_y_offset);
    rotate_ratio_fl = ((mec->param.wheelbase + mec->param.wheeltrack) / 2.0f -
                       mec->param.rotate_x_offset - mec->param.rotate_y_offset);
    rotate_ratio_bl = ((mec->param.wheelbase + mec->param.wheeltrack) / 2.0f +
                       mec->param.rotate_x_offset - mec->param.rotate_y_offset);
    rotate_ratio_br = ((mec->param.wheelbase + mec->param.wheeltrack) / 2.0f +
                       mec->param.rotate_x_offset + mec->param.rotate_y_offset);
    rpm_ratio = mec->param.wheel_perimeter * MOTOR_DECELE_RATIO / (4 * 60.0f);
    ecd_ratio = mec->param.wheel_perimeter * MOTOR_DECELE_RATIO / (4 * MOTOR_ENCODER_ACCURACY);

    last_d_x = d_x;
    last_d_y = d_y;
    last_d_w = d_w;
    d_x = ecd_ratio * (-wheel_fdb[0].total_ecd + wheel_fdb[1].total_ecd + wheel_fdb[2].total_ecd - wheel_fdb[3].total_ecd);
    d_y = ecd_ratio * (-wheel_fdb[0].total_ecd - wheel_fdb[1].total_ecd + wheel_fdb[2].total_ecd + wheel_fdb[3].total_ecd);
    d_w = ecd_ratio * (-wheel_fdb[0].total_ecd / rotate_ratio_fr - wheel_fdb[1].total_ecd / rotate_ratio_fl - wheel_fdb[2].total_ecd / rotate_ratio_bl - wheel_fdb[3].total_ecd / rotate_ratio_br);

    diff_d_x = d_x - last_d_x;
    diff_d_y = d_y - last_d_y;
    diff_d_w = d_w - last_d_w;

    /* use glb_chassis gyro angle data */
    mecanum_angle = mec->gyro.yaw_gyro_angle / RADIAN_COEF;

    position_x += diff_d_x * cos(mecanum_angle) - diff_d_y * sin(mecanum_angle);
    position_y += diff_d_x * sin(mecanum_angle) + diff_d_y * cos(mecanum_angle);

    angle_w += diff_d_w;

    mec->position.position_x_mm = position_x;        //mm
    mec->position.position_y_mm = position_y;        //mm
    mec->position.angle_deg = angle_w * RADIAN_COEF; //degree

    v_x = rpm_ratio * (-wheel_fdb[0].speed_rpm + wheel_fdb[1].speed_rpm + wheel_fdb[2].speed_rpm - wheel_fdb[3].speed_rpm);
    v_y = rpm_ratio * (-wheel_fdb[0].speed_rpm - wheel_fdb[1].speed_rpm + wheel_fdb[2].speed_rpm + wheel_fdb[3].speed_rpm);
    v_w = rpm_ratio * (-wheel_fdb[0].speed_rpm / rotate_ratio_fr - wheel_fdb[1].speed_rpm / rotate_ratio_fl - wheel_fdb[2].speed_rpm / rotate_ratio_bl - wheel_fdb[3].speed_rpm / rotate_ratio_br);

    mec->position.v_x_mm = v_x;                 //mm/s
    mec->position.v_y_mm = v_y;                 //mm/s
    mec->position.rate_deg = v_w * RADIAN_COEF; //degree/s
}
