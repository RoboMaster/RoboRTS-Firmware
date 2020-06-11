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

#ifndef __AHRS_H__
#define __AHRS_H__

#ifdef AHRS_H_GLOBAL
    #define AHRS_H_EXTERN
#else
    #define AHRS_H_EXTERN extern
#endif

struct ahrs_sensor
{
    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

    float mx;
    float my;
    float mz;

    float roll;
    float pitch;
    float yaw;
};

struct uwb_data
{
    int16_t   x;
    int16_t   y;
    uint16_t  yaw;
    int16_t   distance[6];
    uint16_t  error;
    uint16_t  res;
};

#endif // __AHRS_H__
