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
/** @file ramp.h
 *  @version 1.0
 *  @date June 2017
 *
 *  @brief ramp contrl realization
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#ifndef __RAMP_H__
#define __RAMP_H__

#include "stm32f4xx_hal.h"

typedef struct ramp_t
{
  int32_t count;
  int32_t scale;
  float   out;
  void  (*init)(struct ramp_t *ramp, int32_t scale);
  float (*calc)(struct ramp_t *ramp);
}ramp_t;

#define RAMP_GEN_DAFAULT \
{ \
              .count = 0, \
              .scale = 0, \
              .out = 0, \
              .init = &ramp_init, \
              .calc = &ramp_calc, \
            } \
            
void  ramp_init(ramp_t *ramp, int32_t scale);
float ramp_calc(ramp_t *ramp);

#endif
