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
/** @file ramp.c
 *  @version 1.0
 *  @date June 2017
 *
 *  @brief ramp contrl realization
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "ramp.h"

void ramp_init(ramp_t *ramp, int32_t scale)
{
  ramp->count = 0;
  ramp->scale = scale;
}

float ramp_calc(ramp_t *ramp)
{
  if (ramp->scale <= 0)
    return 0;
  
  if (ramp->count++ >= ramp->scale)
    ramp->count = ramp->scale;
  
  ramp->out = ramp->count / ((float)ramp->scale);
  return ramp->out;
}
