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

#ifndef __RAMP_H__
#define __RAMP_H__

#ifdef RAMP_H_GLOBAL
    #define RAMP_H_EXTERN
#else
    #define RAMP_H_EXTERN extern
#endif

#include "stdint.h"

typedef struct ramp_v0_t
{
    int32_t count;
    int32_t scale;
    float   out;
    void (*init)(struct ramp_v0_t *ramp, int32_t scale);
    float (*calc)(struct ramp_v0_t *ramp);
} ramp_v0_t;

#define RAMP_GEN_DAFAULT     \
  {                          \
    .count = 0,              \
    .scale = 0,              \
    .out = 0,                \
    .init = &ramp_v0_init,      \
    .calc = &ramp_v0_calculate, \
  }

void  ramp_v0_init(ramp_v0_t *ramp, int32_t scale);
float ramp_v0_calculate(ramp_v0_t *ramp);

#endif // __RAMP_H__

