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

#ifndef __PERIOD_H__
#define __PERIOD_H__

#ifdef PERIO_H_GLOBAL
  #define PERIOD_H_EXTERN 
#else
  #define PERIOD_H_EXTERN extern
#endif

#include "sys.h"

#define MAX_PERIOD_NUM 64

struct period
{
  uint8_t used;
  uint8_t start_time;
};

int32_t get_period_init(void);
int32_t get_period_start(void);
float get_period_end(uint32_t id);

#endif // __PERIOD_H__
