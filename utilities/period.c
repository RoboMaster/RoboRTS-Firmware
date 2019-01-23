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

#include "period.h"
#include "sys.h"
#include "errno.h"

static struct period SoftPerid[MAX_PERIOD_NUM] = {0};

static int32_t get_period_reset(struct period *period);

int32_t get_period_init(void)
{ 
  for (int i = 0; i < MAX_PERIOD_NUM; i++)
  {
    get_period_reset(&SoftPerid[i]);
  }
  return RM_OK;
}

int32_t get_period_start(void)
{
  for (int i = 0; i < MAX_PERIOD_NUM; i++)
  {
    if (SoftPerid[i].used == 0)
    {
      SoftPerid[i].used = 1;
      SoftPerid[i].start_time = get_time_ms_us();
    }
  }
  return RM_USED;
}

int32_t get_period_start_by_id(uint32_t id)
{
  if (id > MAX_PERIOD_NUM)
  {
    return RM_INVAL;
  }
  
  SoftPerid[id].used = 1;
  SoftPerid[id].start_time = get_time_ms_us();

  return RM_USED;
}

float get_period_end(uint32_t id)
{
  float period;

  if (id > MAX_PERIOD_NUM)
  {
    return RM_INVAL;
  }

  period = get_time_ms_us() - SoftPerid[id].start_time;
  SoftPerid[id].used = 0;
  SoftPerid[id].start_time = 0;
  return period;
}

static int32_t get_period_reset(struct period *period)
{
  period->used = 0;
  period->start_time = 0;
  return RM_OK;
}

static uint16_t sin_maxout = 5000;
static float sin_period = 20;

float sin_freq_output(void)
{
  uint32_t tick = get_time_ms();

  return sin_maxout*sin(2*3.14159265354/sin_period*tick);
}

