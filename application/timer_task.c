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

#include "timer_task.h"

struct soft_timer soft_timer[TIMER_ELEMENT_NUM_MAX - 1];

void timer_task(void const *argument)
{
  uint32_t period = osKernelSysTick();

  while (1)
  {
    TimerISR_Hook();

    for (int i = 0; i < TIMER_ELEMENT_NUM_MAX - 1; i++)
    {
      if ((soft_timer[i].id != 0) && (soft_timer[i].soft_timer_callback != NULL))
      {
        if (soft_timer_check(soft_timer[i].id) == SOFT_TIMER_TIMEOUT)
        {
          soft_timer[i].soft_timer_callback(soft_timer[i].argc);

          soft_timer_update(soft_timer[i].id, soft_timer[i].ticks);
        }
      }
    }

    osDelayUntil(&period, 1);
  }
}

int32_t soft_timer_register(int32_t (*soft_timer_callback)(void *argc), void *argc, uint32_t ticks)
{
  for (int i = 0; i < TIMER_ELEMENT_NUM_MAX - 1; i++)
  {
    if (soft_timer[i].id == 0)
    {
      soft_timer[i].id = soft_timer_req(ticks);
      soft_timer[i].ticks = ticks;
      soft_timer[i].argc = argc;
      soft_timer[i].soft_timer_callback = soft_timer_callback;
      return i;
    }
  }
  return -1;
}
