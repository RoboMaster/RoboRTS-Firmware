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

#ifndef __TIMER_TASK_H__
#define __TIMER_TASK_H__

#include "sys.h"
#include "soft_timer.h"
#include "period.h"

typedef int32_t (*soft_timer_callback)(void *argc);

struct soft_timer
{
    uint8_t id;
    uint32_t ticks;
    void *argc;
    soft_timer_callback callback;
};

#define OS_TIMER_STACK_SIZE    1024
#define OS_TIMER_PRIORITY      osPriorityNormal

void soft_timer_FreeRTOS_init(void);
int32_t soft_timer_register(soft_timer_callback func, void *argc, uint32_t ticks);
void timer_task(void const *argument);

#endif // __TIMER_TASK_H__
