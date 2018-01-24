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
/** @file info_get_task.c
 *  @version 1.0
 *  @date Oct 2017
 *
 *  @brief get infantry sensor and control information
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "info_get_task.h"
#include "comm_task.h"
#include "info_interactive.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "cmsis_os.h"

/* stack usage monitor */
UBaseType_t info_stack_surplus;

/* information get task global parameter */
infantry_structure_t glb_struct;

uint32_t info_time_last;
int info_time_ms;
void info_get_task(void const *argu)
{
  osEvent event;
  
  while (1)
  {
    event = osSignalWait(INFO_GET_EXE_SIGNAL, osWaitForever);
    
    if (event.status == osEventSignal)
    {
      if (event.value.signals & INFO_GET_EXE_SIGNAL)
      {
        info_time_ms = HAL_GetTick() - info_time_last;
        info_time_last = HAL_GetTick();
        
        taskENTER_CRITICAL();
        
        keyboard_global_hook();
        
        get_chassis_info();
        get_gimbal_info();
        get_shoot_info();
        
        get_global_last_info();
        
        taskEXIT_CRITICAL();

        chassis_position_measure();
      }
    }
    
    info_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
  }

//  uint32_t info_time = osKernelSysTick();
//  while (1)
//  {
//    info_time_ms = HAL_GetTick() - info_time_last;
//    info_time_last = HAL_GetTick();
//    
//    taskENTER_CRITICAL();
//    
//    keyboard_global_hook();
//    
//    get_chassis_info();
//    get_gimbal_info();
//    get_shoot_info();
//    
//    taskEXIT_CRITICAL();

//    chassis_position_measure();
//    
//    osDelayUntil(&info_time, INFO_GET_PERIOD);
//    
//  }
}

static void get_global_last_info(void)
{
  glb_sw.last_sw1 = rc.sw1;
  glb_sw.last_sw2 = rc.sw2;
  
  
}

