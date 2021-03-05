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

/* Includes ------------------------------------------------------------------*/
/******************PROTOCL INCLUDE*********************/
#include <stdio.h>
#include <stdarg.h>

#include "protocol_common.h"
/******************USER INCLUDE************************/
#include "cmsis_os.h"

/* Exported functions --------------------------------------------------------*/
/**
  * @brief  alloc size bytes
  * @param  size
  * @retval memory block pointer
  */
void *protocol_p_malloc(uint32_t size)
{
    return heap_malloc(size);
}

/**
  * @brief  free memory
  * @param
  * @retval void
  */
void protocol_p_free(void *ptr)
{
    heap_free(ptr);
}

/**
  * @brief  get os ticks
  * @param  void
  * @retval time, unit:ms
  */
uint32_t protocol_p_get_time(void)
{
    return osKernelSysTick() / portTICK_PERIOD_MS;
}
