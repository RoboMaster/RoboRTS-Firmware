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

/* Includes ------------------------------------------------------------------*/
/******************PROTOCL INCLUDE*********************/
#include <stdio.h>
#include <stdarg.h>

#include "protocol_common.h"
/******************USER INCLUDE************************/
#include "cmsis_os.h"

/* Exported functions --------------------------------------------------------*/
/**
  * @brief  协议内存分配接口函数，用户可以根据实际情况对本函数进行修改
  * @param  size 需要分配内存大小，单位为字节
  * @retval 若分配成功返回分配内存的首地址指针，否则返回NULL
  */
void *protocol_p_malloc(uint32_t size)
{
  return heap_malloc(size);
}

/**
  * @brief  协议内存释放接口函数，用户可以根据实际情况对本函数进行修改
  * @param  ptr 需要释放内存的首地址指针
  * @retval void
  */
void protocol_p_free(void *ptr)
{
  heap_free(ptr);
}

/**
  * @brief  协议获取系统时间接口函数(毫秒)，用户可以根据实际情况对本函数进行修改
  * @param  void
  * @retval 当前系统时间,单位为毫秒
  */
uint32_t protocol_p_get_time(void)
{
  return osKernelSysTick() / portTICK_PERIOD_MS;
}
