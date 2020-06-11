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

#ifndef __SYS_INCLUDES_H__
#define __SYS_INCLUDES_H__

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "errno.h"
#include "linux_list.h"

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

#include "my_math.h"

/* boolean type definitions */
#ifndef TRUE
    #define TRUE                         1               /**< boolean true  */
#endif

#ifndef FALSE
    #define FALSE                        0               /**< boolean fails */
#endif

#ifndef ENABLE
    #define ENABLE                       1
#endif

#ifndef DISABLE
    #define DISABLE                      0
#endif

#ifndef RAD_TO_DEG
    #define RAD_TO_DEG 57.29f
#endif

#define INT_STATE              uint32_t
#define MASTER_INT_STATE_GET() __get_PRIMASK()
#define MASTER_INT_ENABLE()    do{__enable_irq();  }while(0)
#define MASTER_INT_DISABLE()   do{__disable_irq(); }while(0)
#define MASTER_INT_RESTORE(x)  do{__set_PRIMASK(x);}while(0)

#define CRITICAL_SETCION_ENTER()                      \
    do                                                \
    {                                                 \
        INT_STATE cpu_state = MASTER_INT_STATE_GET(); \
        MASTER_INT_DISABLE();

#define CRITICAL_SETCION_EXIT()        \
        MASTER_INT_RESTORE(cpu_state); \
    }                                  \
    while (0)

#define var_cpu_sr() register unsigned long cpu_sr

#define enter_critical()      \
  do                          \
  {                           \
    cpu_sr = __get_PRIMASK(); \
    __disable_irq();          \
  } while (0)

#define exit_critical()    \
  do                       \
  {                        \
    __set_PRIMASK(cpu_sr); \
  } while (0)

#define MUTEX_DECLARE(mutex) unsigned long mutex
#define MUTEX_INIT(mutex)    do{mutex = 0;}while(0)
#define MUTEX_LOCK(mutex)    do{__disable_irq();}while(0)
#define MUTEX_UNLOCK(mutex)  do{__enable_irq();}while(0)

#define device_assert(EX)\
  if (!(EX))           \
  {                    \
    log_assert("fail! %s %s %s", #EX, __FILE__, __LINE__);   \
    while(1);          \
  }

uint32_t get_time_ms(void);
uint32_t get_time_us(void);
float get_time_ms_us(void);

#endif // __SYS_INCLUDES_H__
