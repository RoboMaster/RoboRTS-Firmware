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
/** @file detect_task.h
 *  @version 1.1
 *  @date June 2017
 *
 *  @brief detect module offline or online task
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#ifndef __ERROR_TASK_H__
#define __ERROR_TASK_H__

#include "stm32f4xx_hal.h"
#include "infantry_info.h"

/* detect task period time (ms) */
#define DETECT_TASK_PERIOD 50

typedef enum
{
  DEV_OFFLINE     = 0,
  DEV_RUNNING_ERR = 1,
  SYS_CONFIG_ERR  = 2,
} err_type_e;

typedef struct
{
  uint16_t set_timeout;
  uint16_t delta_time;
  uint32_t last_time;
} offline_dev_t;

typedef struct
{
  /* enable the device error detect */
  uint8_t  enable;
  /* device error exist flag */
  uint8_t  err_exist;
  /* device error priority */
  uint8_t  pri;
  /* device error type */
  uint8_t  type;
  /* the pointer of device offline param */
  offline_dev_t *dev;
} err_dev_t;

typedef struct
{
  /* the pointer of the highest priority error device */
  err_dev_t *err_now;
  err_id_e  err_now_id;
  /* all device be detected list */
  err_dev_t list[ERROR_LIST_LENGTH];

  /* error alarm relevant */
  uint16_t err_count;
  uint16_t beep_tune;
  uint16_t beep_ctrl;
} global_err_t;

extern global_err_t g_err;

void global_err_detector_init(void);
void err_detector_hook(int err_id);
void detect_task(void const *argu);
void detector_param_init(void);

static void module_offline_callback(void);
static void module_offline_detect(void);


#endif
