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

#include "board.h"
#include "os_timer.h"
#include "event_mgr.h"
#include "event.h"
#include "cmsis_os.h"
#include "sensor_task.h"

static void sensor_task(void const *argc);

osThreadId sensor_task_t;

void sensor_task_init(void)
{
    osThreadDef(SENSOR_TASK, sensor_task, osPriorityNormal, 0, 512);
    sensor_task_t = osThreadCreate(osThread(SENSOR_TASK), NULL);
}

float ahrs_run_time;

/**
  * @brief  sensor publisher
  * @param
  * @retval void
  */
void sensor_task(void const *argc)
{
    /* The parameters are not used. */
    (void)argc;
    TickType_t peroid = osKernelSysTick();
    ;
    struct ahrs_sensor gyro_sensor;

    static publisher_t ahrsPub;

    /* set gyro zero drift */
    bmi088_get_offset();

    imu_temp_ctrl_init();
    soft_timer_register(imu_temp_keep, (void *)NULL, 5);

    EventPostInit(&ahrsPub, AHRS_MSG, AHRS_MSG_LEN);

    while (1)
    {
        uint32_t time_id;

        get_period_start(&time_id);

        ahrs_update(&gyro_sensor, SENSOR_TASK_PERIOD);

        EventMsgPost(&ahrsPub, &gyro_sensor, AHRS_MSG_LEN);

        ahrs_run_time = get_period_end(time_id);

        osDelayUntil(&peroid, SENSOR_TASK_PERIOD);
    }
}
