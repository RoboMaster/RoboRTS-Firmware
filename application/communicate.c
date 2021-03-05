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

#include "cmsis_os.h"
#include "infantry_cmd.h"
#include "referee_system.h"
#include "protocol.h"

static void communicate_task(void const *argument);

osThreadId communicate_task_t;

void communicate_task_init(void)
{
    osThreadDef(COMMUNICATE_TASK, communicate_task, osPriorityNormal, 0, 512);
    communicate_task_t = osThreadCreate(osThread(COMMUNICATE_TASK), NULL);
}

int32_t report_firmware_version(uint8_t *buff, uint16_t len)
{
    return FIRMWARE_VERSION;
}

/**
  * @brief  protocol, referee protocol unpack task
  * @param
  * @retval void
  */
void communicate_task(void const *argument)
{

    protocol_rcv_cmd_register(CMD_REPORT_VERSION, report_firmware_version);


    while (1)
    {
        protocol_send_flush();
        protocol_unpack_flush();

        /* referee protocol unpack */
        referee_unpack_fifo_data();
        osDelay(1);
    }
}
