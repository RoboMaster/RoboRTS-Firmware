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

#include "period.h"
#include "sys.h"
#include "errno.h"

/* static soft timer buffer */
static struct period SoftPerid[MAX_PERIOD_NUM + 1] = {0};

/**
  * @brief      a time measurement start
  * @param[out] timer id
  * @retval     error code
  */
int32_t get_period_start(uint32_t *id)
{
    for (int i = 1; i < MAX_PERIOD_NUM; i++)
    {
        if (SoftPerid[i].used == 0)
        {
            SoftPerid[i].used = 1;
            SoftPerid[i].start_time = get_time_ms_us();

            *id = i;
            return E_OK;
        }
    }
    return E_ERROR;
}

/**
  * @brief     a time measurement stop
  * @param[in] timer id
  * @retval    time
  */
float get_period_end(uint32_t id)
{
    float period;
    period = get_time_ms_us() - SoftPerid[id].start_time;
    if ((SoftPerid[id].used == 1) && (id < MAX_PERIOD_NUM + 1))
    {
        SoftPerid[id].used = 0;
        SoftPerid[id].start_time = 0;
        return period;
    }
    return 0;
}

/**
  * @brief     a sin function generator
  * @param[in] A/T/b
  * @retval    output
  */
float sin_freq_output(float sin_maxout, float sin_period, float sin_b)
{
    uint32_t tick = get_time_ms();
    return sin_maxout * sin(2 * 3.14159265354 / sin_period * tick) + sin_b;
}

