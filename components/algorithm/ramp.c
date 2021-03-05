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

#include "ramp.h"

/**
  * @brief     ramp filter initialize
  * @param[in]
  * @retval    void
  */
void ramp_v0_init(ramp_v0_t *ramp, int32_t scale)
{
    ramp->count = 0;
    ramp->scale = scale;
}

/**
  * @brief     caculate output of ramp filter
  * @param[in] ramp: a ramp filter pointer
  * @retval    output
  */
float ramp_v0_calculate(ramp_v0_t *ramp)
{
    if (ramp->scale <= 0)
    {
        return 0;
    }

    if (ramp->count++ >= ramp->scale)
    {
        ramp->count = ramp->scale;
    }

    ramp->out = ramp->count / ((float)ramp->scale);
    return ramp->out;
}

