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

#ifndef __EVENT_H__
#define __EVENT_H__

#include "dbus.h"
#include "drv_dr16.h"

//DBUS: dr16
#define DBUS_MSG  0
#define DBUS_MSG_LEN DR16_DATA_LEN

//SENSOR: ATTITUDE
#define AHRS_MSG  1
#define AHRS_MSG_LEN sizeof(struct ahrs_sensor)

//UWB
#define UWB_MSG   2
#define UWB_MSG_LEN sizeof(struct uwb_data)

#endif //__EVENT_H__



