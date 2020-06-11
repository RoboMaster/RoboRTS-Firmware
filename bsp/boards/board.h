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

#ifndef __BOARD_H__
#define __BOARD_H__

#ifdef BOARD_H_GLOBAL
    #define BOARD_H_EXTERN
#else
    #define BOARD_H_EXTERN extern
#endif

#include "sys.h"

#include "drv_can.h"
#include "drv_dr16.h"
#include "drv_flash.h"
#include "drv_imu.h"
#include "drv_io.h"
#include "drv_uart.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
void board_config(void);

#endif // __BOARD__
