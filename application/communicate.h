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

#ifndef __COMMUNICATE_H__
#define __COMMUNICATE_H__

#ifdef COMMUNICATE_H_GLOBAL
  #define COMMUNICATE_H_EXTERN 
#else
  #define COMMUNICATE_H_EXTERN extern
#endif

#define GIMBAL_CAN_ID         0x500
#define CHASSIS_CAN_ID        0x600

#define RECV_PROTOCOL_SIGNAL      ( 1 << 0 )
#define SEND_PROTOCOL_SIGNAL      ( 1 << 1 )
#define REFEREE_SIGNAL            ( 1 << 2 )

enum interface{
  USB_INTERFACE = 0,
  CAN2_0x500_INTERFACE,
  CAN2_0x600_INTERFACE,
};

struct uwb_data{
  int16_t   x;
  int16_t   y;
  uint16_t  yaw;
  int16_t   distance[6];
  uint16_t  error;
  uint16_t  res;
};

int32_t gimbal_adjust(void);
void communicate_task(void const * argument);

#endif // __COMMUNICATE_H__
