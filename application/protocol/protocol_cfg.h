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

#ifndef _PROTOCOL_CFG_H_
#define _PROTOCOL_CFG_H_

#include "sys.h"

#define PROTOCOL_VERSION                (0)                 /*协议版本*/

#define PROTOCOL_CMD_MAX_NUM            (50)

#define PROTOCOL_DEV_VERSION            ("V0.0.6")          /*协议开发版本号*/

#define PROTOCOL_MAX_DATA_LEN           (512)               /*协议传输每包最大数据长度*/
#define PROTOCOL_HEADER                 (0xAAu)             /*协议包帧头*/
#define PROTOCOL_INTERFACE_MAX          (5)                 /*协议接口最大数量*/
#define PROTOCOL_OBJ_NAME_MAX_LEN       (32)                /*协议接口名字符串最大长度*/
#define PROTOCOL_ROUTE_TABLE_MAX_NUM    (254)               /*协议路由最大条数(不可以超过255)*/ 

#define PROTOCOL_SEND_DBG_PRINTF_SET    PROTOCOL_ENABLE     /*协议发送信息输出使能*/
#define PROTOCOL_SEND_ERR_PRINTF_SET    PROTOCOL_ENABLE     /*协议发送信息输出使能*/
#define PROTOCOL_RCV_DBG_PRINTF_SET     PROTOCOL_ENABLE     /*协议接收信息输出使能*/
#define PROTOCOL_RCV_ERR_PRINTF_SET     PROTOCOL_ENABLE     /*协议接收信息输出使能*/
#define PROTOCOL_ERR_INFO_PRINTF_SET    PROTOCOL_ENABLE     /*协议错误信息输出使能*/
#define PROTOCOL_OTHER_INFO_PRINTF_SET  PROTOCOL_ENABLE     /*协议其他信息输出使能*/

#define PROTOCOL_AUTO_LOOKBACK          PROTOCOL_ENABLE     /*协议自动回环使能*/

#define PROTOCOL_ROUTE_FOWARD           PROTOCOL_ENABLE     /*协议路由转发使能*/

#endif /* _PROTOCOL_CFG_H_ */
