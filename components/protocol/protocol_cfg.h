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

#ifndef _PROTOCOL_CFG_H_
#define _PROTOCOL_CFG_H_

#include "sys.h"

#define PROTOCOL_VERSION                (0)                 /* protocol version */

#define PROTOCOL_SEND_CMD_MAX_NUM       (20)                /* Max CMD Number */
#define PROTOCOL_RECV_CMD_MAX_NUM       (20)                /* Max CMD Number */

#define PROTOCOL_DEV_VERSION            ("V0.0.6")          /* develop version */

#define PROTOCOL_MAX_DATA_LEN           (512)               /* protocol data frame max length */
#define PROTOCOL_HEADER                 (0xAAu)             /* Frame Header */
#define PROTOCOL_INTERFACE_MAX          (10)                /* max interface num */
#define PROTOCOL_OBJ_NAME_MAX_LEN       (50)                /* name max length */
#define PROTOCOL_ROUTE_TABLE_MAX_NUM    (254)               /* router max num(this value must be less than 254.) */

#define PROTOCOL_SEND_DBG_PRINTF_SET    PROTOCOL_DISABLE     /* send dbg log enable */
#define PROTOCOL_SEND_ERR_PRINTF_SET    PROTOCOL_DISABLE     /* send error log enable */
#define PROTOCOL_RCV_DBG_PRINTF_SET     PROTOCOL_ENABLE     /* recv dbg log enable */
#define PROTOCOL_RCV_ERR_PRINTF_SET     PROTOCOL_ENABLE     /* recv error log enable */
#define PROTOCOL_ERR_INFO_PRINTF_SET    PROTOCOL_ENABLE     /* information log enable */
#define PROTOCOL_OTHER_INFO_PRINTF_SET  PROTOCOL_ENABLE     /* other log output enable */

#define PROTOCOL_AUTO_LOOPBACK          PROTOCOL_ENABLE     /* loop back enable */
#define PROTOCOL_LOOPBACK_SIZE          1024u               /* loop back FIFO size */


#define PROTOCOL_ROUTE_FORWARD          PROTOCOL_ENABLE     /* forward route enable*/

#endif /* _PROTOCOL_CFG_H_ */
