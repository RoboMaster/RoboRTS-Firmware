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

#ifndef __PROTOCOL_LOG_H__
#define __PROTOCOL_LOG_H__

#define LOG_TAG "protocol"
#define LOG_OUTPUT_LEVEL LOG_INFO
#include "log.h"

#define protocol_log_d(...)  log_d(__VA_ARGS__);
#define protocol_log_i(...)  log_i(__VA_ARGS__);
#define protocol_log_e(...)  log_e(__VA_ARGS__);

#include "protocol_cfg.h"

/********************DEFINE PRINTF**************************/

#if (PROTOCOL_SEND_DBG_PRINTF_SET == PROTOCOL_ENABLE)
    #ifndef PROTOCOL_SEND_DBG_PRINTF
        #define PROTOCOL_SEND_DBG_PRINTF(...) protocol_log_d(__VA_ARGS__);
    #endif
#else
    #ifndef PROTOCOL_SEND_DBG_PRINTF
        #define PROTOCOL_SEND_DBG_PRINTF(...)
    #endif
#endif

#if (PROTOCOL_SEND_ERR_PRINTF_SET == PROTOCOL_ENABLE)
    #ifndef PROTOCOL_SEND_ERR_PRINTF
        #define PROTOCOL_SEND_ERR_PRINTF(...) protocol_log_e(__VA_ARGS__);
    #endif
#else
    #ifndef PROTOCOL_SEND_ERR_PRINTF
        #define PROTOCOL_SEND_ERR_PRINTF(...)
    #endif
#endif

#if (PROTOCOL_RCV_DBG_PRINTF_SET == PROTOCOL_ENABLE)
    #ifndef PROTOCOL_RCV_DBG_PRINTF
        #define PROTOCOL_RCV_DBG_PRINTF(...) protocol_log_d(__VA_ARGS__);
    #endif
#else
    #ifndef PROTOCOL_RCV_DBG_PRINTF
        #define PROTOCOL_RCV_DBG_PRINTF(...)
    #endif
#endif

#if (PROTOCOL_RCV_ERR_PRINTF_SET == PROTOCOL_ENABLE)
    #ifndef PROTOCOL_RCV_ERR_PRINTF
        #define PROTOCOL_RCV_ERR_PRINTF(...) protocol_log_e(__VA_ARGS__);
    #endif
#else
    #ifndef PROTOCOL_RCV_ERR_PRINTF
        #define PROTOCOL_RCV_ERR_PRINTF(...)
    #endif
#endif

#if (PROTOCOL_ERR_INFO_PRINTF_SET == PROTOCOL_ENABLE)
    #ifndef PROTOCOL_ERR_INFO_PRINTF
        #define PROTOCOL_ERR_INFO_PRINTF(STA, FILE, LINE) protocol_s_error_info_printf(STA, FILE, LINE);
    #endif
#else
    #ifndef PROTOCOL_ERR_INFO_PRINTF
        #define PROTOCOL_ERR_INFO_PRINTF(STA, FILE, LINE)
    #endif
#endif

#if (PROTOCOL_OTHER_INFO_PRINTF_SET == PROTOCOL_ENABLE)
    #ifndef PROTOCOL_OTHER_INFO_PRINTF
        #define PROTOCOL_OTHER_INFO_PRINTF(...) protocol_log_i(__VA_ARGS__);
    #endif
#else
    #ifndef PROTOCOL_OTHER_INFO_PRINTF
        #define PROTOCOL_OTHER_INFO_PRINTF(...)
    #endif
#endif

#endif // __PROTOCOL_LOG_H__
