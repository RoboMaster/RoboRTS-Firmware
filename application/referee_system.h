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

#ifndef __REFEREE_SYSTEM_H__
#define __REFEREE_SYSTEM_H__

#ifdef REFEREE_SYSTEM_H_GLOBAL
  #define REFEREE_SYSTEM_H_EXTERN 
#else
  #define REFEREE_SYSTEM_H_EXTERN extern
#endif

#include "sys.h"

#define REFEREE_SOF    0xA5

#define HEADER_LEN   sizeof(frame_header_t)
#define CMD_LEN      2    //cmdid bytes
#define CRC_LEN      2    //crc16 bytes

/** 
  * @brief  judgement data command id
  */
typedef enum
{
  GAME_INFO_ID       = 0x0001,
  REAL_BLOOD_DATA_ID = 0x0002,
  REAL_SHOOT_DATA_ID = 0x0003,
  REAL_POWER_DATA_ID = 0x0004,
  REAL_FIELD_DATA_ID = 0x0005,
  GAME_RESULT_ID     = 0x0006,
  GAIN_BUFF_ID       = 0x0007,
  ROBOT_POS_DATA_ID  = 0x0008,
  
  STU_CUSTOM_DATA_ID = 0x0100,
  ROBOT_TO_CLIENT_ID = 0x0101,
  CLIENT_TO_ROBOT_ID = 0x0102,
} referee_data_id_e;

#pragma pack(push,1)

typedef struct
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} frame_header_t;

#pragma pack(pop)

#define PROTOCAL_FRAME_MAX_SIZE  200

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct
{
  fifo_s_t       *data_fifo;
  frame_header_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[PROTOCAL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

void referee_param_init(void);
void referee_unpack_fifo_data(void);

uint8_t     ext_get_crc8(uint8_t *p_msg, uint32_t len, uint8_t crc8) ;
uint32_t    ext_verify_crc8(uint8_t *p_msg, uint32_t len);
void        ext_append_crc8(uint8_t *p_msg, uint32_t len);
uint16_t    ext_get_crc16(uint8_t *p_msg, uint16_t len, uint16_t crc16);
uint32_t    ext_verify_crc16(uint8_t *p_msg, uint16_t len);
void        ext_append_crc16(uint8_t* p_msg, uint32_t len) ;
#endif // __REFEREE_SYSTEM_H__
