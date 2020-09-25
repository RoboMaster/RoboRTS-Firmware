/*******************************************************************************
*                      CRC16  functions algorithm  liboray                     *
*   file : mf_crc.h                                                            *
*                                                                              *
*   history :                                                                  *
*     v1.0 2010-07-07   Motorfeng                                              *
*******************************************************************************/
#ifndef  __MF_CRC__
#define  __MF_CRC__

#include <stdint.h>
#include <stddef.h>

#ifndef NULL
    #define NULL    ((void *)0)
#endif

#ifndef __FALSE
    #define __FALSE   (0)
#endif

#ifndef __TRUE
    #define __TRUE    (1)
#endif

uint8_t     get_crc8(uint8_t *pchMessage, uint32_t dwLength, uint8_t ucCRC8);
uint32_t    verify_crc8(uint8_t *pchMessage, uint32_t dwLength);
void        append_crc8(uint8_t *pchMessage, uint32_t dwLength);
uint16_t    get_crc16(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t    verify_crc16(uint8_t *pchMessage, uint32_t dwLength);
void        append_crc16(uint8_t *pchMessage, uint32_t dwLength);
uint32_t    get_crc32(uint8_t *pchMessage, uint32_t dwLength, uint32_t wCRC);
uint32_t    verify_crc32(uint8_t *pchMessage, uint32_t dwLength);
void        append_crc32(uint8_t *pchMessage, uint32_t dwLength);

#endif
/*
********************************************************************************
*                        END
********************************************************************************
*/
