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

#ifndef __DRV_FLASH_H__
#define __DRV_FLASH_H__

#include "stm32f4xx_hal.h"

#define PARAM_SAVED_START_ADDRESS ADDR_FLASH_SECTOR_9 //last sector of flash 2MB flash
#define FLASH_USER_START_ADDR ADDR_FLASH_SECTOR_2      /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR ADDR_FLASH_SECTOR_9       /* End @ of user Flash area */

uint8_t BSP_FLASH_Write(uint8_t *pbuff, uint32_t len);
uint32_t GetSector(uint32_t Address);

#endif // __DRV_FLASH_H__
