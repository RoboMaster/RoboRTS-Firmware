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

#ifndef __IST8310_REG_H__
#define __IST8310_REG_H__

/* IST8310 internal reg addr */
#define IST8310_ADDRESS     0x0E
#define IST8310_DEVICE_ID_A 0x10

/* IST8310 register map */
#define IST8310_WHO_AM_I    0x00
#define IST8310_R_CONFA     0x0A
#define IST8310_R_CONFB     0x0B
#define IST8310_R_MODE      0x02

#define IST8310_R_XL        0x03
#define IST8310_R_XM        0x04
#define IST8310_R_YL        0x05
#define IST8310_R_YM        0x06
#define IST8310_R_ZL        0x07
#define IST8310_R_ZM        0x08

#define IST8310_AVGCNTL     0x41
#define IST8310_PDCNTL      0x42

/* sigle measure mode */
#define IST8310_ODR_MODE    0x01 

#endif // __IST8310_REG_H__
