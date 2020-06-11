
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

#ifndef __ERRNO_H__
#define __ERRNO_H__

#ifndef NULL
    #define NULL ((void *)0)
#endif

#ifndef E_OK
    #define  E_OK              0
#endif

#define E_ERROR             1
#define E_INVAL             2
#define E_EXISTED           3
#define E_UNREGISTERED      4
#define E_NOSTATE           5
#define E_USED              6
#define E_NOMEM             7

#endif // __ERRNO_H__
