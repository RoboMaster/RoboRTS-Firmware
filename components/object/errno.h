
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

#ifndef __ERRNO_H__
#define __ERRNO_H__

#ifdef ERRNO_H_GLOBAL
  #define ERRNO_H_EXTERN 
#else
  #define ERRNO_H_EXTERN extern
#endif

#ifndef NULL
  #define NULL ((void *)0)
#endif

#ifndef RM_OK
#define  RM_OK              0
#endif

#define RM_ERROR             1
#define RM_INVAL             2  
#define RM_EXISTED           3  
#define RM_UNREGISTERED      4 
#define RM_NOSTATE           5
#define RM_USED              6
#define RM_NOMEM             7

#endif // __ERRNO_H__
