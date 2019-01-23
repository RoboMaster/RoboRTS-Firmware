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

#ifndef __TEST_MODULE_H__
#define __TEST_MODULE_H__

#ifdef TEST_MODULE_H_GLOBAL
  #define TEST_MODULE_H_EXTERN 
#else
  #define TEST_MODULE_H_EXTERN extern
#endif

#include "sys.h"

typedef void (*test_module_fn_t)(void* argc);

int32_t test_module_register(void *fn, void * argc);
int32_t test_module_unregister(uint32_t id);
int32_t test_module_execute(void);

#endif // __TEST_MODULE_H__
