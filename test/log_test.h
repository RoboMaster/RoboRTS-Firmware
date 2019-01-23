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

#ifndef __LOG_TEST_H__
#define __LOG_TEST_H__

#ifdef LOG_TEST_H_GLOBAL
  #define LOG_TEST_H_EXTERN 
#else
  #define LOG_TEST_H_EXTERN extern
#endif

#include "ulog.h"
#include "drv_uart.h"
#include "test_module.h"

void log_test_init(void);

#endif // LOG_TEST_H__
