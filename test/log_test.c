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

#include "log_test.h"

int ulog_console_backend_init(void);

void log_test(void);

void log_test_init(void)
{
  ulog_init();
//  ulog_tag_lvl_filter_set("usart6", 7);
  ulog_console_backend_init();
  test_module_register((void *)log_test, NULL);
}

void log_test(void)
{
  static int cnt;
  if (cnt % 500 == 0)  // 1 second
  {
      LOG_E("This is a E log!");
      LOG_W("This is a W log!");
      LOG_I("This is a I log!");
      LOG_D("This is a D log!");
      LOG_RAW("This is a raw log!\r\n");
  }
  cnt++;
}

