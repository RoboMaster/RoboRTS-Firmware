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

#include "test_module.h"

#define MAX_TEST_FN_NUM 30

static test_module_fn_t test_module_fn[MAX_TEST_FN_NUM];
static void *test_module_argc[MAX_TEST_FN_NUM];

int32_t test_module_register(void *fn, void *argc)
{
  for (int i = 0; i < MAX_TEST_FN_NUM; i++)
  {
    if (test_module_fn[i] == NULL)
    {
      test_module_fn[i] = (test_module_fn_t)fn;
      test_module_argc[i] = argc;
      return i;
    }
  }
  return -1;
}

int32_t test_module_unregister(uint32_t id)
{
  if (id < MAX_TEST_FN_NUM)
  {
    test_module_fn[id] = NULL;
    return 0;
  }
  return -1;
}

int32_t test_module_execute(void)
{
  for (int i = 0; i < MAX_TEST_FN_NUM; i++)
  {
    if (test_module_fn[i] != NULL)
    {
      test_module_fn[i](test_module_argc[i]);
    }
  }
  return 0;
}
