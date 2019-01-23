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

#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#ifdef CONTROLLER_H_GLOBAL
#define CONTROLLER_H_EXTERN
#else
#define CONTROLLER_H_EXTERN extern
#endif

#include "object.h"

typedef struct controller *controller_t;

enum controller_type
{
  Controller_Class_PID = 0,
  Controller_Class_Cascade = 1,
  Controller_Class_Unknown,
};

struct controller
{
  struct object parent;
  enum controller_type type;
  uint8_t enable;
  void *param;
  void *feedback;
  float input;
  float output;
  int32_t (*convert_feedback)(struct controller *ctrl, void *feedback);
  int32_t (*control)(struct controller *ctrl, void *param, void *feedback, float input);
};

int32_t controller_register(struct controller *ctrl,
                            const char *name,
                            enum controller_type type,
                            void *param,
                            void *feedback,
                            uint8_t enable);
controller_t controller_find(const char *name);
int32_t controller_execute(struct controller *ctrl, void *input);
int32_t controller_set_input(struct controller *ctrl, float input);
float controller_get_output(struct controller *ctrl, float *out);
enum controller_type controller_get_type(struct controller *ctrl);
int32_t controller_enable(struct controller *ctrl);
int32_t controller_disable(struct controller *ctrl);

#endif // __CONTROLLER_H__
