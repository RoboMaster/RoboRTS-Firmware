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

#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

#ifdef PID_CONTROLLER_H_GLOBAL
  #define PID_CONTROLLER_H_EXTERN 
#else
  #define PID_CONTROLLER_H_EXTERN extern
#endif

#include "pid.h"
#include "controller.h"

typedef struct pid_feedback *pid_feedback_t;

struct pid_feedback
{
  float feedback;
};

typedef struct cascade *cascade_t;

struct cascade
{
  struct pid outer;
  struct pid inter;
};

typedef struct cascade_feedback *cascade_feedback_t;

struct cascade_feedback
{
  float outer_fdb;
  float inter_fdb;
};

int32_t pid_control(struct controller *ctrl, void *param, void *feedback, float input);
int32_t cascade_control(struct controller *ctrl, void *param, void *feedback, float input);
int32_t pid_controller_register(struct controller *ctrl,
                                const char *name,
                                struct pid *param,
                                struct pid_feedback *input,
                                uint8_t enable);
int32_t cascade_controller_register(struct controller *ctrl,
                                    const char *name,
                                    struct cascade *param,
                                    struct cascade_feedback *input,
                                    uint8_t enable);
#endif // __PID_CONTROLLER_H__
