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

#include "pid_controller.h"
#include "errno.h"

int32_t pid_controller_register(struct controller *ctrl,
                                const char *name,
                                struct pid *param,
                                struct pid_feedback *feedback,
                                uint8_t enable)
{
  if (ctrl == NULL)
    return -RM_INVAL;
  
  ctrl->control = pid_control;
  controller_register(ctrl, name, Controller_Class_PID, (void *)param, (void *)feedback, enable);

  return RM_OK;
}              

int32_t pid_control(struct controller *ctrl, void *param, void *feedback, float input)
{
  pid_t pid_param = (pid_t)param;
  pid_feedback_t pid_feedback = (pid_feedback_t)feedback;
  
  pid_calculate(pid_param, pid_feedback->feedback, input);
  
  ctrl->output = pid_param->out;

  return RM_OK; 
}

int32_t cascade_controller_register(struct controller *ctrl,
                                    const char *name,
                                    struct cascade *param,
                                    struct cascade_feedback *feedback,
                                    uint8_t enable)
{
  if (ctrl == NULL)
    return -RM_INVAL;
  
  ctrl->control = cascade_control;
  controller_register(ctrl, name, Controller_Class_Cascade, (void *)param, (void *)feedback, enable);

  return RM_OK;
}

int32_t cascade_control(struct controller *ctrl, void *param, void *feedback, float input)
{
  cascade_t cascade_param = (cascade_t)param;
  cascade_feedback_t cascade_input = (cascade_feedback_t)feedback;

  pid_calculate(&(cascade_param->outer), cascade_input->outer_fdb, input);
  pid_calculate(&(cascade_param->inter), cascade_input->inter_fdb, cascade_param->outer.out);

  ctrl->output = cascade_param->inter.out;

  return RM_OK;
}
