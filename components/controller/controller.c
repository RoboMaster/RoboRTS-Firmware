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

#include "errno.h"
#include "controller.h"

//this is a single input and single output loop controller

int32_t controller_register(struct controller *ctrl,
                            const char *name,
                            enum controller_type type,
                            void *param,
                            void *feedback,
                            uint8_t enable)
{
  if (ctrl == NULL)
    return -RM_INVAL;
  if (controller_find(name) != NULL)
    return -RM_EXISTED;

  object_init(&(ctrl->parent), Object_Class_Controller, name);

  ctrl->type = type;
  ctrl->enable = enable;
  ctrl->param = param;
  ctrl->feedback = feedback;

  return RM_OK;
}

int32_t controller_unregister(struct controller *ctrl)
{
  if (ctrl == NULL)
    return -RM_INVAL;
  if (controller_find(((object_t)ctrl)->name) == NULL)
    return RM_OK;

  object_detach(&(ctrl->parent));

  return RM_OK;
}

controller_t controller_find(const char *name)
{
  struct object *object;

  object = object_find(name, Object_Class_Controller);

  return (controller_t)object;
}

int32_t controller_set_param(struct controller *ctrl, void *param)
{
  if (ctrl == NULL)
    return -RM_INVAL;

  ctrl->param = param;

  return RM_OK;
}

int32_t controller_execute(struct controller *ctrl, void *feedback)
{
  if (ctrl == NULL)
    return -RM_INVAL;

  if (feedback == NULL)
    return -RM_INVAL;

  if (ctrl->convert_feedback == NULL)
  {
    return -RM_INVAL;
  }
  else
  {
    ctrl->convert_feedback(ctrl, feedback);
  }

  if ((ctrl->control != NULL) && (ctrl->enable == 1))
  {
    ctrl->control(ctrl, ctrl->param, ctrl->feedback, ctrl->input);
  }

  return RM_OK;
}

int32_t controller_set_input(struct controller *ctrl, float input)
{
  if (ctrl == NULL)
    return 0;
  ctrl->input = input;
  return RM_OK;
}

float controller_get_output(struct controller *ctrl, float *out)
{
  if (ctrl == NULL)
    return 0;
  *out = ctrl->output;
  return ctrl->output;
}

enum controller_type controller_get_type(struct controller *ctrl)
{
  if (ctrl == NULL)
    return Controller_Class_Unknown;

  return ctrl->type;
}

int32_t controller_enable(struct controller *ctrl)
{
  if (ctrl == NULL)
    return RM_INVAL;
  ctrl->enable = 1;
  return RM_OK;
}

int32_t controller_disable(struct controller *ctrl)
{
  if (ctrl == NULL)
    return RM_INVAL;
  ctrl->enable = 0;
  ctrl->output = 0;
  ctrl->input = 0;
  return RM_OK;
}
