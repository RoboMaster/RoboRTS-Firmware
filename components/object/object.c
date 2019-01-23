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
#include "object.h"

enum object_info_type
{
  Object_Info_Device = 0,
  Object_Info_Module = 1,
  Object_Info_Contoller = 2,
  Object_Info_Chassis = 3,
  Object_Info_Gimbal = 4,
  Object_Info_Shoot = 5,
  Object_Info_Unknown, /**< The object is unknown. */
};

#define _OBJ_CONTAINER_LIST_INIT(c)                                        \
  {                                                                        \
    &(object_container[c].object_list), &(object_container[c].object_list) \
  }
static struct object_information object_container[Object_Info_Unknown] =
    {
        /* initialize object container - Device */
        {Object_Class_Device, _OBJ_CONTAINER_LIST_INIT(Object_Info_Device)},
        /* initialize object container - Module */
        {Object_Class_Module, _OBJ_CONTAINER_LIST_INIT(Object_Info_Module)},
        /* initialize object container - Controller */
        {Object_Class_Controller, _OBJ_CONTAINER_LIST_INIT(Object_Info_Contoller)},
        /* initialize object container - Controller */
        {Object_Class_Chassis, _OBJ_CONTAINER_LIST_INIT(Object_Info_Chassis)},
        /* initialize object container - Controller */
        {Object_Class_Gimbal, _OBJ_CONTAINER_LIST_INIT(Object_Info_Gimbal)},
        /* initialize object container - Controller */
        {Object_Class_Shoot, _OBJ_CONTAINER_LIST_INIT(Object_Info_Shoot)},
};

struct object_information *
object_get_information(enum object_class_type type)
{
  int index;

  for (index = 0; index < Object_Info_Unknown; index++)
    if (object_container[index].type == type)
      return &object_container[index];

  return NULL;
}

int32_t object_init(struct object *object,
                    enum object_class_type type,
                    const char *name)
{
  struct object_information *information;

  var_cpu_sr();

  /* get object information */
  information = object_get_information(type);
  assert_param_obj(information != NULL);

  /* initialize object's parameters */
  object->type = type;

  /* copy name */
  if (strlen(name) > OBJECT_NAME_MAX_LEN - 1)
  {
    return -1;
  }

  strncpy(object->name, name, OBJECT_NAME_MAX_LEN);
  object->name[OBJECT_NAME_MAX_LEN - 1] = '\0';

  /* lock interrupt */
  enter_critical();

  {
    /* insert object into information object list */
    list_add(&(object->list), &(information->object_list));
  }

  /* unlock interrupt */
  exit_critical();
  return 0;
}

object_t object_find(const char *name, enum object_class_type type)
{
  struct object *object = NULL;
  struct object_information *information = NULL;
  list_t *node = NULL;

  var_cpu_sr();

  /* parameter check */
  if ((name == NULL) || (type >= Object_Class_Unknown))
    return NULL;

  /* enter critical */
  enter_critical();

  /* try to find object */
  if (information == NULL)
  {
    information = object_get_information(type);
    assert_param_obj(information != NULL);
  }
  for (node = information->object_list.next;
       node != &(information->object_list);
       node = node->next)
  {
    object = list_entry(node, struct object, list);
    if (strncmp(object->name, name, OBJECT_NAME_MAX_LEN) == 0)
    {
      /* leave critical */
      exit_critical();

      return object;
    }
  }

  /* leave critical */
  exit_critical();

  return NULL;
}

void object_detach(object_t object)
{
  var_cpu_sr();

  /* object check */
  assert_param_obj(object != NULL);

  /* reset object type */
  object->type = Object_Class_Unknown;

  /* lock interrupt */
  enter_critical();

  /* remove from old list */
  list_del(&(object->list));

  /* unlock interrupt */
  exit_critical();
}
