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

#ifndef __OBJECT_H__
#define __OBJECT_H__

#ifdef OBJECT_H_GLOBAL
  #define OBJECT_H_EXTERN
#else
  #define OBJECT_H_EXTERN extern
#endif

#include "sys.h"

//This macro must be greater than 16.
#define OBJECT_NAME_MAX_LEN 32

#if OBJECT_NAME_MAX_LEN < 16
  #error "Macro OBJECT_NAME_MAX_LEN must be greater than 16."
#endif

enum object_class_type
{
  Object_Class_Device = 0,
  Object_Class_Module = 1,
  Object_Class_Controller = 2,
  Object_Class_Chassis = 3,
  Object_Class_Gimbal = 4,
  Object_Class_Shoot = 5,
  Object_Class_Unknown,
};

struct object
{
  char name[OBJECT_NAME_MAX_LEN];
  enum object_class_type type;
  uint8_t flag;
  list_t list;
};

typedef struct object *object_t;

struct object_information
{
  enum object_class_type type; /**< object class type */
  list_t object_list;          /**< object list */
};

object_t object_find(const char *name, enum object_class_type type);
int32_t object_init(struct object *object,
                    enum object_class_type type,
                    const char *name);
void object_detach(object_t object);
struct object_information *
object_get_information(enum object_class_type type);

#endif // __OBJECT_H__
