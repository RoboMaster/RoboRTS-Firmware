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

#include "motor.h"

#define MAX_MOTOR_NUM 6

static void get_encoder_data(motor_device_t motor, uint8_t can_rx_data[]);
static void get_motor_offset(motor_data_t ptr, uint8_t can_rx_data[]);

static fn_can_send motor_can_send = NULL;

int32_t motor_device_register(motor_device_t motor_dev,
                              const char *name,
                              uint16_t flags)
{
  if (motor_dev == NULL)
    return -RM_INVAL;

  if (device_find(name) != NULL)
    return -RM_EXISTED;

  if (motor_device_find_by_canid(motor_dev->can_periph, motor_dev->can_id) != NULL)
    return -RM_EXISTED;

  if ((motor_dev->can_id < 0x201) && (motor_dev->can_id > 0x208))
    return -RM_ERROR;

  motor_dev->parent.type = Device_Class_Motor;
  motor_dev->get_data = get_encoder_data;

  device_register( &(motor_dev->parent), name, flags);

  return RM_OK;
}

void motor_device_can_send_register(fn_can_send fn)
{
  if (fn != NULL)
    motor_can_send = fn;
}

motor_device_t motor_device_find(const char *name)
{
  device_t dev;
  enum device_type type;

  dev = device_find(name);
  
  if(dev == NULL) 
    return NULL;

  type = dev->type;

  if (type == Device_Class_Motor)
  {
    return (motor_device_t)dev;
  }
  else
  {
    return NULL;
  }
}

motor_data_t motor_device_get_data(motor_device_t motor_dev)
{
  if (motor_dev != NULL)
  {
    return &(motor_dev->data);
  }
  return NULL;
}

int32_t motor_device_set_current(motor_device_t motor_dev, int16_t current)
{

  if (motor_dev != NULL)
  {
    motor_dev->current = current;
    return RM_OK;
  }
  return -RM_ERROR;
}

motor_device_t motor_device_find_by_canid(enum device_can can, uint16_t can_id)
{
  struct object *object;
  list_t *node = NULL;
  struct object_information *information;
  enum device_type type;

  var_cpu_sr();
  
  /* enter critical */
  enter_critical();

  /* try to find device object */
  information = object_get_information(Object_Class_Device);

  for (node = information->object_list.next;
       node != &(information->object_list);
       node = node->next)
  {
    object = list_entry(node, struct object, list);

    type = (enum device_type)(((device_t)object)->type);

    if (type != Device_Class_Motor)
    {
      continue;
    }  
    else if ((((motor_device_t)object)->can_id == can_id) && (((motor_device_t)object)->can_periph == can))
    {
      /* leave critical */
      exit_critical();
      return (motor_device_t)object;
    }
  }

  /* leave critical */
  exit_critical();

  /* not found */
  return NULL;
}

static uint8_t motor_send_flag[DEVICE_CAN_NUM][2];
static struct can_msg motor_msg[DEVICE_CAN_NUM][2];

int32_t motor_device_can_output(enum device_can m_can)
{
  struct object *object;
  list_t *node = NULL;
  struct object_information *information;
  motor_device_t motor_dev;

  memset(motor_msg, 0, sizeof(motor_msg));

  var_cpu_sr();
   
  /* enter critical */
  enter_critical();

  /* try to find device object */
  information = object_get_information(Object_Class_Device);

  for (node = information->object_list.next;
       node != &(information->object_list);
       node = node->next)
  {
    object = list_entry(node, struct object, list);
    motor_dev = (motor_device_t)object;
    if(motor_dev->parent.type == Device_Class_Motor)
    {
      if (((motor_device_t)object)->can_id < 0x205)
      {
        motor_msg[motor_dev->can_periph][0].id = 0x200;
        motor_msg[motor_dev->can_periph][0].data[(motor_dev->can_id - 0x201) * 2] = motor_dev->current >> 8;
        motor_msg[motor_dev->can_periph][0].data[(motor_dev->can_id - 0x201) * 2 + 1] = motor_dev->current;
        motor_send_flag[motor_dev->can_periph][0] = 1;
      }
      else
      {
        motor_msg[motor_dev->can_periph][1].id = 0x1FF;
        motor_msg[motor_dev->can_periph][1].data[(motor_dev->can_id - 0x205) * 2] = motor_dev->current >> 8;
        motor_msg[motor_dev->can_periph][1].data[(motor_dev->can_id - 0x205) * 2 + 1] = motor_dev->current;
        motor_send_flag[motor_dev->can_periph][1] = 1;
      }
    }
  }

  /* leave critical */
  exit_critical();
  
  for (int j = 0; j < 2; j++)
  {
    if (motor_send_flag[m_can][j] == 1)
    {
      if (motor_can_send != NULL)
        motor_can_send(m_can, motor_msg[m_can][j]);
      motor_send_flag[m_can][j] = 0;
    }
  }

  /* not found */
  return RM_OK;
}

int32_t motor_device_data_update(enum device_can can, uint16_t can_id, uint8_t can_rx_data[])
{
  motor_device_t motor_dev;
  motor_dev = motor_device_find_by_canid(can, can_id);
  if (motor_dev != NULL)
  {
    motor_dev->get_data(motor_dev, can_rx_data);
    return RM_OK;
  }
  return -RM_UNREGISTERED;
}

static void get_encoder_data(motor_device_t motor, uint8_t can_rx_data[])
{
  motor_data_t ptr = &(motor->data);
  ptr->msg_cnt++;

  if (ptr->msg_cnt > 50)
  {
    motor->init_offset_f = 0;
  }

  if (motor->init_offset_f == 1)
  {
    get_motor_offset(ptr, can_rx_data);
    return;
  }

  ptr->last_ecd = ptr->ecd;
  ptr->ecd = (uint16_t)(can_rx_data[0] << 8 | can_rx_data[1]);

  if (ptr->ecd - ptr->last_ecd > 4096)
  {
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
  }
  else if (ptr->ecd - ptr->last_ecd < -4096)
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }

  ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
  /* total angle, unit is degree */
  ptr->total_angle = ptr->total_ecd / ENCODER_ANGLE_RATIO;

  ptr->speed_rpm = (int16_t)(can_rx_data[2] << 8 | can_rx_data[3]);
  ptr->given_current = (int16_t)(can_rx_data[4] << 8 | can_rx_data[5]);
}

static void get_motor_offset(motor_data_t ptr, uint8_t can_rx_data[])
{
  ptr->ecd = (uint16_t)(can_rx_data[0] << 8 | can_rx_data[1]);
  ptr->offset_ecd = ptr->ecd;
}
