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

#include "shoot.h"
#include "drv_io.h"

static int32_t shoot_pid_input_convert(struct controller *ctrl, void *input);
static int32_t shoot_fric_ctrl(struct shoot *shoot);
static int32_t shoot_cmd_ctrl(struct shoot *shoot);
static int32_t shoot_block_check(struct shoot *shoot);

int32_t shoot_pid_register(struct shoot *shoot, const char *name, enum device_can can)
{
  char motor_name[OBJECT_NAME_MAX_LEN] = {0};
  uint8_t name_len;

  int32_t err;

  if (shoot == NULL)
    return -RM_INVAL;
  if (shoot_find(name) != NULL)
    return -RM_EXISTED;

  object_init(&(shoot->parent), Object_Class_Shoot, name);

  name_len = strlen(name);

  if (name_len > OBJECT_NAME_MAX_LEN / 2)
  {
    name_len = OBJECT_NAME_MAX_LEN / 2;
  }

  memcpy(&motor_name, name, name_len);
  shoot->motor.can_periph = can;
  shoot->motor.can_id = 0x207;
  shoot->motor.init_offset_f = 1;

  shoot->ctrl.convert_feedback = shoot_pid_input_convert;

  pid_struct_init(&(shoot->motor_pid), 30000, 10000, 10, 0.1, 7);

  shoot->param.block_current = BLOCK_CURRENT_DEFAULT;
  shoot->param.block_speed = BLOCK_SPEED_DEFAULT;
  shoot->param.block_timeout = BLOCK_TIMEOUT_DEFAULT;
  shoot->param.turn_speed = TURN_SPEED_DEFAULT;
  shoot->param.check_timeout = BLOCK_CHECK_TIMEOUT_DEFAULT;

  memcpy(&motor_name[name_len], "_TURN\0", 6);

  err = motor_device_register(&(shoot->motor), motor_name, 0);
  if (err != RM_OK)
    goto end;

  memcpy(&motor_name[name_len], "_CTL\0", 7);

  err = pid_controller_register(&(shoot->ctrl), motor_name, &(shoot->motor_pid), &(shoot->motor_feedback), 1);
  if (err != RM_OK)
    goto end;

  shoot_state_update(shoot);

  return RM_OK;
end:
  object_detach(&(shoot->parent));

  return err;
}

int32_t shoot_set_fric_speed(struct shoot *shoot, uint16_t fric_spd1, uint16_t fric_spd2)
{
  if (shoot == NULL)
    return -RM_INVAL;
  shoot->target.fric_spd[0] = fric_spd1;
  shoot->target.fric_spd[1] = fric_spd2;

  return RM_OK;
}

int32_t shoot_get_fric_speed(struct shoot *shoot, uint16_t *fric_spd1, uint16_t *fric_spd2)
{
  if (shoot == NULL)
    return -RM_INVAL;
  fric_get_speed(fric_spd1, fric_spd2);
  return RM_OK;
}

int32_t shoot_set_cmd(struct shoot *shoot, uint8_t cmd, uint32_t shoot_num)
{
  if (shoot == NULL)
    return -RM_INVAL;

  shoot->cmd = cmd;

  if (cmd == SHOOT_ONCE_CMD)
  {
    shoot->target.shoot_num = shoot->shoot_num + shoot_num;
  }

  return RM_OK;
}

int32_t shoot_execute(struct shoot *shoot)
{
  float motor_out;
  struct motor_data *pdata;

  if (shoot == NULL)
    return -RM_INVAL;

  shoot_fric_ctrl(shoot);
  shoot_block_check(shoot);
  shoot_cmd_ctrl(shoot);

  pdata = motor_device_get_data(&(shoot->motor));

  controller_set_input(&(shoot->ctrl), shoot->target.motor_speed);
  controller_execute(&(shoot->ctrl), (void *)pdata);
  controller_get_output(&(shoot->ctrl), &motor_out);

  motor_device_set_current(&shoot->motor, (int16_t)motor_out);

  return RM_OK;
}

int32_t shoot_state_update(struct shoot *shoot)
{
  if (shoot == NULL)
    return -RM_INVAL;

  shoot->trigger_key = get_trig_status();
  if (shoot->trigger_key == TRIG_PRESS_DOWN)
  {
    shoot->target.motor_speed = 0;
    shoot->state = SHOOT_READY;
  }
  else if (shoot->trigger_key == TRIG_BOUNCE_UP)
  {
    shoot->target.motor_speed = shoot->param.turn_speed;
    shoot->state = SHOOT_INIT;
    if (shoot->cmd == SHOOT_ONCE_CMD)
    {
      shoot->shoot_num++;
      shoot->cmd = SHOOT_STOP_CMD;
    }
  }
  return RM_OK;
}

int32_t shoot_set_turn_speed(struct shoot *shoot, uint16_t speed)
{
  if (shoot == NULL)
    return -RM_INVAL;
  
  VAL_LIMIT(speed, 1000, 2500);

  shoot->param.turn_speed = speed;

  return RM_OK;
}

shoot_t shoot_find(const char *name)
{
  struct object *object;

  object = object_find(name, Object_Class_Shoot);

  return (shoot_t)object;
}

int32_t shoot_enable(struct shoot *shoot)
{
  if (shoot == NULL)
    return -RM_INVAL;

  controller_enable(&(shoot->ctrl));

  return RM_OK;
}

int32_t shoot_disable(struct shoot *shoot)
{
  if (shoot == NULL)
    return -RM_INVAL;
  shoot_set_fric_speed(shoot, 0, 0);
  controller_disable(&(shoot->ctrl));

  return RM_OK;
}

static int32_t shoot_block_check(struct shoot *shoot)
{
  static uint8_t first_block_f = 0;
  static uint32_t check_time;
  
  if (shoot == NULL)
    return -RM_INVAL;

  if (shoot->motor.current > shoot->param.block_current)
  {
    if (first_block_f == 0)
    {
      first_block_f = 1;
      check_time = get_time_ms();
    }
    else if(get_time_ms() - check_time > shoot->param.check_timeout)
    {
      first_block_f = 0;
      shoot->block_time = get_time_ms();
      shoot->state = SHOOT_BLOCK;
    }
  }
  else
  {
    first_block_f = 0;
  }
  
  return RM_OK;
}

static int32_t shoot_cmd_ctrl(struct shoot *shoot)
{
  if (shoot == NULL)
    return -RM_INVAL;

  if (shoot->state == SHOOT_INIT)
  {
    shoot->target.motor_speed = shoot->param.turn_speed;
  }
  else if (shoot->state == SHOOT_READY)
  {
    if ((shoot->fric_spd[0] >= FRIC_MIN_SPEED) && (shoot->fric_spd[1] >= FRIC_MIN_SPEED))
    {
      switch (shoot->cmd)
      {
      case SHOOT_ONCE_CMD:
      case SHOOT_CONTINUOUS_CMD:
      {
        shoot->target.motor_speed = shoot->param.turn_speed;
      }
      break;
      case SHOOT_STOP_CMD:
      {
        if (shoot->shoot_num < shoot->target.shoot_num)
          shoot->cmd = SHOOT_ONCE_CMD;
      }
      break;
      default:
        break;
      }
    }
    else
      shoot->cmd = SHOOT_STOP_CMD;
  }
  else if (shoot->state == SHOOT_BLOCK)
  {
    shoot->target.motor_speed = shoot->param.block_speed;
    if (get_time_ms() - shoot->block_time > shoot->param.block_timeout)
    {
      shoot_state_update(shoot);
    }
  }
 
	if ((shoot->fric_spd[0] >= FRIC_MIN_SPEED) && (shoot->fric_spd[1] >= FRIC_MIN_SPEED))
	{
		controller_enable(&(shoot->ctrl));
	}
	else
	{
		controller_disable(&(shoot->ctrl));
	}
	
  return RM_OK;
}

static int32_t shoot_fric_ctrl(struct shoot *shoot)
{
  if (shoot == NULL)
    return -RM_INVAL;

  VAL_LIMIT(shoot->target.fric_spd[0], FIRC_STOP_SPEED, FIRC_MAX_SPEED);
  VAL_LIMIT(shoot->target.fric_spd[1], FIRC_STOP_SPEED, FIRC_MAX_SPEED);

  shoot_get_fric_speed(shoot, &(shoot->fric_spd[0]), &(shoot->fric_spd[1]));

  if (shoot->target.fric_spd[0] != shoot->fric_spd[0])
  {
    if (shoot->target.fric_spd[0] < shoot->fric_spd[0])
    {
      shoot->fric_spd[0] -= 1;
    }
    else
    {
      shoot->fric_spd[0] += 1;
    }
  }
  else if (shoot->target.fric_spd[1] != shoot->fric_spd[1])
  {
    if (shoot->target.fric_spd[1] < shoot->fric_spd[1])
    {
      shoot->fric_spd[1] -= 1;
    }
    else
    {
      shoot->fric_spd[1] += 1;
    }
  }

  fric_set_output(shoot->fric_spd[0], shoot->fric_spd[1]);

  return RM_OK;
}

static int32_t shoot_pid_input_convert(struct controller *ctrl, void *input)
{
  pid_feedback_t pid_fdb = (pid_feedback_t)(ctrl->feedback);
  motor_data_t data = (motor_data_t)input;
  pid_fdb->feedback = data->speed_rpm;

  return RM_OK;
}
