/****************************************************************************
 *  Copyright (C) 2020 RoboMaster.
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

#define LOG_TAG "shoot"
#include "log.h"

static int32_t shoot_fric_ctrl(struct shoot *shoot);
static int32_t shoot_cmd_ctrl(struct shoot *shoot);
static int32_t shoot_block_check(struct shoot *shoot);

/**
  * @brief     turn motor pid init
  * @param[in]
  * @retval    error code
  */
int32_t shoot_pid_init(struct shoot *shoot, const char *name, struct pid_param param, enum device_can can)
{
    char motor_name[OBJECT_NAME_MAX_LEN] = {0};
    uint8_t name_len;

    int32_t err;

    device_assert(shoot != NULL);

    name_len = strlen(name);

    memcpy(&motor_name, name, name_len);
    shoot->motor.can_periph = can;
    shoot->motor.can_id = 0x207;
    shoot->motor.init_offset_f = 1;

    pid_struct_init(&(shoot->motor_pid), param.max_out, param.integral_limit, param.p, param.i, param.d);

    shoot->param.block_current = BLOCK_CURRENT_DEFAULT;
    shoot->param.block_speed = BLOCK_SPEED_DEFAULT;
    shoot->param.block_timeout = BLOCK_TIMEOUT_DEFAULT;
    shoot->param.turn_speed = TURN_SPEED_DEFAULT;
    shoot->param.check_timeout = BLOCK_CHECK_TIMEOUT_DEFAULT;

    memcpy(&motor_name[name_len], "_TURN\0", 6);

    err = motor_register(&(shoot->motor), motor_name);
    if (err != E_OK)
    {
        goto end;
    }

    memcpy(&motor_name[name_len], "_CTL\0", 7);

    shoot_state_update(shoot);

    return E_OK;
end:

    return err;
}

/**
  * @brief     set friction wheel speed
  * @param[in]
  * @retval    error code
  */
int32_t shoot_set_fric_speed(struct shoot *shoot, uint16_t fric_spd1, uint16_t fric_spd2)
{
    device_assert(shoot != NULL);

    shoot->target.fric_spd[0] = fric_spd1;
    shoot->target.fric_spd[1] = fric_spd2;

    return E_OK;
}

int32_t shoot_get_fric_speed(struct shoot *shoot, uint16_t *fric_spd1, uint16_t *fric_spd2)
{
    device_assert(shoot != NULL);

    fric_get_speed(fric_spd1, fric_spd2);
    return E_OK;
}

/**
  * @brief     set shoot num
  * @param[in]
  * @retval    error code
  */
int32_t shoot_set_cmd(struct shoot *shoot, uint8_t cmd, uint32_t shoot_num)
{
    device_assert(shoot != NULL);

    shoot->cmd = cmd;

    if (cmd == SHOOT_ONCE_CMD)
    {
        shoot->target.shoot_num = shoot->shoot_num + shoot_num;
    }

    return E_OK;
}

int32_t shoot_pid_calculate(struct shoot *shoot)
{
    float motor_out;

    device_assert(shoot != NULL);

    shoot_fric_ctrl(shoot);
    shoot_block_check(shoot);
    shoot_cmd_ctrl(shoot);

    motor_out = pid_calculate(&shoot->motor_pid, shoot->motor.data.speed_rpm, shoot->target.motor_speed);
    motor_set_current(&shoot->motor, (int16_t)motor_out);

    return E_OK;
}

/**
  * @brief     update shoot status by interupt
  * @param[in]
  * @retval    error code
  */
int32_t shoot_state_update(struct shoot *shoot)
{
    device_assert(shoot != NULL);

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
    return E_OK;
}

int32_t shoot_set_turn_speed(struct shoot *shoot, uint16_t speed)
{
    device_assert(shoot != NULL);

    VAL_LIMIT(speed, 1000, 2500);

    shoot->param.turn_speed = speed;

    return E_OK;
}

int32_t shoot_enable(struct shoot *shoot)
{
    device_assert(shoot != NULL);

    shoot->motor_pid.enable = 1;

    return E_OK;
}

int32_t shoot_disable(struct shoot *shoot)
{
    device_assert(shoot != NULL);

    shoot_set_fric_speed(shoot, 0, 0);

    shoot->motor_pid.enable = 0;

    shoot->target.shoot_num = shoot->shoot_num;

    return E_OK;
}

/**
  * @brief     when motor current more than a value, change motor turn arround.
  * @param[in]
  * @retval    error code
  */
static int32_t shoot_block_check(struct shoot *shoot)
{
    static uint8_t first_block_f = 0;
    static uint32_t check_time;

    device_assert(shoot != NULL);

    if (shoot->motor.current > shoot->param.block_current)
    {
        if (first_block_f == 0)
        {
            first_block_f = 1;
            check_time = get_time_ms();
        }
        else if (get_time_ms() - check_time > shoot->param.check_timeout)
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

    return E_OK;
}

static int32_t shoot_cmd_ctrl(struct shoot *shoot)
{
    device_assert(shoot != NULL);

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
                {
                    shoot->cmd = SHOOT_ONCE_CMD;
                }
            }
            break;
            default:
                break;
            }
        }
        else
        {
            shoot->cmd = SHOOT_STOP_CMD;
        }
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
        shoot->motor_pid.enable = 1;
    }
    else
    {
        shoot->motor_pid.enable = 0;
    }

    return E_OK;
}

/**
  * @brief     friction wheel start slowly.
  * @param[in]
  * @retval    error code
  */
static int32_t shoot_fric_ctrl(struct shoot *shoot)
{
    device_assert(shoot != NULL);

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

    return E_OK;
}
