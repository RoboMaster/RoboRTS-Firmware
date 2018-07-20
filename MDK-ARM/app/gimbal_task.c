/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
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
/** @file gimbal_task.c
 *  @version 1.1
 *  @date Oct 2017
 *
 *  @brief gimbal control task
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "gimbal_task.h"
#include "chassis_task.h"
#include "shoot_task.h"
#include "comm_task.h"
#include "modeswitch_task.h"
#include "info_interactive.h"
#include "remote_ctrl.h"
#include "infantry_info.h"
#include "bsp_uart.h"
#include "calibrate.h"
#include "keyboard.h"
#include "pid.h"
#include "sys_config.h"
#include "ramp.h"
#include "cmsis_os.h"
#include "math.h"
#include "stdlib.h" 
#include "string.h"

#include "kalman_filter.h"

//#define OLD_TRIGGER
/* gimbal patrol angle (degree)*/
#define PATROL_ANGLE     15
/* patrol period time (ms) */
#define PATROL_PERIOD    3000
/* gimbal back center time (ms) */
#define BACK_CENTER_TIME 2500
/* stack usage monitor */
UBaseType_t gimbal_stack_surplus;

/* gimbal task global parameter */
gimbal_t gim;

/* gimbal task static parameter */
/* control ramp parameter */
static ramp_t     yaw_ramp = RAMP_GEN_DAFAULT;
static ramp_t     pit_ramp = RAMP_GEN_DAFAULT;

uint32_t gimbal_time_last;
uint32_t gimbal_time_ms;


/* for debug */
int yaw_angle_fdb_js;
int yaw_angle_ref_js;
int yaw_speed_fdb_js;
int yaw_speed_ref_js;

int pit_angle_fdb_js;
int pit_angle_ref_js;
int pit_speed_fdb_js;
int pit_speed_ref_js;


typedef struct  // speed_calc_data_t
{
  int delay_cnt;
  int freq;
  int last_time;
  float last_position;
  float speed;
  float last_speed;
  float processed_speed;
} speed_calc_data_t;


/* kalman param */
#ifdef CAMERA_ON_GIMBAL
kalman_filter_init_t yaw_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.001, 0, 1},
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {1000, 0, 0, 2000}
};
#else
kalman_filter_init_t yaw_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.001, 0, 1},
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {1000, 0, 0, 2000}
};
#endif

kalman_filter_init_t pitch_kalman_filter_para = 
{
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.001, 0, 1},
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {2000, 0, 0, 5000}
};

kalman_filter_init_t dist_kalman_filter_para = 
{
  .P_data = {2, 0, 0, 0},
  .A_data = {1, 0, 0, 0},
  .H_data = {1, 0, 0, 0},
  .Q_data = {1, 0, 0, 0},
  .R_data = {10000, 0, 0, 0}
};


kalman_filter_t yaw_kalman_filter;
kalman_filter_t pitch_kalman_filter;
kalman_filter_t dist_kalman_filter;

speed_calc_data_t yaw_speed_struct;
speed_calc_data_t pitch_speed_struct;
speed_calc_data_t dist_speed_struct;

static float yaw_speed_raw;
static float pitch_speed_raw;
static float dist_speed_raw;

// yaw pitch yaw_v pitch_v
float gimbal_attitude_archive[120][4];
uint8_t archive_index = 0;
uint8_t get_index = 0;

float target_speed_calc(speed_calc_data_t *S, uint32_t time, float position);
//float target_speed_calc(speed_calc_data_t *S, float position);


void gimbal_self_check(void)
{
  if ( !read_gimbal_offset(&(gim.pit_center_offset), &(gim.yaw_center_offset)) )
  {
    /* gimbal has not been calibrated */
    no_cali_data_handler();
  }
}

extern TaskHandle_t can_msg_send_task_t;
extern TaskHandle_t shoot_task_t;
void gimbal_task(void const *argu)
{
  gimbal_time_ms = HAL_GetTick() - gimbal_time_last;
  gimbal_time_last = HAL_GetTick();
  
//---------------------------------------------------
  archive_index++;
  if (archive_index > 99) //system delay ms
    archive_index = 0;
  gimbal_attitude_archive[archive_index][0] = gim.sensor.yaw_gyro_angle;//gim.pid.yaw_angle_fdb;
  gimbal_attitude_archive[archive_index][1] = gim.pid.pit_angle_fdb;
  gimbal_attitude_archive[archive_index][2] = gim.pid.yaw_spd_fdb;
  gimbal_attitude_archive[archive_index][3] = gim.pid.pit_spd_fdb;
  get_index = archive_index + 50;
  if( get_index > 99)
    get_index -= 100;
//----------------------------------------------------
  
  mat_init(&yaw_kalman_filter.Q,2,2, yaw_kalman_filter_para.Q_data);
  mat_init(&yaw_kalman_filter.R,2,2, yaw_kalman_filter_para.R_data);
  
  
  switch (gim.ctrl_mode)
  {
    case GIMBAL_INIT:
      init_mode_handler();
    break;
    
    case GIMBAL_NO_ARTI_INPUT:
      no_action_handler();
    break;

    case GIMBAL_FOLLOW_ZGYRO:
      closed_loop_handler();
    break;

    case GIMBAL_PATROL_MODE:
      shoot.c_shoot_cmd = 0;
      gimbal_patrol_handler();
    break;

    case GIMBAL_POSITION_MODE:
      pc_position_ctrl_handler();
    break;

    case GIMBAL_RELATIVE_MODE:
      pc_relative_ctrl_handler();
    break;

    default:
    break;
  }

  pid_calc(&pid_yaw, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref);
  pid_calc(&pid_pit, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref);
  
  gim.pid.yaw_spd_ref = pid_yaw.out;
  gim.pid.pit_spd_ref = pid_pit.out;
  
  gim.pid.yaw_spd_fdb = gim.sensor.yaw_palstance;
  gim.pid.pit_spd_fdb = gim.sensor.pit_palstance;
  
  pid_calc(&pid_yaw_spd, gim.pid.yaw_spd_fdb, gim.pid.yaw_spd_ref);
  pid_calc(&pid_pit_spd, gim.pid.pit_spd_fdb, gim.pid.pit_spd_ref);

  /* safe protect */
  if (gimbal_is_controllable())
  {
    glb_cur.gimbal_cur[0] = YAW_MOTO_POSITIVE_DIR*pid_yaw_spd.out;
    glb_cur.gimbal_cur[1] = PIT_MOTO_POSITIVE_DIR*pid_pit_spd.out;
    glb_cur.gimbal_cur[2] = pid_trigger_spd.out;
  }
  else
  {
    memset(glb_cur.gimbal_cur, 0, sizeof(glb_cur.gimbal_cur));
    gim.ctrl_mode = GIMBAL_RELAX;
    //pid_trigger.iout = 0;
  }
  
  yaw_angle_ref_js = gim.pid.yaw_angle_ref*1000;
  yaw_angle_fdb_js = gim.pid.yaw_angle_fdb*1000;
  yaw_speed_ref_js = pid_yaw.out*1000;
  yaw_speed_fdb_js = gim.sensor.yaw_palstance*1000;

  pit_angle_ref_js = gim.pid.pit_angle_ref*1000;
  pit_angle_fdb_js = gim.pid.pit_angle_fdb*1000;
  pit_speed_ref_js = pid_pit.out*1000;
  pit_speed_fdb_js = gim.sensor.pit_palstance*1000;

  osSignalSet(can_msg_send_task_t, GIMBAL_MOTOR_MSG_SEND);
  osSignalSet(shoot_task_t, SHOT_TASK_EXE_SIGNAL);

  gimbal_stack_surplus = uxTaskGetStackHighWaterMark(NULL);

}

void init_mode_handler(void)
{
  /* lift gimbal pitch */
  gim.pid.pit_angle_fdb = gim.sensor.pit_relative_angle;
  gim.pid.pit_angle_ref = gim.sensor.pit_relative_angle * (1 - ramp_calc(&pit_ramp));
  /* keep yaw unmove this time */
  gim.pid.yaw_angle_fdb = gim.sensor.yaw_relative_angle;
  gim.pid.yaw_angle_ref = gim.ecd_offset_angle;

  if(gim.pid.pit_angle_fdb <= 2.0f)
  {
    /* yaw back center after pitch arrive */
    gim.pid.yaw_angle_ref = gim.sensor.yaw_relative_angle * ( 1 - ramp_calc(&yaw_ramp));
    
    if (gim.pid.yaw_angle_fdb >= -1.5f && gim.pid.yaw_angle_fdb <= 1.5f)
    {
      /* yaw arrive and switch gimbal state */
      gim.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
      
      gim.yaw_offset_angle = gim.sensor.yaw_gyro_angle;
      gim.pid.pit_angle_ref = 0;
      gim.pid.yaw_angle_ref = 0;
    }
  }
}

void no_action_handler(void)
{
  if (gim.input.no_action_flag == 1)
  {
    if ((HAL_GetTick() - gim.input.no_action_time) < 1500)
    {
      closed_loop_handler();
    }
    else
    {
      gim.input.no_action_flag = 2;
      gim.pid.yaw_angle_ref = 0;
    }
  }
  
  if (gim.input.no_action_flag == 2)
  {
    chassis.follow_gimbal = 0;
    gim.pid.pit_angle_fdb = gim.sensor.pit_relative_angle;
    gim.pid.yaw_angle_fdb = gim.sensor.yaw_relative_angle;
  }
}

void closed_loop_handler(void)
{
  static float chassis_angle_tmp;
  static float limit_angle_range = 2;
  
  gim.pid.pit_angle_fdb = gim.sensor.pit_relative_angle;
  gim.pid.yaw_angle_fdb = gim.sensor.yaw_gyro_angle - gim.yaw_offset_angle;
  
  /* chassis angle relative to gim.pid.yaw_angle_fdb */
  chassis_angle_tmp = gim.pid.yaw_angle_fdb - gim.sensor.yaw_relative_angle;
  /* limit gimbal yaw axis angle */
  if (chassis.ctrl_mode == DODGE_MODE)
  {
    if ((gim.sensor.yaw_relative_angle >= YAW_ANGLE_MIN - limit_angle_range - 35) && \
        (gim.sensor.yaw_relative_angle <= YAW_ANGLE_MAX + limit_angle_range + 35))
    {
      gim.pid.yaw_angle_ref += rm.yaw_v * GIMBAL_RC_MOVE_RATIO_YAW
                             + km.yaw_v * GIMBAL_PC_MOVE_RATIO_YAW;
      
      VAL_LIMIT(gim.pid.yaw_angle_ref, chassis_angle_tmp + YAW_ANGLE_MIN - 35, chassis_angle_tmp + YAW_ANGLE_MAX + 35);
    }
  }
  else
  {
    if ((gim.sensor.yaw_relative_angle >= YAW_ANGLE_MIN - limit_angle_range) && \
        (gim.sensor.yaw_relative_angle <= YAW_ANGLE_MAX + limit_angle_range))
    {
      gim.pid.yaw_angle_ref += rm.yaw_v * GIMBAL_RC_MOVE_RATIO_YAW
                             + km.yaw_v * GIMBAL_PC_MOVE_RATIO_YAW;
      
      VAL_LIMIT(gim.pid.yaw_angle_ref, chassis_angle_tmp + YAW_ANGLE_MIN, chassis_angle_tmp + YAW_ANGLE_MAX);
    }
  }
  
  /* limit gimbal pitch axis angle */
  if ((gim.sensor.pit_relative_angle >= PIT_ANGLE_MIN - limit_angle_range) && \
      (gim.sensor.pit_relative_angle <= PIT_ANGLE_MAX + limit_angle_range))
  {
    gim.pid.pit_angle_ref += rm.pit_v * GIMBAL_RC_MOVE_RATIO_PIT
                       + km.pit_v * GIMBAL_PC_MOVE_RATIO_PIT;
    VAL_LIMIT(gim.pid.pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
  }
}


/* camera on chassis */
void pc_position_ctrl_handler(void)
{
  static float chassis_angle_tmp;
  chassis_angle_tmp = gim.pid.yaw_angle_fdb - gim.sensor.yaw_relative_angle;
  
  gim.pid.pit_angle_fdb = gim.sensor.pit_relative_angle;
  gim.pid.yaw_angle_fdb = gim.sensor.yaw_relative_angle;
  
  gim.pid.yaw_angle_ref = pc_recv_mesg.gimbal_control_data.yaw_ref;
  gim.pid.pit_angle_ref = pc_recv_mesg.gimbal_control_data.pit_ref;
  
  VAL_LIMIT(gim.pid.yaw_angle_ref, chassis_angle_tmp + YAW_ANGLE_MIN - 35, chassis_angle_tmp + YAW_ANGLE_MAX + 35);
  VAL_LIMIT(gim.pid.pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
}






/* camera on gimbal */
extern uint32_t pc_yaw_time;
uint32_t time_raw;
uint32_t time_raw_last;
float position_threshold = 100;
float shoot_delta        = 1.0;

void pc_relative_ctrl_handler(void)
{
  static float yaw_angle_raw;
  static float pitch_angle_raw;
  static float dist_mm_raw;
  
  static float chassis_angle_tmp;
  
  if ((HAL_GetTick() - pc_yaw_time) > 2000)
    gim.pid.yaw_angle_fdb = gim.sensor.yaw_relative_angle;
  else
    gim.pid.yaw_angle_fdb = gim.sensor.yaw_gyro_angle;

  gim.pid.pit_angle_fdb = gim.sensor.pit_relative_angle;
  
  //get raw control command
  time_raw = pc_recv_mesg.gimbal_control_data.time;
  if ((time_raw != time_raw_last) && (pc_recv_mesg.gimbal_control_data.ctrl_mode == GIMBAL_RELATIVE_MODE))
  {
    yaw_angle_raw   = pc_recv_mesg.gimbal_control_data.yaw_ref + gimbal_attitude_archive[get_index][0];
    pitch_angle_raw = pc_recv_mesg.gimbal_control_data.pit_ref;
    dist_mm_raw     = pc_recv_mesg.gimbal_control_data.tgt_dist;
    
    time_raw_last = time_raw;
  }
  
  //calc speed
  yaw_speed_raw   = target_speed_calc(&yaw_speed_struct,   time_raw, yaw_angle_raw);
  pitch_speed_raw = target_speed_calc(&pitch_speed_struct, time_raw, pitch_angle_raw);
  dist_speed_raw  = target_speed_calc(&dist_speed_struct,  time_raw, dist_mm_raw);

  //kalman output  0:angle  1:speed
  float *yaw_kf_result   = kalman_filter_calc(&yaw_kalman_filter,   yaw_angle_raw,   yaw_speed_raw);
  float *pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, pitch_angle_raw, pitch_speed_raw);
  float *dist_kf_result  = kalman_filter_calc(&dist_kalman_filter,  dist_mm_raw,     dist_speed_raw);

  
  if ((gim.sensor.yaw_relative_angle >= YAW_ANGLE_MIN-35) && \
      (gim.sensor.yaw_relative_angle <= YAW_ANGLE_MAX+35))
  {
    gim.pid.yaw_angle_ref = yaw_kf_result[0];// + yaw_kf_result[1]*(0.10f + dist_kf_result[0]/18000);
  }
  chassis_angle_tmp = gim.sensor.yaw_gyro_angle - gim.sensor.yaw_relative_angle;
  VAL_LIMIT(gim.pid.yaw_angle_ref, chassis_angle_tmp + YAW_ANGLE_MIN - 35, chassis_angle_tmp + YAW_ANGLE_MAX + 35);
  
  gim.pid.pit_angle_ref = pitch_angle_raw;//pitch_kf_result[0] + pit_deg;
  VAL_LIMIT(gim.pid.pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
  
  
  if ((HAL_GetTick() - pc_yaw_time) > 2000)
  {
    gim.pid.pit_angle_ref = 0;
    gim.pid.yaw_angle_ref = 0;
    
    shoot.c_shoot_cmd = 0;
  }
  else
  {
    if (fabs(pc_recv_mesg.gimbal_control_data.yaw_ref) < shoot_delta)
      shoot.c_shoot_cmd = 1;
    else
      shoot.c_shoot_cmd = 0;
    
    if (((HAL_GetTick() - pc_yaw_time) > 500) || (dist_mm_raw > 3000))
      shoot.c_shoot_cmd = 0;
  }
  
  //************taskEXIT_CRITICAL();****************
}


int8_t count_sign = 1;
static void gimbal_patrol_handler(void)
{
  static float patrol_once_angle = PATROL_ANGLE*4.0/(PATROL_PERIOD/GIMBAL_PERIOD);
  
  if (gim.sensor.yaw_relative_angle >= PATROL_ANGLE)
    count_sign = -1;
  
  if(gim.sensor.yaw_relative_angle <= -PATROL_ANGLE)
    count_sign = 1;
  
  gim.pid.pit_angle_ref = 0;
  gim.pid.yaw_angle_ref += count_sign*patrol_once_angle;
  
  gim.pid.pit_angle_fdb = gim.sensor.pit_relative_angle;
  gim.pid.yaw_angle_fdb = gim.sensor.yaw_relative_angle;
  
  VAL_LIMIT(gim.pid.yaw_angle_ref, YAW_ANGLE_MIN - 35, YAW_ANGLE_MAX + 35);
}


/**
  * @brief initialize gimbal pid parameter
  *
  */
void gimbal_param_init(void)
{
  memset(&gim, 0, sizeof(gimbal_t));
  
  gim.ctrl_mode      = GIMBAL_NO_ARTI_INPUT;
  gim.last_ctrl_mode = GIMBAL_RELAX;
  gim.input.ac_mode        = NO_ACTION;
  gim.input.action_angle   = 5.0f;
  
  /* pitch axis motor pid parameter */
  PID_struct_init(&pid_pit, POSITION_PID, 1000, 0,
                  30, 0, 0); //
  PID_struct_init(&pid_pit_spd, POSITION_PID, 6000, 3000,
                  12, 0.1, 0);  //16

  /* yaw axis motor pid parameter */
  PID_struct_init(&pid_yaw, POSITION_PID, 1000, 0,
                  60, 0, 0); //
  PID_struct_init(&pid_yaw_spd, POSITION_PID, 6000, 3000,
                  20 , 0.2, 0);  //30
  
  /* bullet trigger motor pid parameter */
  PID_struct_init(&pid_trigger, POSITION_PID, 10000, 2000,
                  15, 0, 10);
  PID_struct_init(&pid_trigger_spd, POSITION_PID, 8000, 3000,
                  1.5, 0.1, 5);
                  
  kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);
  kalman_filter_init(&pitch_kalman_filter, &pitch_kalman_filter_para);
  kalman_filter_init(&dist_kalman_filter, &dist_kalman_filter_para);
}

void gimbal_back_param(void)
{
  ramp_init(&pit_ramp, BACK_CENTER_TIME/GIMBAL_PERIOD);
  ramp_init(&yaw_ramp, BACK_CENTER_TIME/GIMBAL_PERIOD);
}



float speed_threshold = 10.0f;
float target_speed_calc(speed_calc_data_t *S, uint32_t time, float position)
{
  S->delay_cnt++;

  if (time != S->last_time)
  {
    S->speed = (position - S->last_position) / (time - S->last_time) * 1000;
#if 1
    if ((S->speed - S->processed_speed) < -speed_threshold)
    {
        S->processed_speed = S->processed_speed - speed_threshold;
    }
    else if ((S->speed - S->processed_speed) > speed_threshold)
    {
        S->processed_speed = S->processed_speed + speed_threshold;
    }
    else 
#endif
      S->processed_speed = S->speed;
    
    S->last_time = time;
    S->last_position = position;
    S->last_speed = S->speed;
    S->delay_cnt = 0;
  }
  
  if(S->delay_cnt > 200) // delay 200ms speed = 0
  {
    S->processed_speed = 0;
  }

  return S->processed_speed;
}


