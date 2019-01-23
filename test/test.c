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

#include "test_module.h"
#include "can.h"
#include "drv_can.h"
#include "drv_imu.h"
#include "drv_io.h"
#include "motor.h"
#include "detect.h"
#include "dbus.h"
#include "ahrs.h"
#include "pid_controller.h"
#include "board.h"
#include "init.h"
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "log_test.h"

//#define TEST_SELF_ENABLE

#ifdef TEST_SELF_ENABLE

#define TEST_MOTOR_DEFAULT \
  {                        \
    {0},                   \
    {0},                   \
    DEVICE_CAN1,           \
    0x201,                 \
    0, 0, 0                \
  }

struct ahrs_sensor test_sensor;
struct attitude test_madgwick_atti;
struct attitude test_mahony_atti;

struct rc_device test_rc;
struct detect_device test_detect;

struct chassis test_chassis;
struct gimbal test_gimbal;
struct shoot test_shoot;

struct motor_device test_motor = TEST_MOTOR_DEFAULT;
struct controller test_motor_ctrl;
struct pid test_pid;
struct pid_feedback test_pid_feedback;
float test_target;

void ahrs_test(void *argc);
void motor_test(void *argc);
void chassis_test(void *argc);
void rc_test(void *argc);
void gimbal_test(void *argc);
void gimbal_pid_test(void *argc);
void limit_test(void *argc);
void shoot_test(void *argc);
void detect_test(void *argc);

void test_call_fnuc1(void *argc);
void test_call_fnuc2(void *argc);

int32_t test_pid_motor_convert_input(struct controller *ctrl, void *input)
{
  pid_feedback_t pid_fdb = (pid_feedback_t)(ctrl->feedback);
  motor_data_t data = (motor_data_t)input;
  pid_fdb->feedback = data->speed_rpm;

  return RM_OK;
}

void test_init(void)
{
  /*******rc test********************/
  // rc_device_register(&test_rc, "rc", 0);
  // test_module_register((void *)rc_test, NULL);

  /*******motor controller***********/
  // pid_struct_init(&test_pid, 10000, 500, 4.5f, 0.05, 0);
  // test_motor_ctrl.convert_feedback = test_pid_motor_convert_input;
  // pid_controller_register(&test_motor_ctrl, "test_ctrl", &test_pid, &test_pid_feedback, 1);
  // motor_device_register(&test_motor, "test_motor", 0);
  // test_module_register((void *)motor_test, NULL);

  /*******gimbal controller***********/
  //  mpu_device_init();
  //  test_module_register((void *)mpu_get_data, (void *)&test_sensor);
  //  test_module_register((void *)ahrs_test, NULL);
  //  gimbal_cascade_register(&test_gimbal, "gimbal", DEVICE_CAN1);
  //  test_module_register((void *)gimbal_test, NULL);
  //  test_module_register((void *)gimbal_pid_test, NULL);

  /*******chassis controller***********/
  //  chassis_pid_register(&test_chassis, "chassis", DEVICE_CAN1);
  //  test_module_register((void *)chassis_test, NULL);

  /*******limit test*******************/
  //  test_module_register((void *)limit_test, NULL);
  
  /*******shoot test*******************/
  // pwm_device_init();
  // shoot_pid_register(&test_shoot, "shoot", DEVICE_CAN1);
  // test_module_register((void *)shoot_test, NULL);
  
  /*******detect test******************/
  // detect_device_register(&test_detect, "detect", 0, 0);
  // detect_device_add_event(&test_detect, 1, 100, test_call_fnuc1, NULL);
  // detect_device_add_event(&test_detect, 2, 100, test_call_fnuc2, NULL);
  // test_module_register((void *)detect_test, NULL);
  
  /*******log test******************/
  // log_test_init();
  
  /*******cli test******************/
  
}

int32_t test_detect_flag;
int32_t test_detect_state1, test_detect_state2;
uint8_t call_state1, call_state2;

void detect_test(void *argc)
{
  detect_device_check(&test_detect, 0xffffffff);  

  if(detect_device_get_event(&test_detect) == 0)
  {
    test_detect_state1 = 0;
    test_detect_state2 = 0;
    call_state1 = 0;
    call_state2 = 0;
  }

  if(detect_device_get_event(&test_detect) == 1)
  {
    test_detect_state1 = 1;
  }
  else
  {
    test_detect_state1 = 0;
  }

  if(detect_device_get_event(&test_detect) == 2)
  {
    test_detect_state2 = 1;
  }
  else
  {
    test_detect_state2 = 0;
  }
  
  if(test_detect_flag == 0)
  {
    detect_device_set_mode(&test_detect, 0); 
  }
  if(test_detect_flag == 1)
  {
    detect_device_set_mode(&test_detect, 1);  
  }
  if(test_detect_flag == 2)
  {
    detect_device_update(&test_detect, 3); 
  }
  if(test_detect_flag == 3)
  {
    detect_device_update(&test_detect, 1); 
  }
  if(test_detect_flag == 4)
  {
    detect_device_update(&test_detect, 2);  
  }
  if(test_detect_flag == 5)
  {
    detect_device_enable_event(&test_detect, 1); 
  }
  if(test_detect_flag == 6)
  {
    detect_device_disable_event(&test_detect, 1);  
  }
}

void test_call_fnuc1(void *argc)
{
  call_state1 = 1;
}

void test_call_fnuc2(void *argc)
{
  call_state2 = 1;
}

uint8_t rc_flag;

void rc_test(void *argc)
{
  if (rc_device_get_state(&test_rc, RC_S1_UP2MID) == RM_OK)
  {
    rc_flag = 1;
  }
  if (rc_device_get_state(&test_rc, RC_S1_MID2UP) == RM_OK)
  {
    rc_flag = 2;
  }
  if (rc_device_get_state(&test_rc, RC_S1_DOWN2MID) == RM_OK)
  {
    rc_flag = 3;
  }
  if (rc_device_get_state(&test_rc, RC_S1_MID2DOWN) == RM_OK)
  {
    rc_flag = 4;
  }
  if (rc_device_get_state(&test_rc, RC_S2_UP2MID) == RM_OK)
  {
    rc_flag = 1;
  }
  if (rc_device_get_state(&test_rc, RC_S2_MID2UP) == RM_OK)
  {
    rc_flag = 2;
  }
  if (rc_device_get_state(&test_rc, RC_S2_DOWN2MID) == RM_OK)
  {
    rc_flag = 3;
  }
  if (rc_device_get_state(&test_rc, RC_S2_MID2DOWN) == RM_OK)
  {
    rc_flag = 4;
  }
}

void chassis_test(void *argc)
{
  chassis_execute(&test_chassis);
  motor_device_can_output(DEVICE_CAN1);
}

void gimbal_test(void *argc)
{
  test_gimbal.param.yaw_ecd_center = 5126;
  test_gimbal.param.pitch_ecd_center = 6100;
  gimbal_pitch_gyro_update(&test_gimbal, test_mahony_atti.roll);
  gimbal_rate_update(&test_gimbal, test_sensor.wz*RAD_TO_DEG, test_sensor.wx*RAD_TO_DEG);
  gimbal_set_yaw_mode(&test_gimbal, GYRO_MODE);
  gimbal_execute(&test_gimbal);
  motor_device_can_output(DEVICE_CAN1);
}

int32_t yaw_angle_fdb_js, yaw_angle_ref_js;
int32_t pit_angle_fdb_js, pit_angle_ref_js;
int32_t yaw_spd_fdb_js, yaw_spd_ref_js;
int32_t pit_spd_fdb_js, pit_spd_ref_js; 

float test_angle_yaw;
float test_angle_pitch;
float test_delta_pitch;
float test_delta_yaw;
uint8_t test_yaw_mode;
uint8_t test_gim_flag;

void gimbal_set_inter_test(void *argc)
{
  if(test_gim_flag == 1)
  {
    gimbal_set_pitch_angle(&test_gimbal, test_angle_pitch);
    gimbal_set_yaw_angle(&test_gimbal, test_angle_yaw, test_yaw_mode);
    test_gim_flag = 0;
  }

  if(test_gim_flag == 2)
  {
    gimbal_set_pitch_delta(&test_gimbal, test_delta_pitch);
    gimbal_set_yaw_delta(&test_gimbal, test_delta_yaw);
    test_gim_flag = 0;
  }
  
  if(test_gim_flag == 3)
  {
    gimbal_pitch_disable(&test_gimbal);
    gimbal_yaw_disable(&test_gimbal);
  }
  
  if(test_gim_flag == 4)
  {
    gimbal_pitch_enable(&test_gimbal);
    gimbal_yaw_enable(&test_gimbal);
  }
  
  yaw_angle_fdb_js = test_gimbal.cascade[0].outer.get * 1000;
  yaw_angle_ref_js = test_gimbal.cascade[0].outer.set * 1000;
  pit_angle_fdb_js = test_gimbal.cascade[1].outer.get * 1000;
  pit_angle_ref_js = test_gimbal.cascade[1].outer.set * 1000;

  yaw_spd_fdb_js = test_gimbal.cascade[0].inter.get * 1000;
  yaw_spd_ref_js = test_gimbal.cascade[0].inter.set * 1000;
  pit_spd_fdb_js = test_gimbal.cascade[1].inter.get * 1000;
  pit_spd_ref_js = test_gimbal.cascade[1].inter.set * 1000; 

}

uint8_t test_shoot_flag; 
uint16_t test_fric_spd1, test_fric_spd2;
uint16_t test_get_spd1, test_get_spd2;
uint8_t test_cmd;
uint32_t test_shoot_num;

void shoot_test(void *argc)
{
  shoot_execute(&test_shoot);

  if(test_shoot_flag ==1)
  {
    shoot_set_fric_speed(&test_shoot, test_fric_spd1, test_fric_spd2);
    test_shoot_flag = 0; 
  }
  if(test_shoot_flag ==2)
  {
     shoot_get_fric_speed(&test_shoot, &test_get_spd1, &test_get_spd2);
    test_shoot_flag = 0; 
  }
  if(test_shoot_flag ==3)
  {
    shoot_set_cmd(&test_shoot, test_cmd, test_shoot_num);
    test_shoot_flag = 0; 
  }
  if(test_shoot_flag ==4)
  {
    shoot_enable(&test_shoot);
    test_shoot_flag = 0; 
  }
  if(test_shoot_flag ==5)
  {
    shoot_disable(&test_shoot);
    test_shoot_flag = 0; 
  }
  motor_device_can_output(DEVICE_CAN1);
}

void ahrs_test(void *argc)
{
//  madgwick_ahrs_updateIMU((struct ahrs_sensor *)&test_sensor, &test_madgwick_atti);
  mahony_ahrs_updateIMU((struct ahrs_sensor *)&test_sensor, &test_mahony_atti);
}

float test_data;
float test_360, test_180;

void limit_test(void *argc)
{
  ANGLE_LIMIT_360(test_360, test_data);
  
  ANGLE_LIMIT_360(test_180, test_data);
  ANGLE_LIMIT_360_TO_180(test_180);
}

void motor_test(void *argc)
{
  float test_out;
  motor_data_t ptest_data;

  ptest_data = motor_device_get_data(&test_motor);

  controller_set_input(&test_motor_ctrl, test_target);
  controller_execute(&test_motor_ctrl, (void *)ptest_data);
  controller_get_output(&test_motor_ctrl, &test_out);

  motor_device_set_current(&test_motor, (int16_t)test_out);

  motor_device_can_output(DEVICE_CAN1);
}

#else

void test_init(void)
{

}

#endif

