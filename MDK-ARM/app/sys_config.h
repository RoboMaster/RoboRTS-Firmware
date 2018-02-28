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

#ifndef __SYS_H__
#define __SYS_H__

#include "stm32f4xx_hal.h"

/**********************remote control setting***************************/
/* normalized remote controller proportion */
#define RC_RESOLUTION     660.0f

/*************************chassis setting*******************************/
/* remote mode chassis move speed limit */
/* left and right speed (mm/s) */
#define CHASSIS_RC_MAX_SPEED_X  3300.0f
#define CHASSIS_RC_MOVE_RATIO_X 1.0f
/* back and forward speed (mm/s) */
#define CHASSIS_RC_MAX_SPEED_Y  3300.0f
#define CHASSIS_RC_MOVE_RATIO_Y 1.0f
/* chassis rotation speed (deg/s) */
/* used only chassis open loop mode */
#define CHASSIS_RC_MAX_SPEED_R 300.0f
#define CHASSIS_RC_MOVE_RATIO_R 1.0f

/* keyboard mode speed limit */
/* left and right speed (mm/s) */
#define CHASSIS_KB_MAX_SPEED_X  3300.0f 
#define CHASSIS_KB_MOVE_RATIO_X 1.0f
/* back and forward speed (mm/s) */
#define CHASSIS_KB_MAX_SPEED_Y  3300.0f
#define CHASSIS_KB_MOVE_RATIO_Y 1.0f

/**************************gimbal setting*******************************/
/* remote mode gimbal speed limit */
/* pitch axis speed */
#define GIMBAL_RC_MOVE_RATIO_PIT 1.0f
/* yaw axis speed */
#define GIMBAL_RC_MOVE_RATIO_YAW 1.0f

/* keyboard mode gimbal speed limit */
/* pitch axis speed */
#define GIMBAL_PC_MOVE_RATIO_PIT 1.0f
/* yaw axis speed */
#define GIMBAL_PC_MOVE_RATIO_YAW 1.0f

/**************************shoot  setting********************************/
/* shoot speed */
#define DEFAULT_FRIC_WHEEL_SPEED 1150 //maximum value is 2500
/* shoot frequence */
#define TRIGGER_MOTOR_SPEED      2000 



/************************ chassis parameter ****************************/
/* the radius of wheel(mm) */
#define RADIUS     76
/* the perimeter of wheel(mm) */
#define PERIMETER  478

/* wheel track distance(mm) */
#define WHEELTRACK 403
/* wheelbase distance(mm) */
#define WHEELBASE  385

/* gimbal is relative to chassis center x axis offset(mm) */
#define GIMBAL_X_OFFSET 150
/* gimbal is relative to chassis center y axis offset(mm) */
#define GIMBAL_Y_OFFSET 0

/* chassis motor use 3508 default */
/* define CHASSIS_EC60 to use EC60 */
#define CHASSIS_EC60

#ifdef CHASSIS_EC60
  /* chassis motor use EC60 */
  /* the deceleration ratio of chassis motor */
  #define CHASSIS_DECELE_RATIO (1.0f)
  /* single 3508 motor maximum speed, unit is rpm */
  #define MAX_WHEEL_RPM        400   //440rpm = 3500mm/s
  /* chassis maximum translation speed, unit is mm/s */
  #define MAX_CHASSIS_VX_SPEED 3300  //415rpm
  #define MAX_CHASSIS_VY_SPEED 3300
  /* chassis maximum rotation speed, unit is degree/s */
  #define MAX_CHASSIS_VR_SPEED 300
#else
  /* chassis motor use 3508 */
  /* the deceleration ratio of chassis motor */
  #define CHASSIS_DECELE_RATIO (1.0f/19.0f)
  /* single 3508 motor maximum speed, unit is rpm */
  #define MAX_WHEEL_RPM        8500  //8347rpm = 3500mm/s
  /* chassis maximum translation speed, unit is mm/s */
  #define MAX_CHASSIS_VX_SPEED 3300  //8000rpm
  #define MAX_CHASSIS_VY_SPEED 3300
  /* chassis maximum rotation speed, unit is degree/s */
  #define MAX_CHASSIS_VR_SPEED 300   //5000rpm
#endif

/************************** gimbal parameter *****************************/
/* the ratio of motor encoder value translate to degree */
#define ENCODER_ANGLE_RATIO    (8192.0f/360.0f)
/* the deceleration ratio of pitch axis motor */
#define PIT_DECELE_RATIO       1.0f
/* the deceleration ratio of yaw axis motor */
#define YAW_DECELE_RATIO       1.0f
/* the positive direction of pitch axis motor */
#define PIT_MOTO_POSITIVE_DIR  -1.0f
/* the positive direction of yaw axis motor */
#define YAW_MOTO_POSITIVE_DIR  1.0f
/* the positive direction of tirgger motor */
#define TRI_MOTO_POSITIVE_DIR  1.0f




/***********************system interface setting****************************/

/* automatic navigation interface */
#define AUTO_NAVIGATION

/* can relevant */
#define CHASSIS_CAN       hcan1
#define ZGYRO_CAN         hcan2
#define CHASSIS_ZGYRO_CAN hcan1
#define GIMBAL_CAN        hcan1
#define TRIGGER_CAN       hcan1


/* uart relevant */
/**
  * @attention
  * close usart DMA receive interrupt, so need add 
  * uart_receive_handler() before HAL_UART_IROHandler() in uart interrupt function
*/
#define DBUS_HUART         huart1 //for dji remote controler reciever
#define JUDGE_HUART        huart3 //connected to judge system
#define COMPUTER_HUART     huart6 //connected to manifold/TXone


/* gimbal relevant */
#define PIT_ANGLE_MAX        15
#define PIT_ANGLE_MIN        -20
#define YAW_ANGLE_MAX        50
#define YAW_ANGLE_MIN        -50

#define LEFT_FRICTION        TIM1->CCR1
#define RIGHT_FIRCTION       TIM1->CCR4

/* detect task relevant */
#define BEEP_ERR
#ifdef BEEP_ERR
  #define BEEP_TUNE TIM3->ARR
  #define BEEP_CTRL TIM3->CCR1
#endif

#define DEFAULT_TUNE  300

/* imu temperature control */
#define IMU_PWM_PULSE      TIM3->CCR2
#define DEFAULT_IMU_TEMP   50

/* math relevant */

/* radian coefficient */
#define RADIAN_COEF 57.3f
/* circumference ratio */
#define PI          3.142f

#define VAL_LIMIT(val, min, max) \
do {\
if((val) <= (min))\
{\
  (val) = (min);\
}\
else if((val) >= (max))\
{\
  (val) = (max);\
}\
} while(0)\

#endif
