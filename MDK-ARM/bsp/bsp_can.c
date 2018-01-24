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
/** @file bsp_can.c
 *  @version 1.0
 *  @date Apr 2017
 *
 *  @brief receive external can device message
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "bsp_can.h"
#include "detect_task.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "bsp_uart.h"
#include "string.h"
#include "sys_config.h"

moto_measure_t moto_pit;
moto_measure_t moto_yaw;
moto_measure_t moto_trigger;
moto_measure_t moto_chassis[4];

//float yaw_zgyro_angle;


void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
{
  switch (_hcan->pRxMsg->StdId)
  {
    case CAN_3510_M1_ID:
    case CAN_3510_M2_ID:
    case CAN_3510_M3_ID:
    case CAN_3510_M4_ID:
    {
      static uint8_t i;
      i = _hcan->pRxMsg->StdId - CAN_3510_M1_ID;

      moto_chassis[i].msg_cnt++ <= 50 ? get_moto_offset(&moto_chassis[i], _hcan) : encoder_data_handle(&moto_chassis[i], _hcan);
      err_detector_hook(CHASSIS_M1_OFFLINE + i);
    }
    break;
    case CAN_YAW_MOTOR_ID:
    {
      encoder_data_handle(&moto_yaw, _hcan);
      err_detector_hook(GIMBAL_YAW_OFFLINE);
    }
    break;
    case CAN_PIT_MOTOR_ID:
    {
      encoder_data_handle(&moto_pit, _hcan);
      err_detector_hook(GIMBAL_PIT_OFFLINE);
    }
    break;
    case CAN_TRIGGER_MOTOR_ID:
    {
      if (_hcan == &TRIGGER_CAN)
      {
        moto_trigger.msg_cnt++;
        moto_trigger.msg_cnt <= 10 ? get_moto_offset(&moto_trigger, _hcan) : encoder_data_handle(&moto_trigger, _hcan);
        err_detector_hook(TRIGGER_MOTO_OFFLINE);
      }
      else
      {
      }
    }
    break;
    
    case CAN_CHASSIS_ZGYRO_ID:
    {
      chassis.gyro_angle = 0.001f * ((int32_t)(_hcan->pRxMsg->Data[0] << 24) |
                                              (_hcan->pRxMsg->Data[1] << 16) |
                                              (_hcan->pRxMsg->Data[2] << 8) |
                                              (_hcan->pRxMsg->Data[3]));
      
      chassis.gyro_palstance = 0.001f * ((int32_t)(_hcan->pRxMsg->Data[4] << 24) |
                                                  (_hcan->pRxMsg->Data[5] << 16) |
                                                  (_hcan->pRxMsg->Data[6] << 8) |
                                                  (_hcan->pRxMsg->Data[7]));
      
      err_detector_hook(CHASSIS_GYRO_OFFLINE);
    }
    break;

    case CAN_GIMBAL_ZGYRO_ID:
    {
//      gim.sensor.gyro_angle = 0.001f * ((int32_t)(_hcan->pRxMsg->Data[0] << 24) |
//                                          (_hcan->pRxMsg->Data[1] << 16) |
//                                          (_hcan->pRxMsg->Data[2] << 8) |
//                                          (_hcan->pRxMsg->Data[3]));
//      
//      err_detector_hook(GIMBAL_GYRO_OFFLINE);
    }
    break;
    

    default:
    {
    }
    break;
  }
  
  __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
  __HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_FMP0);
}



/**
  * @brief     get motor rpm and calculate motor round_count/total_encoder/total_angle
  * @param     ptr: Pointer to a moto_measure_t structure
  * @attention this function should be called after get_moto_offset() function
  */
void encoder_data_handle(moto_measure_t* ptr, CAN_HandleTypeDef* hcan)
{
  ptr->last_ecd = ptr->ecd;
  ptr->ecd      = (uint16_t)(hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);
  
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
  
#ifdef CHASSIS_EC60
  int32_t temp_sum = 0;
  ptr->rate_buf[ptr->buf_cut++] = ptr->ecd_raw_rate;
  if (ptr->buf_cut >= FILTER_BUF)
    ptr->buf_cut = 0;
  for (uint8_t i = 0; i < FILTER_BUF; i++)
  {
    temp_sum += ptr->rate_buf[i];
  }
  ptr->filter_rate = (int32_t)(temp_sum/FILTER_BUF);
  ptr->speed_rpm   = (int16_t)(ptr->filter_rate * 7.324f);
#else
  ptr->speed_rpm     = (int16_t)(hcan->pRxMsg->Data[2] << 8 | hcan->pRxMsg->Data[3]);
  ptr->given_current = (int16_t)(hcan->pRxMsg->Data[4] << 8 | hcan->pRxMsg->Data[5]);
#endif

}

/**
  * @brief     get motor initialize offset value
  * @param     ptr: Pointer to a moto_measure_t structure
  * @retval    None
  * @attention this function should be called after system can init
  */
void get_moto_offset(moto_measure_t* ptr, CAN_HandleTypeDef* hcan)
{
    ptr->ecd        = (uint16_t)(hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);
    ptr->offset_ecd = ptr->ecd;
}
/**
  * @brief   can filter initialization
  * @param   CAN_HandleTypeDef
  * @retval  None
  */
void can_device_init(void)
{
  //can1 &can2 use same filter config
  CAN_FilterConfTypeDef  can_filter;
  static CanTxMsgTypeDef Tx1Message;
  static CanRxMsgTypeDef Rx1Message;
  static CanTxMsgTypeDef Tx2Message;
  static CanRxMsgTypeDef Rx2Message;

  can_filter.FilterNumber         = 0;
  can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh         = 0x0000;
  can_filter.FilterIdLow          = 0x0000;
  can_filter.FilterMaskIdHigh     = 0x0000;
  can_filter.FilterMaskIdLow      = 0x0000;
  can_filter.FilterFIFOAssignment = CAN_FilterFIFO0;
  can_filter.BankNumber           = 14;
  can_filter.FilterActivation     = ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &can_filter);
  //while (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK);
  
  can_filter.FilterNumber         = 14;
  HAL_CAN_ConfigFilter(&hcan2, &can_filter);
  //while (HAL_CAN_ConfigFilter(&hcan2, &can_filter) != HAL_OK);
    
  hcan1.pTxMsg = &Tx1Message;
  hcan1.pRxMsg = &Rx1Message;
  hcan2.pTxMsg = &Tx2Message;
  hcan2.pRxMsg = &Rx2Message;
}



void can_receive_start(void)
{
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
  HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0); 
}

/**
  * @brief     reset single axis gyroscope 
  * @attention gyro reset at least wait 2s  
  */
void gyro_device_init(void)
{
  while (ZGYRO_CAN.State == HAL_CAN_STATE_BUSY_TX);
  ZGYRO_CAN.pTxMsg->StdId   = CAN_ZGYRO_RST_ID;
  ZGYRO_CAN.pTxMsg->IDE     = CAN_ID_STD;
  ZGYRO_CAN.pTxMsg->RTR     = CAN_RTR_DATA;
  ZGYRO_CAN.pTxMsg->DLC     = 0x08;
  ZGYRO_CAN.pTxMsg->Data[0] = 0;
  ZGYRO_CAN.pTxMsg->Data[1] = 1;
  ZGYRO_CAN.pTxMsg->Data[2] = 2;
  ZGYRO_CAN.pTxMsg->Data[3] = 3;
  ZGYRO_CAN.pTxMsg->Data[4] = 4;
  ZGYRO_CAN.pTxMsg->Data[5] = 5;
  ZGYRO_CAN.pTxMsg->Data[6] = 6;
  ZGYRO_CAN.pTxMsg->Data[7] = 7;
  HAL_CAN_Transmit(&ZGYRO_CAN, 10);
}

/**
  * @brief  send current which pid calculate to esc. message to calibrate 6025 gimbal motor esc
  * @param  current value corresponding motor(yaw/pitch/trigger)
  */
void send_gimbal_cur(int16_t yaw_iq, int16_t pit_iq, int16_t trigger_iq)
{
  GIMBAL_CAN.pTxMsg->StdId   = 0x1ff;
  GIMBAL_CAN.pTxMsg->IDE     = CAN_ID_STD;
  GIMBAL_CAN.pTxMsg->RTR     = CAN_RTR_DATA;
  GIMBAL_CAN.pTxMsg->DLC     = 8;
  GIMBAL_CAN.pTxMsg->Data[0] = yaw_iq >> 8;
  GIMBAL_CAN.pTxMsg->Data[1] = yaw_iq;
  GIMBAL_CAN.pTxMsg->Data[2] = pit_iq >> 8;
  GIMBAL_CAN.pTxMsg->Data[3] = pit_iq;
  GIMBAL_CAN.pTxMsg->Data[4] = trigger_iq >> 8;
  GIMBAL_CAN.pTxMsg->Data[5] = trigger_iq;
  GIMBAL_CAN.pTxMsg->Data[6] = 0;
  GIMBAL_CAN.pTxMsg->Data[7] = 0;
  HAL_CAN_Transmit(&GIMBAL_CAN, 10);
}

/**
  * @brief  send calculated current to motor
  * @param  3510 motor ESC id
  */
void send_chassis_cur(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
  CHASSIS_CAN.pTxMsg->StdId   = 0x200;
  CHASSIS_CAN.pTxMsg->IDE     = CAN_ID_STD;
  CHASSIS_CAN.pTxMsg->RTR     = CAN_RTR_DATA;
  CHASSIS_CAN.pTxMsg->DLC     = 0x08;
  CHASSIS_CAN.pTxMsg->Data[0] = iq1 >> 8;
  CHASSIS_CAN.pTxMsg->Data[1] = iq1;
  CHASSIS_CAN.pTxMsg->Data[2] = iq2 >> 8;
  CHASSIS_CAN.pTxMsg->Data[3] = iq2;
  CHASSIS_CAN.pTxMsg->Data[4] = iq3 >> 8;
  CHASSIS_CAN.pTxMsg->Data[5] = iq3;
  CHASSIS_CAN.pTxMsg->Data[6] = iq4 >> 8;
  CHASSIS_CAN.pTxMsg->Data[7] = iq4;
  HAL_CAN_Transmit(&CHASSIS_CAN, 10);
}
