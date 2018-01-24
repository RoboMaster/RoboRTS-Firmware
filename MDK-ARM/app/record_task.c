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
/** @file record_task.c
 *  @version 1.1
 *  @date Sep 2017
 *
 *  @brief SD card write and read interface and user custom module
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "record_task.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "pid.h"
#include "sys_config.h"

#include "rtc.h"
#include "fatfs.h"
#include "string.h"
#include "fatfs.h"

#if 0
UBaseType_t execute_stack_surplus;

/* file system relevant */
static FATFS sd_fatfs;
static FIL my_file;
static FRESULT res;
static char *f_name = "test_long_name_file.txt";

/* writen data */
static uint32_t byteswritten;
static char wtext[50];
static uint32_t f_pointer = 0;

/* rtc time */
static RTC_DateTypeDef sdate;
static RTC_TimeTypeDef stime;

static uint8_t sd_in;
#endif

/**
  * @brief        write data at the end of the file
  * @param[in]    pointer to the file object
  * @param[in]    pointer to the data to be written
  * @retval       the number of bytes written
  */
uint32_t sd_write_data(FIL *file, char *data)
{
  uint32_t f_p;
  uint32_t w_bytes;
  f_p = file->fsize;
  f_lseek(file, f_p);
  f_write(file, data, strlen(data), (void *)&w_bytes);
  return w_bytes;
}

void record_task(void const *argu)
{
  osThreadSuspend(NULL);
//  sd_in = sd_insert();
//  if (sd_in)
//  {
//    res = f_mount(&sd_fatfs, (TCHAR const*)SDPath, 1);
//    res = f_open(&my_file, f_name, FA_CREATE_ALWAYS | FA_WRITE);
//    
//    for (int i = 0; i < 1000; i++)
//    {
//      HAL_RTC_GetTime(&hrtc, &stime, RTC_FORMAT_BIN);
//      HAL_RTC_GetDate(&hrtc, &sdate, RTC_FORMAT_BIN);
//      
//      memset(wtext, 0, 50);
//      sprintf(wtext, "20%02d-%02d-%02d %02d:%02d:%02d\r\n", 
//                     sdate.Year, sdate.Month, sdate.Date, 
//                     stime.Hours, stime.Minutes, stime.Seconds);
//      
//      byteswritten = sd_write_data(&my_file, wtext);
//     
//      osDelay(5);
//    }
//    
//    f_close(&my_file);
//  }
//  uint32_t record_wake_time = osKernelSysTick();
//  while(1)
//  {
//    HAL_RTC_GetTime(&hrtc, &stime, RTC_FORMAT_BIN);
//    HAL_RTC_GetDate(&hrtc, &sdate, RTC_FORMAT_BIN);
//    
//    execute_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
//    
//    osDelayUntil(&record_wake_time, record_task_PERIOD);
//  }

}






