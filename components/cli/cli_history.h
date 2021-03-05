/**
*  @file history_input.c/h
*  @version 1.0
*  @date 2017-09-27
*  @author sky
*
*  @brief
*   管理命令行输入的历史记录
*
*  @attention
*
*
*  @copyright 2017 RoboMaster. All right reserved.
*
*/

#ifndef CLI_HISTORY_H
#define CLI_HISTORY_H

#include <string.h>
#include <stdint.h>

/* 保留N条历史记录 */
#define MAX_HISTORY_NUM     5
#define MAX_CMD_SIZE        64

typedef struct
{
    char str_buf[MAX_HISTORY_NUM][MAX_CMD_SIZE];
    int8_t write_idx;
    int8_t read_cnt;
    int8_t valid_cnt;
} cli_history_t;



/* 将新输入字符串存入历史数据 */
void history_save(cli_history_t *history, char *str);
/* 读取上一个历史记录 */
void history_get_last(cli_history_t *history, char *str, int *len);
/* 读取下一个历史记录 */
void history_get_next(cli_history_t *history, char *str, int *len);


#endif

