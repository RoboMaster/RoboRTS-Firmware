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


#include "cli_history.h"

/* 将新输入字符串存入历史数据 */
void history_save(cli_history_t *history, char *str)
{
    int tmp_idx;

    tmp_idx = history->write_idx - 1;
    if (tmp_idx < 0)
    {
        tmp_idx = history->valid_cnt - 1;
    }
    /* 只有和上次命令不一样才存入历史记录 */
    if ((history->valid_cnt == 0) || strcmp(history->str_buf[tmp_idx], str))
    {
        strncpy(history->str_buf[history->write_idx], str, MAX_CMD_SIZE);
        history->str_buf[history->write_idx][MAX_CMD_SIZE - 1] = '\0';
        history->write_idx ++;
        /* 更新有效个数 */
        if (history->valid_cnt < history->write_idx)
        {
            history->valid_cnt = history->write_idx;
        }
        if (history->write_idx >= MAX_HISTORY_NUM)
        {
            history->write_idx = 0;
        }
    }
    history->read_cnt = 0;
}

/* 读取上一个历史记录 */
void history_get_last(cli_history_t *history, char *str, int *len)
{
    int tmp_idx;

    /* 限定范围内读取记录 */
    if (history->read_cnt < history->valid_cnt)
    {
        history->read_cnt ++;
        tmp_idx = history->write_idx - history->read_cnt;
        if (tmp_idx < 0)
        {
            tmp_idx += history->valid_cnt;
        }
        *len = strlen(history->str_buf[tmp_idx]);
        strcpy(str, history->str_buf[tmp_idx]);
    }
}


/* 读取下一个历史记录 */
void history_get_next(cli_history_t *history, char *str, int *len)
{
    int tmp_idx;

    /* 限定范围内读取记录 */
    if (history->read_cnt > 0)
    {
        history->read_cnt --;
        if (history->read_cnt == 0)
        {
            /* 当没有下一个历史记录时，清空当前字符串 */
            *str = '\0';
            *len = 0;
            return;
        }
        tmp_idx = history->write_idx - history->read_cnt;
        if (tmp_idx < 0)
        {
            tmp_idx += history->valid_cnt;
        }
        *len = strlen(history->str_buf[tmp_idx]);
        strcpy(str, history->str_buf[tmp_idx]);
    }
}



