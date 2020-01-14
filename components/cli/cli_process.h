/**
*  @file cli_process.c/h
*  @version 1.0
*  @date 2018-06-12
*  @author sky
*
*  @brief
*
*
*  @attention
*
*
*  @copyright 2018 RoboMaster. All right reserved.
*
*/

#ifndef CLI_PROCESS_H
#define CLI_PROCESS_H

#include <string.h>
#include <stdint.h>
#include "cli_interpreter.h"
#include "cli_history.h"


#define CLI_OUTPUT_SIZE     256             /* 执行命令时每次输出最大长度 */
#define CLI_INPUT_SIZE      MAX_CMD_SIZE    /* 输入命令最大长度 */


typedef int (*cli_output_t)(char *out_buf);

void cli_process(char *str_buf, int str_len, cli_output_t out_fun);
void backspace_n(int n, cli_output_t out_fun);
int str2int(int *int_out, const char *str_in);



#endif //CLI_PROCESS_H
