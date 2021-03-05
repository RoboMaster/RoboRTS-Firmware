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

/* 用户头文件 */
#include "log.h"
#include "sys.h"
#include "cli_process.h"

#include "board.h"
#include "fifo.h"
#include "shell.h"

/* 命令行密码 */
#define CLI_PWD "robomaster"

/************************************** 系统密码 ************************************/
int pwd_command(char *write_buf, int buf_len, const char *cmd_str);

cli_cmd_t pwd_cmd =
{
    .cmd_str = "pwd",
    .help_str =
    "\r\npwd\r\n"
    " input password to login\r\n",
    .cli_cmd_handler = pwd_command,
    .expect_param_num = 1
};
int pwd_command(char *write_buf, int buf_len, const char *cmd_str)
{
    const char *param[1];
    int param_len[1];

    param[0] = cli_get_param(cmd_str, 1, &param_len[0]);
    cli_get_param_end(cmd_str);
    if (strcmp(param[0], CLI_PWD) == 0)
    {
        strcpy(write_buf, "login success, welcome!\r\n");
    }
    else
    {
        strcpy(write_buf, "incorrect passwd!\r\n");
    }

    return 0;
}

/************************************** 环境变量 ***********************************/
void ef_print_env(void);

int env_command(char *write_buf, int buf_len, const char *cmd_str);

cli_cmd_t env_cmd =
{
    .cmd_str = "env",
    .help_str =
    "\r\nenv\r\n"
    "env value prient\r\n",
    .cli_cmd_handler = env_command,
    .expect_param_num = 1
};
int env_command(char *write_buf, int buf_len, const char *cmd_str)
{
    const char *param[1];
    int param_len[1];

    param[0] = cli_get_param(cmd_str, 1, &param_len[0]);

    if (strcmp(param[0], "print") == 0)
    {
        ef_print_env();
    }
    else
    {
        strcpy(write_buf, "incorrect param!\r\n");
    }

    return 0;
}

void cli_cmd_init(void)
{
    cli_cmd_register(&pwd_cmd);
    cli_cmd_register(&env_cmd);
}

/*---------------------------------------------------------------------------------------------------------------------------*/

/********************************* cli_pthread.c ************************************/

#include "cli_process.h"

#define CMD_BUFSIZE MAX_CMD_SIZE

/* 内部函数申明 */
static void pthread_cli(void const *argc);

int cli_send(char *out_str)
{
    usart1_transmit((uint8_t *)out_str, strlen(out_str) + 1);
    return 0;
}

fifo_s_t shell_fifo;
char shell_buf[CMD_BUFSIZE];

void shell_interupt(uint8_t *buff, uint16_t len)
{
    fifo_s_puts_noprotect(&shell_fifo, (char *)buff, len);
}

osThreadId shell_task_t;

/* 创建任务 */
int thread_cli_init(void)
{
    fifo_s_init(&shell_fifo, shell_buf, CMD_BUFSIZE);
    osThreadDef(SHELL_TASK, pthread_cli, osPriorityNormal, 0, 512);
    shell_task_t = osThreadCreate(osThread(SHELL_TASK), NULL);
    return 0;
}

/****************************************   线程函数  ***************************************/
void pthread_cli(void const *argc)
{
    char input_buf[CMD_BUFSIZE];
    int ret;

    /* The parameters are not used. */
    (void)argc;

    cli_cmd_init();

    while (1)
    {
        uint16_t used_len;
        used_len = shell_fifo.used_num;
        ret = fifo_s_gets(&shell_fifo, input_buf, used_len);
        if (ret > 0)
        {
            cli_process(input_buf, ret, cli_send);
        }
        osDelay(50);
    }
}
