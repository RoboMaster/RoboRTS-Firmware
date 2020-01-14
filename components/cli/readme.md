命令行解析器，根据FreeRTOS中CLI组件封装而来：

1. 记录历史命令功能，通过上下键切换
2. help功能，输入help列出所有命令，输入help *cmd*，可查看指定命令帮助信息



API函数见`cli_interpreter.h`文件





使用示例见`cli_process.c`文件末尾：

```c
/********************************* cli_user_cmd.c ************************************/
/* 用户头文件 */
#include "cli_process.h"

/* 命令行密码 */
#define CLI_PWD     "pwd"


/************************************* 日志打印等级 ************************************/
int printd_command(char *write_buf, int buf_len, const char *cmd_str);

cli_cmd_t printd_cmd =
{
    .cmd_str = "printd",
    .help_str =
        "\r\nprintd\r\n"
        " change printd level:\r\n"
        " MUTE:0 EMERG:1 WARN:2 INFO:3 DEBUG:4\r\n",
	.cli_cmd_handler = printd_command,
	.expect_param_num = -1
};
int printd_command(char *write_buf, int buf_len, const char *cmd_str)
{
    const char *param[1];
    int param_len[1];
    int param_num;
    int ret;
    const char *level[] = {"MUTE", "EMERG", "WARN", "INFO", "DEBUG"};

    param[0] = cli_get_param(cmd_str, 1, &param_len[0]);
    cli_get_param_end(cmd_str);
    if(param[0] == NULL)
    {
        sprintf(write_buf, "current printd level is %s!\r\n", level[printd_level_get()]);
    }
    else
    {
        ret = str2int(&param_num, param[0]);
        if((ret == 0) && (param_num >= 0) && (param_num <= 4))
        {
            printd_level_set(param_num);
            sprintf(write_buf, "set printd level to %s!\r\n", level[printd_level_get()]);
        }
        else
        {
            strcpy(write_buf, "error: param should be a number in 0-4!\r\n");
        }
    }

    return 0;
}

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
    if(strcmp(param[0], CLI_PWD) == 0)
    {
        cli_cmd_register(&printd_cmd);
        strcpy(write_buf, "login success, welcome!\r\n");
    }
    else
    {
        strcpy(write_buf, "incorrect passwd!\r\n");
    }

    return 0;
}


void cli_cmd_init(void)
{
    cli_cmd_register(&pwd_cmd);
}


/*---------------------------------------------------------------------------------------------------------------------------*/

/********************************* cli_pthread.c ************************************/

#include "cli_process.h"

#define CMD_BUFSIZE     MAX_CMD_SIZE


static pthread_t tid_cli;
/* 内部函数申明 */
static void *pthread_cli(void *argc);


/* 创建任务 */
int thread_cli_init(void)
{
    if (pthread_create(&tid_cli, NULL, pthread_cli, NULL))
        err_sys("thread creat faild!");

    return 0;
}


int cli_send(char *out_str)
{
    return write(STDOUT_FILENO, out_str, strlen(out_str) + 1);
}


/****************************************  线程函数  ***************************************/

static void *pthread_cli(void *argc)
{
    char input_buf[CMD_BUFSIZE];
    int ret;

	/* The parameters are not used. */
    ( void ) argc;

    cli_cmd_init();

	while(1)
	{
        ret = read(STDIN_FILENO, input_buf, CMD_BUFSIZE);
        if(ret > 0)
        {
            cli_process(input_buf, ret, cli_send);
        }
        sleepms(50);
	}

    pthread_exit(NULL);
}

```

