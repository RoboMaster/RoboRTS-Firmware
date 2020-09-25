/**
*  @file cli_process.c/h
*  @version 1.0
*  @date 2018-06-12
*  @author sky
*
*  @brief
*   处理命令字符和键盘交互，外部只需调用 cli_process 即可
*
*  @attention
*
*  @copyright 2018 RoboMaster. All right reserved.
*
*/

#include "cli_process.h"


static int input_idx = 0;
static char in_str[CLI_INPUT_SIZE];
static char out_str[CLI_OUTPUT_SIZE];
static char state = 0;
static cli_history_t cli_history;

/* 终端命令解析处理 */
void cli_process(char *str_buf, int str_len, cli_output_t out_fun)
{
    int str_idx = 0;
    int more_data_flag;
    char input_char;

    while (str_idx < str_len)
    {
        input_char = str_buf[str_idx];
        str_idx ++;
        /* 以回车键判断命令结束 */
        if ((input_char == '\r') || (input_char == '\n'))
        {
            out_fun("\r\n");
            in_str[input_idx] = '\0';
            if (input_idx > 0)
            {
                history_save(&cli_history, in_str);
                do
                {
                    /* 扫描命令 */
                    more_data_flag = cli_cmd_process(in_str, out_str, CLI_OUTPUT_SIZE);
                    out_fun(out_str);
                }
                while (more_data_flag);
            }
            input_idx = 0;
            out_fun("RoboMaster>");
        }
        else
        {
            /* 识别特殊字符串 */
            if (state != 0)
            {
                /* 向上箭头的键值 1B 5B 41, A,B,C,D分别对应上下右左 */
                switch (state)
                {
                case 1:
                    if (input_char == 0x5B)
                    {
                        state ++;
                    }
                    else
                    {
                        state = 0;
                    }
                    break;
                case 2:
                    if (input_char == 'A') /* 上箭头 */
                    {
                        backspace_n(input_idx, out_fun);
                        history_get_last(&cli_history, in_str, &input_idx);
                        out_fun(in_str);
                    }
                    else if (input_char == 'B') /* 下箭头 */
                    {
                        backspace_n(input_idx, out_fun);
                        history_get_next(&cli_history, in_str, &input_idx);
                        out_fun(in_str);
                    }
                    state = 0;
                    break;
                default:
                    state = 0;
                    break;
                }
            }
            else
            {
                switch (input_char)
                {
                case 0x1B:  //arrow key start
                {
                    state = 1;
                    break;
                }
                case '\b':  //backspace
                {
                    /* 退格处理 */
                    if (input_idx > 0)
                    {
                        input_idx--;
                        in_str[input_idx] = '\0';
                        backspace_n(1, out_fun);
                    }
                    break;
                }
                default:
                {
                    if ((input_char >= ' ') && (input_char <= '~'))
                    {
                        /* 只存储可打印字符到命令缓存 */
                        if (input_idx < CLI_INPUT_SIZE - 1)
                        {
                            in_str[input_idx++] = input_char;
                            out_str[0] = input_char;
                            out_str[1] = '\0';
                            out_fun(out_str);
                        }
                    }
                    break;
                }
                }
            }
        }
    }
}


/* 终端退格N个字符 */
void backspace_n(int n, cli_output_t out_fun)
{
    int i;

    for (i = 0; i < n; i ++)
    {
        out_fun("\b");
    }
    for (i = 0; i < n; i ++)
    {
        out_fun(" ");
    }
    for (i = 0; i < n; i ++)
    {
        out_fun("\b");
    }
}

/* m^n, m 的 n 次方 */
static int _power(int m, int n)
{
    int result = 1;

    while (n--)
    {
        result *= m;
    }
    return result;
}
/*
把字符串转为数字，支持负数和16进制转换,0x开头识别为16进制
参数：
    *str_in:数字字符串指针
    *int_out:转换完的结果存放地址.
返回值:
    0：成功转换完成.
    其他：1,数据格式错误.  2,16进制位数为0.  3,起始格式错误.  4,十进制位数为0.
 */
int str2int(int *int_out, const char *str_in)
{
    char *input_char;
    int tmp;
    char digit_cnt = 0;     /* 数字的位数 */
    char hex_dec = 10;      /* 默认为10进制 */
    char sym_flag = 0;      /* 0,没有符号标记;1,表示正数;2,表示负数 */

    input_char = (char *)str_in;
    *int_out = 0;   /* 清零 */

    while (1)
    {
        /* 转换为小写 */
        if (*input_char >= 'A' && *input_char <= 'F')
        {
            *input_char += 0x20;
        }

        /* 识别符号和位数 */
        if ((*input_char >= '0' && *input_char <= '9') || ((*str_in == '-' || *str_in == '+') && digit_cnt == 0) || \
                (*input_char >= 'a' && *input_char <= 'f') || (*input_char == 'x' && digit_cnt == 1))   /* 检查合法性 */
        {
            if (*input_char >= 'a')
            {
                hex_dec = 16;    /* 字符串中存在字母,为16进制格式 */
            }

            if (*str_in == '-')
            {
                str_in ++;
                sym_flag = 2;
            }
            else if (*str_in == '+')
            {
                str_in ++;
                sym_flag = 1;
            }
            else
            {
                digit_cnt++;
            }
        }
        else if (*input_char == '\0' || *input_char == ' ')
        {
            break;
        }
        else
        {
            return 1;    /* 不全是十进制或者16进制数据 */
        }
        input_char++;
    }

    /* 识别非法状态 */
    input_char = (char *)str_in; /* 重新定位到字符串开始的地址 */
    if (hex_dec == 16)
    {
        if (digit_cnt < 3)
        {
            return 2;    /* 位数小于3，直接退出.因为0x就占了2个 */
        }
        if (*input_char == '0' && (*(input_char + 1) == 'x'))
        {
            input_char += 2;
            digit_cnt -= 2;
        }
        else
        {
            return 3;    /* 没有识别出数字格式 */
        }
    }
    else if (digit_cnt == 0)
    {
        return 4;
    }


    /* 转换为数字 */
    while (1)
    {
        if (digit_cnt)
        {
            digit_cnt--;
        }
        if (*input_char <= '9' && *input_char >= '0')
        {
            tmp = *input_char - '0';
        }
        else
        {
            tmp = *input_char - 'a' + 10;
        }
        *int_out += tmp * _power(hex_dec, digit_cnt);
        input_char++;
        if (*input_char == '\0' || *input_char == ' ')
        {
            break;
        }
    }

    if (sym_flag == 2)
    {
        *int_out = *int_out * -1;
    }

    return 0;
}

