/* From FreeRTOS+CLI V1.0.4 */
/* Standard includes. */
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

/* Utils includes. */
#include "cli_interpreter.h"

typedef struct cmd_input_list
{
    const cli_cmd_t *cli_cmd;
    struct cmd_input_list *next;
} cli_list_item_t;

/*
 * The callback function that is executed when "help" is entered.  This is the
 * only default command that is always present.
 */
static int help_cmd(char *write_buf, int buf_len, const char *cmd_str);

/*
 * Return the number of parameters that follow the command name.
 */
static char get_param_num(const char *cmd_str);

/* The definition of the "help" command.  This command is always at the front
of the list of registered commands. */
static const cli_cmd_t cmd_help =
{
    .cmd_str = "help",
    .help_str =
    "\r\nhelp:\r\n"
    " lists all the registered commands\r\n",
    .cli_cmd_handler = help_cmd,
    .expect_param_num = -1
};

/* The definition of the list of commands.  Commands that are registered are
added to this list. */
static cli_list_item_t head_cmds =
{
    &cmd_help,  /* The first command in the list is always \
                        the help command, defined in this file. */
    NULL        /* The next pointer is initialised to NULL, \
                        as there are no other registered commands yet. */
};

/* A buffer into which command outputs can be written is declared here, rather
than in the command console implementation, to allow multiple command consoles
to share the same buffer.  For example, an application may allow access to the
command interpreter by UART and by Ethernet.  Sharing a buffer is done purely
to save RAM.  Note, however, that the command console itself is not re-entrant,
so only one command interpreter interface can be used at any one time.  For that
reason, no attempt at providing mutual exclusion to the output_buf array is
attempted.*/
static char output_buf[CLI_MAX_OUTPUT_SIZE];

/*-----------------------------------------------------------*/

int cli_cmd_register(const cli_cmd_t *const cli_cmd)
{
    static cli_list_item_t *last_cmd = &head_cmds;
    cli_list_item_t *new_list_item;
    int ret = -1;

    /* Check the parameter is not NULL. */
    CLI_ASSERT(cli_cmd);

    /* Create a new list item that will reference the command being registered. */
    new_list_item = (cli_list_item_t *)malloc(sizeof(cli_list_item_t));
    CLI_ASSERT(new_list_item);

    if (new_list_item != NULL)
    {
        // ENTER_CRITICAL();
        {
            /* Reference the command being registered from the newly created
            list item. */
            new_list_item->cli_cmd = cli_cmd;

            /* The new list item will get added to the end of the list, so
            next has nowhere to point. */
            new_list_item->next = NULL;

            /* Add the newly created list item to the end of the already existing
            list. */
            last_cmd->next = new_list_item;

            /* Set the end of list marker to the new list item. */
            last_cmd = new_list_item;
        }
        // EXIT_CRITICAL();

        ret = 0;
    }

    return ret;
}
/*-----------------------------------------------------------*/

int cli_cmd_process(const char *const cmd_input, char *write_buf, int buf_len)
{
    static const cli_list_item_t *list_item = NULL;
    int ret = 1;
    const char *cmd_str;
    int cmd_str_len;

    /* Note:  This function is not re-entrant.  It must not be called from more
    than one task. */

    if (list_item == NULL)
    {
        /* Search for the command string in the list of registered commands. */
        for (list_item = &head_cmds; list_item != NULL; list_item = list_item->next)
        {
            cmd_str = list_item->cli_cmd->cmd_str;
            cmd_str_len = strlen(cmd_str);

            /* To ensure the string lengths match exactly, so as not to pick up
            a sub-string of a longer command, check the byte after the expected
            end of the string is either the end of the string or a space before
            a parameter. */
            if ((cmd_input[cmd_str_len] == ' ') || (cmd_input[cmd_str_len] == '\0'))
            {
                if (strncmp(cmd_input, cmd_str, cmd_str_len) == 0)
                {
                    /* The command has been found.  Check it has the expected
                    number of parameters.  If expect_param_num is -1,
                    then there could be a variable number of parameters and no
                    check is made. */
                    if (list_item->cli_cmd->expect_param_num >= 0)
                    {
                        if (get_param_num(cmd_input) != list_item->cli_cmd->expect_param_num)
                        {
                            ret = 0;
                        }
                    }
                    break;
                }
            }
        }
    }

    if ((list_item != NULL) && (ret == 0))
    {
        const char tip[] = "incorrect command parameter(s).\r\n";
        /* The command was found, but the number of parameters with the command
        was incorrect. */
        strncpy(write_buf, tip, sizeof(tip));
        strncpy(write_buf + sizeof(tip) - 1, list_item->cli_cmd->help_str, buf_len - sizeof(tip));
        list_item = NULL;
    }
    else if (list_item != NULL)
    {
        /* Call the callback function that is registered to this command. */
        *write_buf = '\0';
        ret = list_item->cli_cmd->cli_cmd_handler(write_buf, buf_len, cmd_input);

        /* If ret is 0, then no further strings will be returned
        after this one, and list_item can be reset to NULL ready to search
        for the next entered command. */
        if (ret == 0)
        {
            list_item = NULL;
        }
    }
    else
    {
        /* list_item was NULL, the command was not found. */
        strncpy(write_buf, "command not recognised.\r\n"
                "enter 'help' to view a list of available commands.\r\n", buf_len);
        ret = 0;
    }

    return ret;
}
/*-----------------------------------------------------------*/

char *cli_get_output_buf(void)
{
    return output_buf;
}
/*-----------------------------------------------------------*/

const char *cli_get_param(const char *cmd_str, int wanted_param, int *param_str_len)
{
    int param_found = 0;
    int str_len = 0;
    const char *ret = NULL;


    while (param_found < wanted_param)
    {
        /* Index the character pointer past the current word.  If this is the start
        of the command string then the first word is the command itself. */
        while (((*cmd_str) != '\0') && ((*cmd_str) != ' '))
        {
            cmd_str++;
        }

        /* Find the start of the next string. */
        while (((*cmd_str) != '\0') && ((*cmd_str) == ' '))
        {
            cmd_str++;
        }

        /* Was a string found? */
        if (*cmd_str != '\0')
        {
            /* Is this the start of the required parameter? */
            param_found++;

            if (param_found == wanted_param)
            {
                /* How long is the parameter? */
                ret = cmd_str;
                while (((*cmd_str) != '\0') && ((*cmd_str) != ' '))
                {
                    str_len++;
                    cmd_str++;
                }

                if (str_len == 0)
                {
                    ret = NULL;
                }

                break;
            }
        }
        else
        {
            break;
        }
    }

    if (param_str_len != NULL)
    {
        *param_str_len = str_len;
    }

    return ret;
}
/*-----------------------------------------------------------*/

int cli_get_param_end(const char *cmd_str)
{
    char *str = (char *)cmd_str;

    if (str == NULL)
    {
        return -1;
    }

    while (*str != '\0')
    {
        if (*str == ' ')
        {
            *str = '\0';
        }
        str++;
    }

    return 0;
}
/*-----------------------------------------------------------*/

static int help_cmd(char *write_buf, int buf_len, const char *cmd_str)
{
    static const cli_list_item_t *list_item = NULL;
    static int once_flag = 0;
    const char *param;
    int ret;

    /* Find the specified cmd help string. */
    if (once_flag == 0)
    {
        param = cli_get_param(cmd_str, 1, NULL);
        if (param != NULL)
        {
            list_item = &head_cmds;
            while (list_item != NULL)
            {
                if (strcmp(param, list_item->cli_cmd->cmd_str) == 0)
                {
                    strncpy(write_buf, list_item->cli_cmd->help_str, buf_len);
                    list_item = NULL;
                    return 0;
                }
                list_item = list_item->next;
            }
            strncpy(write_buf, "cannot find the command help string.\r\n"
                    "enter 'help' to view a list of available commands.\r\n\r\n", buf_len);
            return 0;
        }
        once_flag = 1;
    }

    if (list_item == NULL)
    {
        /* Reset the list_item pointer back to the start of the list. */
        list_item = &head_cmds;
    }

    /* Return the next command help string, before moving the pointer on to
    the next command in the list. */
    strncpy(write_buf, list_item->cli_cmd->help_str, buf_len);
    list_item = list_item->next;

    if (list_item == NULL)
    {
        /* There are no more commands in the list, so there will be no more
        strings to return after this one and 0 should be returned. */
        once_flag = 0;
        ret = 0;
    }
    else
    {
        ret = 1;
    }

    return ret;
}
/*-----------------------------------------------------------*/

static char get_param_num(const char *cmd_str)
{
    char param_num = 0;
    int last_was_space = 0;

    /* Count the number of space delimited words in cmd_str. */
    while (*cmd_str != '\0')
    {
        if ((*cmd_str) == ' ')
        {
            if (last_was_space != 1)
            {
                param_num++;
                last_was_space = 1;
            }
        }
        else
        {
            last_was_space = 0;
        }

        cmd_str++;
    }

    /* If the command string ended with spaces, then there will have been too
    many parameters counted. */
    if (last_was_space == 1)
    {
        param_num--;
    }

    /* The value returned is one less than the number of space delimited words,
    as the first word should be the command itself. */
    return param_num;
}
