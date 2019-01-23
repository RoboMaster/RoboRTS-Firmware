/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-08-25     armink       the first version
 */

#include <stdarg.h>
#include "ulog.h"

#ifdef ULOG_OUTPUT_FLOAT
#include <stdio.h>
#endif

#ifdef ULOG_USING_COLOR
/**
 * CSI(Control Sequence Introducer/Initiator) sign
 * more information on https://en.wikipedia.org/wiki/ANSI_escape_code
 */
#define CSI_START                      "\033["
#define CSI_END                        "\033[0m"
/* output log front color */
#define F_BLACK                        "30m"
#define F_RED                          "31m"
#define F_GREEN                        "32m"
#define F_YELLOW                       "33m"
#define F_BLUE                         "34m"
#define F_MAGENTA                      "35m"
#define F_CYAN                         "36m"
#define F_WHITE                        "37m"

/* output log default color definition */
#ifndef ULOG_COLOR_DEBUG
#define ULOG_COLOR_DEBUG               NULL
#endif
#ifndef ULOG_COLOR_INFO
#define ULOG_COLOR_INFO                (F_GREEN)
#endif
#ifndef ULOG_COLOR_WARN
#define ULOG_COLOR_WARN                (F_YELLOW)
#endif
#ifndef ULOG_COLOR_ERROR
#define ULOG_COLOR_ERROR               (F_RED)
#endif
#ifndef ULOG_COLOR_ASSERT
#define ULOG_COLOR_ASSERT              (F_MAGENTA)
#endif
#endif /* ULOG_USING_COLOR */

#if ULOG_LINE_BUF_SIZE < 80
#error "the log line buffer size must more than 80"
#endif

struct rt_ulog
{
    ulog_bool_t init_ok;
    /* all backends */
    list_t backend_list;
    /* the thread log's line buffer */
    char log_buf[ULOG_LINE_BUF_SIZE];

#ifdef ULOG_USING_FILTER
    struct
    {
        /* all tag's level filter */
        list_t tag_lvl_list;
        /* global filter level, tag and keyword */
        uint32_t level;
        char tag[ULOG_FILTER_TAG_MAX_LEN + 1];
        char keyword[ULOG_FILTER_KW_MAX_LEN + 1];
    } filter;
#endif /* ULOG_USING_FILTER */
};

/* level output info */
static const char * const level_output_info[] =
{
        "A/",
        NULL,
        NULL,
        "E/",
        "W/",
        NULL,
        "I/",
        "D/",
};

#ifdef ULOG_USING_COLOR
/* color output info */
static const char * const color_output_info[] =
{
        ULOG_COLOR_ASSERT,
        NULL,
        NULL,
        ULOG_COLOR_ERROR,
        ULOG_COLOR_WARN,
        NULL,
        ULOG_COLOR_INFO,
        ULOG_COLOR_DEBUG,
};
#endif /* ULOG_USING_COLOR */

/* ulog local object */
static struct rt_ulog ulog = { 0 };

size_t ulog_strcpy(size_t cur_len, char *dst, const char *src)
{
    const char *src_old = src;

    ULOG_ASSERT(dst);
    ULOG_ASSERT(src);

    while (*src != 0)
    {
        /* make sure destination has enough space */
        if (cur_len++ <= ULOG_LINE_BUF_SIZE)
        {
            *dst++ = *src++;
        }
        else
        {
            break;
        }
    }
    return src - src_old;
}

size_t ulog_ultoa(char *s, unsigned long int n)
{
    size_t i = 0, j = 0, len = 0;
    char swap;

    do
    {
        s[len++] = n % 10 + '0';
    } while (n /= 10);
    s[len] = '\0';
    /* reverse string */
    for (i = 0, j = len - 1; i < j; ++i, --j)
    {
        swap = s[i];
        s[i] = s[j];
        s[j] = swap;
    }
    return len;
}

static char *get_log_buf(void)
{
   return ulog.log_buf;
}

uint32_t ulog_formater(char *log_buf, uint32_t level, const char *tag, ulog_bool_t newline,
        const char *format, va_list args)
{
    /* the caller has locker, so it can use static variable for reduce stack usage */
    static ulog_size_t log_len, newline_len;
    static int fmt_result;

    ULOG_ASSERT(log_buf);
    ULOG_ASSERT(level <= LOG_LVL_DBG);
    ULOG_ASSERT(tag);
    ULOG_ASSERT(format);

    log_len = 0;
    newline_len = strlen(ULOG_NEWLINE_SIGN);

#ifdef ULOG_USING_COLOR
    /* add CSI start sign and color info */
    if (color_output_info[level])
    {
        log_len += ulog_strcpy(log_len, log_buf + log_len, CSI_START);
        log_len += ulog_strcpy(log_len, log_buf + log_len, color_output_info[level]);
    }
#endif /* ULOG_USING_COLOR */

#ifdef ULOG_OUTPUT_TIME
    /* add time info */
    {
        snprintf(log_buf + log_len, ULOG_LINE_BUF_SIZE - log_len, "[%d.%03d]", ulog_get_time_ms()/1000, ulog_get_time_ms()%1000);
        log_len += strlen(log_buf + log_len);
    }
#endif /* ULOG_OUTPUT_TIME */

#ifdef ULOG_OUTPUT_LEVEL

#ifdef ULOG_OUTPUT_TIME
    log_len += ulog_strcpy(log_len, log_buf + log_len, " ");
#endif

    /* add level info */
    log_len += ulog_strcpy(log_len, log_buf + log_len, level_output_info[level]);
#endif /* ULOG_OUTPUT_LEVEL */

#ifdef ULOG_OUTPUT_TAG

#if !defined(ULOG_OUTPUT_LEVEL) && defined(ULOG_OUTPUT_TIME)
    log_len += ulog_strcpy(log_len, log_buf + log_len, " ");
#endif

    /* add tag info */
    log_len += ulog_strcpy(log_len, log_buf + log_len, tag);
#endif /* ULOG_OUTPUT_TAG */

    log_len += ulog_strcpy(log_len, log_buf + log_len, ": ");

#ifdef ULOG_OUTPUT_FLOAT
    fmt_result = vsnprintf(log_buf + log_len, ULOG_LINE_BUF_SIZE - log_len, format, args);
#else
    fmt_result = rt_vsnprintf(log_buf + log_len, ULOG_LINE_BUF_SIZE - log_len, format, args);
#endif /* ULOG_OUTPUT_FLOAT */

    /* calculate log length */
    if ((log_len + fmt_result <= ULOG_LINE_BUF_SIZE) && (fmt_result > -1))
    {
        log_len += fmt_result;
    }
    else
    {
        /* using max length */
        log_len = ULOG_LINE_BUF_SIZE;
    }

    /* overflow check and reserve some space for CSI end sign and newline sign */
#ifdef ULOG_USING_COLOR
    if (log_len + (sizeof(CSI_END) - 1) + newline_len > ULOG_LINE_BUF_SIZE)
    {
        /* using max length */
        log_len = ULOG_LINE_BUF_SIZE;
        /* reserve some space for CSI end sign */
        log_len -= (sizeof(CSI_END) - 1);
#else
    if (log_len + newline_len > ULOG_LINE_BUF_SIZE)
    {
        /* using max length */
        log_len = ULOG_LINE_BUF_SIZE;
#endif /* ULOG_USING_COLOR */
        /* reserve some space for newline sign */
        log_len -= newline_len;
    }

    /* package newline sign */
    if (newline)
    {
        log_len += ulog_strcpy(log_len, log_buf + log_len, ULOG_NEWLINE_SIGN);
    }

#ifdef ULOG_USING_COLOR
    /* add CSI end sign  */
    if (color_output_info[level])
    {
        log_len += ulog_strcpy(log_len, log_buf + log_len, CSI_END);
    }
#endif /* ULOG_USING_COLOR */

    return log_len;
}

void ulog_output_to_all_backend(uint32_t level, const char *tag, ulog_bool_t is_raw, const char *log, ulog_size_t size)
{
    list_t *node;
    ulog_backend_t backend;

    if (!ulog.init_ok)
        return;

    /* output for all backends */
    for (node = ulog.backend_list.next; node != &ulog.backend_list; node = node->next)
    {
        backend = list_entry(node, struct ulog_backend, list);
#if !defined(ULOG_USING_COLOR)
        backend->output(backend, level, tag, is_raw, log, size);
#else
        if (backend->support_color)
        {
            backend->output(backend, level, tag, is_raw, log, size);
        }
        else
        {
            /* recalculate the log start address and log size when backend not supported color */
            ulog_size_t color_info_len = strlen(color_output_info[level]);
            if (color_info_len)
            {
                ulog_size_t color_hdr_len = strlen(CSI_START) + color_info_len;

                log += color_hdr_len;
                size -= (color_hdr_len + (sizeof(CSI_END) - 1));
            }
            backend->output(backend, level, tag, is_raw, log, size);
        }
#endif /* !defined(ULOG_USING_COLOR) || defined(ULOG_USING_SYSLOG) */
    }
}

static void do_output(uint32_t level, const char *tag, ulog_bool_t is_raw, const char *log_buf, ulog_size_t log_len)
{
    /* output to all backends */
    ulog_output_to_all_backend(level, tag, is_raw, log_buf, log_len);
}

/**
 * output the log by variable argument list
 *
 * @param level level
 * @param tag tag
 * @param newline has_newline
 * @param format output format
 * @param args variable argument list
 */
void ulog_voutput(uint32_t level, const char *tag, ulog_bool_t newline, const char *format, va_list args)
{
    char *log_buf = NULL;
    ulog_size_t log_len = 0;

    var_cpu_sr();

#ifndef ULOG_USING_SYSLOG
    ULOG_ASSERT(level <= LOG_LVL_DBG);
#else
    ULOG_ASSERT(LOG_PRI(level) <= LOG_DEBUG);
#endif /* ULOG_USING_SYSLOG */

    ULOG_ASSERT(tag);
    ULOG_ASSERT(format);

    if (!ulog.init_ok)
    {
        return;
    }

#ifdef ULOG_USING_FILTER
    /* level filter */
    if (level > ulog.filter.level || level > ulog_tag_lvl_filter_get(tag))
    {
        return;
    }
    else if (!strstr(tag, ulog.filter.tag))
    {
        /* tag filter */
        return;
    }
#endif /* ULOG_USING_FILTER */

    /* get log buffer */
    log_buf = get_log_buf();

    /* lock output */
    enter_critical();

    log_len = ulog_formater(log_buf, level, tag, newline, format, args);

#ifdef ULOG_USING_FILTER
    /* keyword filter */
    if (ulog.filter.keyword[0] != '\0')
    {
        /* add string end sign */
        log_buf[log_len] = '\0';
        /* find the keyword */
        if (!strstr(log_buf, ulog.filter.keyword))
        {
            /* unlock output */
            exit_critical();
            return;
        }
    }
#endif /* ULOG_USING_FILTER */
    /* do log output */
    do_output(level, tag, FALSE, log_buf, log_len);

    /* unlock output */
    exit_critical();
}

/**
 * output the log
 *
 * @param level level
 * @param tag tag
 * @param newline has newline
 * @param format output format
 * @param ... args
 */
void ulog_output(uint32_t level, const char *tag, ulog_bool_t newline, const char *format, ...)
{
    va_list args;

    /* args point to the first variable parameter */
    va_start(args, format);

    ulog_voutput(level, tag, newline, format, args);

    va_end(args);
}

/**
 * output RAW string format log
 *
 * @param format output format
 * @param ... args
 */
void ulog_raw(const char *format, ...)
{
    ulog_size_t log_len = 0;
    char *log_buf = NULL;
    va_list args;
    int fmt_result;
    var_cpu_sr();

    ULOG_ASSERT(ulog.init_ok);

    /* get log buffer */
    log_buf = get_log_buf();

    /* lock output */
    enter_critical();
    /* args point to the first variable parameter */
    va_start(args, format);

#ifdef ULOG_OUTPUT_FLOAT
    fmt_result = vsnprintf(log_buf, ULOG_LINE_BUF_SIZE, format, args);
#else
    fmt_result = rt_vsnprintf(log_buf, ULOG_LINE_BUF_SIZE, format, args);
#endif /* ULOG_OUTPUT_FLOAT */

    va_end(args);

    /* calculate log length */
    if ((fmt_result > -1) && (fmt_result <= ULOG_LINE_BUF_SIZE))
    {
        log_len = fmt_result;
    }
    else
    {
        log_len = ULOG_LINE_BUF_SIZE;
    }

    /* do log output */
    do_output(LOG_LVL_DBG, NULL, TRUE, log_buf, log_len);

    /* unlock output */
    exit_critical();
}

/**
 * dump the hex format data to log
 *
 * @param tag name for hex object, it will show on log header
 * @param width hex number for every line, such as: 16, 32
 * @param buf hex buffer
 * @param size buffer size
 */
void ulog_hexdump(const char *tag, ulog_size_t width, uint8_t *buf, ulog_size_t size)
{
#define __is_print(ch)       ((unsigned int)((ch) - ' ') < 127u - ' ')

    ulog_size_t i, j;
    ulog_size_t log_len = 0, name_len = strlen(tag);
    char *log_buf = NULL, dump_string[8];
    int fmt_result;
    var_cpu_sr();

    ULOG_ASSERT(ulog.init_ok);

#ifdef ULOG_USING_FILTER
    /* level filter */
#ifndef ULOG_USING_SYSLOG
    if (LOG_LVL_DBG > ulog.filter.level || LOG_LVL_DBG > ulog_tag_lvl_filter_get(tag))
    {
        return;
    }
#else
    if ((LOG_MASK(LOG_DEBUG) & ulog.filter.level) == 0)
    {
        return;
    }
#endif /* ULOG_USING_SYSLOG */
    else if (!strstr(tag, ulog.filter.tag))
    {
        /* tag filter */
        return;
    }
#endif /* ULOG_USING_FILTER */

    /* get log buffer */
    log_buf = get_log_buf();

    /* lock output */
    enter_critical();

    for (i = 0, log_len = 0; i < size; i += width)
    {
        /* package header */
        if (i == 0)
        {
            log_len += ulog_strcpy(log_len, log_buf + log_len, "D/HEX ");
            log_len += ulog_strcpy(log_len, log_buf + log_len, tag);
            log_len += ulog_strcpy(log_len, log_buf + log_len, ": ");
        }
        else
        {
            log_len = 6 + name_len + 2;
            memset(log_buf, ' ', log_len);
        }
        fmt_result = snprintf(log_buf + log_len, ULOG_LINE_BUF_SIZE, "%04X-%04X: ", i, i + width);
        /* calculate log length */
        if ((fmt_result > -1) && (fmt_result <= ULOG_LINE_BUF_SIZE))
        {
            log_len += fmt_result;
        }
        else
        {
            log_len = ULOG_LINE_BUF_SIZE;
        }
        /* dump hex */
        for (j = 0; j < width; j++)
        {
            if (i + j < size)
            {
                snprintf(dump_string, sizeof(dump_string), "%02X ", buf[i + j]);
            }
            else
            {
                strncpy(dump_string, "   ", sizeof(dump_string));
            }
            log_len += ulog_strcpy(log_len, log_buf + log_len, dump_string);
            if ((j + 1) % 8 == 0)
            {
                log_len += ulog_strcpy(log_len, log_buf + log_len, " ");
            }
        }
        log_len += ulog_strcpy(log_len, log_buf + log_len, "  ");
        /* dump char for hex */
        for (j = 0; j < width; j++)
        {
            if (i + j < size)
            {
                snprintf(dump_string, sizeof(dump_string), "%c", __is_print(buf[i + j]) ? buf[i + j] : '.');
                log_len += ulog_strcpy(log_len, log_buf + log_len, dump_string);
            }
        }
        /* overflow check and reserve some space for newline sign */
        if (log_len + strlen(ULOG_NEWLINE_SIGN) > ULOG_LINE_BUF_SIZE)
        {
            log_len = ULOG_LINE_BUF_SIZE - strlen(ULOG_NEWLINE_SIGN);
        }
        /* package newline sign */
        log_len += ulog_strcpy(log_len, log_buf + log_len, ULOG_NEWLINE_SIGN);
        /* do log output */
        do_output(LOG_LVL_DBG, NULL, TRUE, log_buf, log_len);
    }
    /* unlock output */
    exit_critical();
}

#ifdef ULOG_USING_FILTER
/**
 * Set the filter's level by different tag.
 * The log on this tag which level is less than it will stop output.
 *
 * example:
 *     // the example tag log enter silent mode
 *     ulog_set_filter_lvl("example", LOG_FILTER_LVL_SILENT);
 *     // the example tag log which level is less than INFO level will stop output
 *     ulog_set_filter_lvl("example", LOG_LVL_INFO);
 *     // remove example tag's level filter, all level log will resume output
 *     ulog_set_filter_lvl("example", LOG_FILTER_LVL_ALL);
 *
 * @param tag log tag
 * @param level The filter level. When the level is LOG_FILTER_LVL_SILENT, the log enter silent mode.
 *        When the level is LOG_FILTER_LVL_ALL, it will remove this tag's level filer.
 *        Then all level log will resume output.
 *
 * @return  0 : success
 *         -5 : no memory
 *         -10: level is out of range
 */
int ulog_tag_lvl_filter_set(const char *tag, uint32_t level)
{
    list_t *node;
    ulog_tag_lvl_filter_t tag_lvl = NULL;
    int result = RM_OK;
    var_cpu_sr();

    if (level > LOG_FILTER_LVL_ALL)
        return -RM_INVAL;

    if (!ulog.init_ok)
        return result;

    /* lock output */
    enter_critical();
    /* find the tag in list */
    for (node = ulog_tag_lvl_list_get()->next; node != ulog_tag_lvl_list_get(); node = node->next)
    {
        tag_lvl = list_entry(node, struct ulog_tag_lvl_filter, list);
        if (!strncmp(tag_lvl->tag, tag, ULOG_FILTER_TAG_MAX_LEN))
        {
            break;
        }
        else
        {
            tag_lvl = NULL;
        }
    }
    /* find OK */
    if (tag_lvl)
    {
        if (level == LOG_FILTER_LVL_ALL)
        {
            /* remove current tag's level filter when input level is the lowest level */
            list_del(&tag_lvl->list);
            heap_free(tag_lvl);
        }
        else
        {
            /* update level */
            tag_lvl->level = level;
        }
    }
    else
    {
        /* only add the new tag's level filer when level is not LOG_FILTER_LVL_ALL */
        if (level != LOG_FILTER_LVL_ALL)
        {
            /* new a tag's level filter */
            tag_lvl = (ulog_tag_lvl_filter_t)heap_malloc(sizeof(struct ulog_tag_lvl_filter));
            if (tag_lvl)
            {
                memset(tag_lvl->tag, 0 , sizeof(tag_lvl->tag));
                strncpy(tag_lvl->tag, tag, ULOG_FILTER_TAG_MAX_LEN);
                tag_lvl->level = level;
                list_add(&tag_lvl->list, ulog_tag_lvl_list_get());
            }
            else
            {
                result = -RM_NOMEM;
            }
        }
    }
    /* unlock output */
    exit_critical();

    return result;
}

/**
 * get the level on tag's level filer
 *
 * @param tag log tag
 *
 * @return It will return the lowest level when tag was not found.
 *         Other level will return when tag was found.
 */
uint32_t ulog_tag_lvl_filter_get(const char *tag)
{
    list_t *node;
    ulog_tag_lvl_filter_t tag_lvl = NULL;
    uint32_t level = LOG_FILTER_LVL_ALL;
    var_cpu_sr();

    if (!ulog.init_ok)
        return level;

    /* lock output */
    enter_critical();
    /* find the tag in list */
    for (node = ulog_tag_lvl_list_get()->next; node != ulog_tag_lvl_list_get(); node = node->next)
    {
        tag_lvl = list_entry(node, struct ulog_tag_lvl_filter, list);
        if (!strncmp(tag_lvl->tag, tag, ULOG_FILTER_TAG_MAX_LEN))
        {
            level = tag_lvl->level;
            break;
        }
    }
    /* unlock output */
    exit_critical();

    return level;
}

/**
 * get the tag's level list on filter
 *
 * @return tag's level list
 */
list_t *ulog_tag_lvl_list_get(void)
{
    return &ulog.filter.tag_lvl_list;
}

/**
 * set log global filter level
 *
 * @param level log level: LOG_LVL_ASSERT, LOG_LVL_ERROR, LOG_LVL_WARNING, LOG_LVL_INFO, LOG_LVL_DBG
 *              LOG_FILTER_LVL_SILENT: disable all log output, except assert level
 *              LOG_FILTER_LVL_ALL: enable all log output
 */
void ulog_global_filter_lvl_set(uint32_t level)
{
    ULOG_ASSERT(level <= LOG_FILTER_LVL_ALL);

    ulog.filter.level = level;
}

/**
 * get log global filter level
 *
 * @return log level: LOG_LVL_ASSERT, LOG_LVL_ERROR, LOG_LVL_WARNING, LOG_LVL_INFO, LOG_LVL_DBG
 *              LOG_FILTER_LVL_SILENT: disable all log output, except assert level
 *              LOG_FILTER_LVL_ALL: enable all log output
 */
uint32_t ulog_global_filter_lvl_get(void)
{
    return ulog.filter.level;
}

/**
 * set log global filter tag
 *
 * @param tag tag
 */
void ulog_global_filter_tag_set(const char *tag)
{
    ULOG_ASSERT(tag);

    strncpy(ulog.filter.tag, tag, ULOG_FILTER_TAG_MAX_LEN);
}

/**
 * get log global filter tag
 *
 * @return tag
 */
const char *ulog_global_filter_tag_get(void)
{
    return ulog.filter.tag;
}

/**
 * set log global filter keyword
 *
 * @param keyword keyword
 */
void ulog_global_filter_kw_set(const char *keyword)
{
    ULOG_ASSERT(keyword);

    strncpy(ulog.filter.keyword, keyword, ULOG_FILTER_KW_MAX_LEN);
}

/**
 * get log global filter keyword
 *
 * @return keyword
 */
const char *ulog_global_filter_kw_get(void)
{
    return ulog.filter.keyword;
}

int32_t ulog_backend_register(ulog_backend_t backend, const char *name, ulog_bool_t support_color)
{
    var_cpu_sr();

    ULOG_ASSERT(backend);
    ULOG_ASSERT(name);
    ULOG_ASSERT(ulog.init_ok);
    ULOG_ASSERT(backend->output);

    if (backend->init)
    {
        backend->init(backend);
    }

    backend->support_color = support_color;
    memcpy(backend->name, name, ULOG_NAME_MAX_NUM);

    enter_critical();
    list_add(&backend->list, &ulog.backend_list);
    exit_critical();

    return RM_OK;
}
#endif /* ULOG_USING_FILTER */

int32_t ulog_backend_unregister(ulog_backend_t backend)
{
    var_cpu_sr();

    ULOG_ASSERT(backend);
    ULOG_ASSERT(ulog.init_ok);

    if (backend->deinit)
    {
        backend->deinit(backend);
    }

    enter_critical();
    list_del(&ulog.backend_list);
    exit_critical();

    return RM_OK;
}

/**
 * flush all backends's log
 */
void ulog_flush(void)
{
    list_t *node;
    ulog_backend_t backend;

    if (!ulog.init_ok)
        return;

    /* flush all backends */
    for (node = (&ulog.backend_list)->next; node != &ulog.backend_list; node = node->next)
    {
        backend = list_entry(node, struct ulog_backend, list);
        if (backend->flush)
        {
            backend->flush(backend);
        }
    }
}

int ulog_init(void)
{
    if (ulog.init_ok)
        return 0;
    
    INIT_LIST_HEAD(&ulog.backend_list);

#ifdef ULOG_USING_FILTER
    INIT_LIST_HEAD(ulog_tag_lvl_list_get());
#endif

#ifdef ULOG_USING_FILTER
    ulog_global_filter_lvl_set(LOG_FILTER_LVL_ALL);
#endif

    ulog.init_ok = TRUE;

    return 0;
}

void ulog_deinit(void)
{
    list_t *node;
    ulog_backend_t backend;

    if (!ulog.init_ok)
        return;

    /* deinit all backends */
    for (node = (&ulog.backend_list)->next; node != &ulog.backend_list; node = node->next)
    {
        backend = list_entry(node, struct ulog_backend, list);
        if (backend->deinit)
        {
            backend->deinit(backend);
        }
    }

#ifdef ULOG_USING_FILTER
    /* deinit tag's level filter */
    {
        ulog_tag_lvl_filter_t tag_lvl;
         for (node = (ulog_tag_lvl_list_get())->next; node != ulog_tag_lvl_list_get(); node = node->next)
        {
            tag_lvl = list_entry(node, struct ulog_tag_lvl_filter, list);
            heap_free(tag_lvl);
        }
    }
#endif /* ULOG_USING_FILTER */

    ulog.init_ok = FALSE;
}

