#ifndef __ULOG_CONFIG_H__
#define __ULOG_CONFIG_H__

#ifdef ULOG_CONFIG_H_GLOBAL
  #define ULOG_CONFIG_H_EXTERN 
#else
  #define ULOG_CONFIG_H_EXTERN extern
#endif

/* config */

#define ULOG_OUTPUT_LVL 7
#define ULOG_OUTPUT_FLOAT
#define ULOG_USING_COLOR
#define ULOG_USING_FILTER
#define ULOG_OUTPUT_TIME
#define ULOG_OUTPUT_LEVEL
#define ULOG_OUTPUT_TAG

#endif // __ULOG_CONFIG_H__
