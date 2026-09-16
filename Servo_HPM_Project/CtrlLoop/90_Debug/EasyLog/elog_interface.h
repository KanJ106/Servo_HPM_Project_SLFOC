#ifndef __ELOG_INTERFACE_H
#define __ELOG_INTERFACE_H

#include "elog.h"

#define USER_RTT_VIEW 1

#if !defined(LOG_TAG)
    #define LOG_TAG                    "main"
#endif
#undef LOG_LVL
#if defined(XX_LOG_LVL)
    #define LOG_LVL                    XX_LOG_LVL
#endif

#define ELOG_ENABLE 1

#if (ELOG_ENABLE == 1)
	#define ELOG_A(...) elog_a(__VA_ARGS__)
    #define ELOG_E(...) elog_e(__VA_ARGS__)
    #define ELOG_W(...) elog_w(__VA_ARGS__)
    #define ELOG_I(...) elog_i(__VA_ARGS__)
    #define ELOG_D(...) elog_d(__VA_ARGS__)
    #define ELOG_V(...) elog_v(__VA_ARGS__)

#else
	#define ELOG_A(...) ((void)0)
    #define ELOG_E(...) ((void)0)
    #define ELOG_W(...) ((void)0)
    #define ELOG_I(...) ((void)0)
    #define ELOG_D(...) ((void)0)
    #define ELOG_V(...) ((void)0)

#endif


void elog_func_init(void);
void test_elog(void);

#endif
