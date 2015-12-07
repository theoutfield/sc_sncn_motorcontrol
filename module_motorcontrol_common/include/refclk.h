/**
 * @file refclk.h
 * @brief Reference Clock definitions
 * @author Synapticon GmbH <support@synapticon.com>
 */


#pragma once

#include <xs1.h>

/* 10NSEC_STD --> 1 */
#define USEC_STD       100
#define MSEC_STD    100000
#define SEC_STD  100000000


#if PLATFORM_REFERENCE_MHZ == 100

#define USEC_FAST USEC_STD
#define MSEC_FAST MSEC_STD
#define SEC_FAST  SEC_STD

#else /* REFCLK_STD == 100 MHZ , REFCLK_FAST == 250 MHZ */


/* 4NSEC_FAST --> 1 */
#define USEC_FAST       250
#define MSEC_FAST    250000
#define SEC_FAST  250000000

#endif
