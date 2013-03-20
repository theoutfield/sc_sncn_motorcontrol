
/**************************************************************************
 * \file refclk.h
 *	Reference Clock file
 *
 * Defines clock ticks for different time units which is CORE dependent
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Authors:  Martin Schwarz <mschwarz@synapticon.com>
 *
 * All code contained in this package under Synapticon copyright must be
 * licensing for any use from Synapticon. Please contact support@synapticon.com for
 * details of licensing.

 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **************************************************************************/
#pragma once
#include <xs1.h>

/* Clock definition for CORES 0, 1 and 2.   10NSEC_STD --> 1 */
#define USEC_STD       100
#define MSEC_STD    100000
#define SEC_STD  100000000


#if PLATFORM_REFERENCE_MHZ == 100

#define USEC_FAST USEC_STD
#define MSEC_FAST MSEC_STD
#define SEC_FAST  SEC_STD

#else /* REFCLK_STD == 100 MHZ , REFCLK_FAST == 250 MHZ */


/* Clock definition for CORE 3. 			4NSEC_FAST --> 1 */
#define USEC_FAST       250
#define MSEC_FAST    250000
#define SEC_FAST  250000000

#endif
