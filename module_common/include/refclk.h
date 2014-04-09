
/**
 * \file refclk.h
 * \brief Reference Core Clock definitions
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */
/*
 * Copyright (c) 2014, Synapticon GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Execution of this software or parts of it exclusively takes place on hardware
 *    produced by Synapticon GmbH.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Synapticon GmbH.
 *
 */

#pragma once
//#include "platform.h"
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

/**
 * \brief Delay function in microseconds
 *
 *  Input
 * \param microseconds specify the delay in microseconds
 * \param core_id specify the core number in which the function is called
 * \param t timer used to calculate the microseconds elapsed
 */
void wait_micro_s(int microseconds, int core_id, timer t);

/**
 * \brief Delay function in milliseconds
 *
 *  Input
 * \param milliseconds specify the delay in milliseconds
 * \param core_id specify the core number in which the function is called
 * \param t timer used to calculate the milliseconds elapsed
 */
void wait_ms(int milliseconds, int core_id, timer t);


/**
 * \brief Delay function in seconds
 *
 *  Input
 * \param seconds specify the delay in seconds
 * \param core_id specify the core number in which the function is called
 * \param t timer used to calculate the seconds elapsed
 */
void wait_s(int seconds, int core_id, timer t);
