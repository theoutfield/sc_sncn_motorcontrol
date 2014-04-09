
/**
 * \file watchdog.h
 * \brief Watchdog Implementation
 * \author Victor de la Cruz <vdelacruz@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */
/*
 * Copyright (c) 2014, Synapticon GmbH & XMOS Ltd
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

#define WD_CMD_EN_MOTOR		1
#define WD_CMD_DIS_MOTOR	2
#define WD_CMD_TICK			3
#define WD_CMD_START		4

/** \brief Run the watchdog timer server
 *
 * The watchdog timer needs a constant stream of pulses to prevent it
 * from shutting down the motor.  This is a thread server which implements
 * the watchdog timer output.
 *
 * The watchdog control has two differents ports attached to
 * the watchdog circuitry. The enable signal must be the LSB
 * bit in a 4-bit port. The tick control must be a 1-bit port.
 *
 * \param c_watchdog the control channel for controlling the watchdog
 * \param p_wd_tick control for the tick of the watchdog
 * \param p_shared_leds_wden control port for the watchdog device
 */
void run_watchdog(chanend c_watchdog, out port p_wd_tick, out port p_shared_leds_wden);
