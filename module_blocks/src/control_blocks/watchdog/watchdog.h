/**
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Authors:  Victor de la Cruz <vdelacruz@synapticon.com>
 * Version: 0.1 (2013-08-22 1505)
 *
 * All code contained in this package under Synapticon copyright must be
 * licensing for any use from Synapticon. Please contact support@synapticon.com for
 * details of licensing.
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 * The copyrights, all other intellectual and industrial 
 * property rights are retained by XMOS and/or its licensors. 
 * Terms and conditions covering the use of this code can
 * be found in the Xmos End User License Agreement.
 *
 * Copyright XMOS Ltd 2013
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the 
 * copyright notice above.
 *
 **/                                   
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
 * \param c_wd the control channel for controlling the watchdog
 * \param p_wd_tick control for the tick of the watchdog
 * \param p_shared_leds_wden control port for the watchdog device
 */
void do_wd(chanend c_wd, out port p_wd_tick, out port p_shared_leds_wden);

