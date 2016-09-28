/*
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
 **/



#ifndef _PWM_GENERAL_H_
#define _PWM_GENERAL_H_

/** Define Number of buffers in storage ring */
#define _NUM_PWM_BUFS 2  // Double-buffered

/** Define PWM port width resolution */
#define _PORT_RES_BITS 5 // PWM port width resoltion (e.g. 5 for 32-bits)

/** Define PWM port width in bits */
#define _PWM_PORT_WID (1 << _PORT_RES_BITS) // PWM port width in bits
#define _HALF_PORT_WID (_PWM_PORT_WID >> 1) // Half of PWM port width in bits

/** Different PWM Control Commands (Client --> Server) */
typedef enum CMD_PWM_ETAG
{
	// NB Don't use non-negative integer, due to conflicts with Buffer indices
	PWM_CMD_ACK = (-2),	// PWM Server Command Acknowledged (Control)
	PWM_CMD_LOOP_STOP = (-1), // Stop while-loop.
} CMD_PWM_ENUM;

/** Different PWM Phases */
typedef enum PWM_PHASE_ETAG
{
  _PWM_PHASE_A = 0,    // 1st Phase
  _PWM_PHASE_B,		  // 2nd Phase
  _PWM_PHASE_C,		  // 3rd Phase
  _NUM_PWM_PHASES      // Handy Value!-)
} PWM_PHASE_ENUM;

/** Structure containing PWM parameters for one motor */
typedef struct PWM_PARAM_TAG //
{
	unsigned widths[_NUM_PWM_PHASES]; // Array of PWM width values
	int id; // Unique Motor identifier e.g. 0 or 1 (NB -1 used to signal termination)
} PWM_PARAM_TYP;

/** Structure containing pwm communication control data */
typedef struct PWM_COMMS_TAG
{
	PWM_PARAM_TYP params; // Structure of PWM parameters (for Server)
	int buf; 	// double-buffer identifier. e.g. 0 or 1
	unsigned mem_addr; // Shared memory address (if used)
} PWM_COMMS_TYP;

// Structure containing data for doing timed load of buffered output port
typedef struct PWM_PORT_TAG
{
	unsigned pattern;		// Bit-pattern written to port (used to define pulse edge)
	signed time_off;	    // time-offset to start of pattern
} PWM_PORT_TYP;

// Structure containing pwm output data for one phase (& one edge)
typedef struct PWM_PHASE_TAG // Structure containing string
{
	PWM_PORT_TYP hi; // Port data for high leg (V+) of balanced line
	PWM_PORT_TYP lo; // Port data for low leg (V-) of balanced line
} PWM_PHASE_TYP;

// Structure containing data for one pulse edge for all phases
typedef struct PWM_EDGE_TAG
{
	PWM_PHASE_TYP phase_data[_NUM_PWM_PHASES]; // Array of phase-data structures, one for each phase
} PWM_EDGE_TYP;

// Structure containing pwm output data for one buffer
typedef struct PWM_BUFFER_TAG
{
	PWM_EDGE_TYP rise_edg; // data structure for rising edge of all pulses
	PWM_EDGE_TYP fall_edg; // data structure for falling edge of all pulses
} PWM_BUFFER_TYP;

// Structure containing pwm output data for all buffers
typedef struct PWM_ARRAY_TAG
{
	PWM_BUFFER_TYP buf_data[_NUM_PWM_BUFS]; // Array of buffer-data structures, one for each buffer
} PWM_ARRAY_TYP;


#endif // _PWM_GENERAL_H_
