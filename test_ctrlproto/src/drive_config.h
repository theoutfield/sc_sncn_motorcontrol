/**
 * \brief Modes of Operation (CiA402)
 *
 * 	M - Mandatory, C - Conditional, R - Recommended, O - optional , FG - Function Group
 */

#define	pp		1 	/* Profile Position mode 									O*/
#define vl 		2	/* Velocity mode (frequency converter) 						O*/
#define pv 		3   /* Profile velocity mode 									O*/
#define tq 		4   /* Torque profile mode 										O*/
#define hm 		6	/* Homing mode 												O*/
#define ip 		7	/* Interpolated position mode 								O*/
#define csp 	8	/* Cyclic synchronous position mode 						C*/
#define csv 	9	/* Cyclic synchronous velocity mode 						C*/
#define cst 	10	/* Cyclic synchronous torque mode 							C*/
#define cstca 	11	/* Cyclic synchronous torque mode with commutation angle 	O*/

/* Manufacturer specific mode -128...-1 optional */





/* Controlword */

//Common for all Modes of Operation (CiA402)

#define shut_down				0x0006
#define swith_on				0x000F
#define quick_stop  			0x000B
#define clear_fault 			0x0080

//Operation Mode specific control words (complies with CiA402)

		/* Homing mode */
#define start_homing 			0x001F
#define halt_homing  			0x011F

	/* Profile Position Mode */
#define absolute_positioning 	0x001F	 	// not supported yet
#define relative_positioning 	0x005F   	// supported currently
#define stop_positioning		0x010F

	/*Profile Velocity Mode*/
#define halt_profile_velocity	0x010F


/* Statusword */
//state defined is ORed with current state

#define target_reached 			0x0400

		/* Homing Mode */
#define homing_attained 		0x1000

	/* Profile Position Mode */
#define set_position_ack	  	0x1000

	/* Profile Velocity Mode */
#define target_velocity_reached 0x0400



