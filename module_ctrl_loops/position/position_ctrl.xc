#include <position_ctrl.h>
#include <xscope.h>
#include <print.h>
#include <drive_config.h>

//#define DEBUG
//#define debug_print



/*Internal Controller Configs*/
#define SET_POSITION_TOKEN 40
#define GET_POSITION_TOKEN 41
#define HALL 1
#define QEI 2
#define HALL_PRECISION		2
#define QEI_PRECISION		512

#define SET_CTRL_PARAMETER 	101
#define SENSOR_SELECT      	151
#define SHUTDOWN_POS	 	201
#define ENABLE_POS			251



extern int position_factor(int gear_ratio, int qei_max_real, int pole_pairs, int sensor_used);

int init_position_control(chanend c_position_ctrl)
{
	int init_state = INIT_BUSY;

	POSITION_CTRL_ENABLE(); 					//signal position ctrl loop

	// init check from position control loop
	while(1)
	{
		init_state = __check_position_init(c_position_ctrl);
		if(init_state == INIT)
		{
#ifdef debug_print
			printstrln("pos intialized");
#endif
			break;
		}
	}
	return init_state;
}

//internal functions
void set_position(int target_position, chanend c_position_ctrl)
{
	POSITION_CTRL_WRITE(SET_POSITION_TOKEN);
	POSITION_CTRL_WRITE(target_position);
}


int get_position(chanend c_position_ctrl)
{
	int position;
	POSITION_CTRL_WRITE(GET_POSITION_TOKEN);
	POSITION_CTRL_READ(position);
	return position;
}

int position_limit(int position, int max_position_limit, int min_position_limit)
{
	if (position > max_position_limit)
	{
		return max_position_limit;
	}
	else if (position < min_position_limit)
	{
		return min_position_limit;
	}
	else if (position >= min_position_limit && position <= max_position_limit)
	{
		return position;
	}
}

//csp mode function
void set_position_csp(csp_par &csp_params, int target_position, int position_offset, int velocity_offset,\
		              int torque_offset, chanend c_position_ctrl)
{
	set_position( position_limit( (target_position + position_offset) * csp_params.base.polarity ,	\
								   csp_params.max_position_limit * 10000  , 						\
								   csp_params.min_position_limit * 10000) , c_position_ctrl);
}


void init_position_ctrl_param_ecat(ctrl_par &position_ctrl_params, chanend c_position_ctrl)
{
	POSITION_CTRL_WRITE(SET_CTRL_PARAMETER);
	POSITION_CTRL_WRITE(position_ctrl_params.Kp_n);
	POSITION_CTRL_WRITE(position_ctrl_params.Kp_d);
	POSITION_CTRL_WRITE(position_ctrl_params.Ki_n);
	POSITION_CTRL_WRITE(position_ctrl_params.Ki_d);
	POSITION_CTRL_WRITE(position_ctrl_params.Kd_n);
	POSITION_CTRL_WRITE(position_ctrl_params.Kd_d);
	POSITION_CTRL_WRITE(position_ctrl_params.Integral_limit);
}

void init_position_ctrl_hall(hall_par &hall_params, chanend c_position_ctrl)
{
	c_position_ctrl <: SET_POS_CTRL_HALL;
	c_position_ctrl <: hall_params.gear_ratio;
	c_position_ctrl <: hall_params.pole_pairs;
}

void init_position_ctrl_qei(qei_par &qei_params, chanend c_position_ctrl)
{
	c_position_ctrl <: SET_POS_CTRL_QEI;
	c_position_ctrl <: qei_params.gear_ratio;
	c_position_ctrl <: qei_params.index;
	c_position_ctrl <: qei_params.real_counts;
	c_position_ctrl <: qei_params.max_count;
}


void init_position_sensor_ecat(int sensor_used, chanend c_position_ctrl)
{
	POSITION_CTRL_WRITE(SENSOR_SELECT);
	POSITION_CTRL_WRITE(sensor_used);
}

void shutdown_position_ctrl(chanend c_position_ctrl)
{
	POSITION_CTRL_WRITE(SHUTDOWN_POS);
	POSITION_CTRL_WRITE(1);
}

void enable_position_ctrl(chanend c_position_ctrl)
{
	POSITION_CTRL_WRITE(ENABLE_POS);
	POSITION_CTRL_WRITE(0);
}

void position_control(ctrl_par &position_ctrl_params, hall_par &hall_params, qei_par &qei_params, int sensor_used, \
		              chanend c_hall, chanend c_qei, chanend c_position_ctrl, chanend c_commutation)
{
	int actual_position = 0;
	int target_position = 0;

	int error_position = 0;
	int error_position_D = 0;
	int error_position_I = 0;
	int previous_error = 0;
	int position_control_out = 0;

	timer ts;
	unsigned int time;

	int command;
	int deactivate = 0;
	int activate = 0;
	int direction = 0;

	int precision;
	int precision_factor;

	int init_state = INIT_BUSY;

	while(1)
	{
		int received_command = UNSET;
		select
		{
			case POSITION_CTRL_READ(command):
				if(command == SET)
				{
					activate = SET;
					received_command = SET;
					while(1)
					{
						init_state = __check_commutation_init(c_commutation);
						if(init_state == INIT)
						{
							printstrln("commutation intialized");
							init_state = INIT_BUSY;
							break;
						}
					}
#ifdef debug_print
					printstrln("pos activated");
#endif
				}
				else if(command == UNSET)
				{
					activate = UNSET;
					received_command = SET;
#ifdef debug_print
					printstrln("pos disabled");
#endif
				}
				else if(command == CHECK_BUSY)
				{
					POSITION_CTRL_WRITE(init_state);
				}
				break;

			default:
				break;
		}
		if(received_command == SET)
		{
//			POSITION_CTRL_WRITE(received_command);
			break;
		}
	}

	//printstrln("start pos");

	ts:> time;

	if(sensor_used == HALL)
	{
		precision_factor = position_factor(hall_params.gear_ratio, 1, hall_params.pole_pairs, sensor_used);
		precision = HALL_PRECISION;
	}
	else if(sensor_used == QEI)
	{
		precision_factor = position_factor(qei_params.gear_ratio, qei_params.real_counts, 1, sensor_used);
		precision = QEI_PRECISION;
	}

	init_state = INIT;
	while(activate)
	{
		#pragma ordered
		select
		{
			case ts when timerafter(time + position_ctrl_params.Loop_time) :> time: // 1 ms

				/* acq actual position hall/qei */

				if(sensor_used == HALL)
				{   /* 100/(500*819) ~ 1/4095 appr (hall)  - to keep position info from hall in same range as qei*/
					{actual_position , direction}= get_hall_position_absolute(c_hall);
					actual_position = ( ( ( (actual_position/500)*precision_factor)/precision )/819)*100;
				}
				else if(sensor_used == QEI)
				{
					{actual_position, direction} =  get_qei_position_absolute(c_qei);
					actual_position = (actual_position * precision_factor)/precision;
				}

				/* Controller */

				error_position = (target_position - actual_position);
				error_position_I = error_position_I + error_position;
				error_position_D = error_position - previous_error;

				if(error_position_I > position_ctrl_params.Integral_limit)
				{
					error_position_I = position_ctrl_params.Integral_limit;
				}
				else if(error_position_I < -position_ctrl_params.Integral_limit)
				{
					error_position_I = 0 - position_ctrl_params.Integral_limit;
				}

				position_control_out = (position_ctrl_params.Kp_n * error_position)/position_ctrl_params.Kp_d   \
									 + (position_ctrl_params.Ki_n * error_position_I)/position_ctrl_params.Ki_d \
									 + (position_ctrl_params.Kd_n * error_position_D)/position_ctrl_params.Kd_d;

				if(position_control_out > position_ctrl_params.Control_limit)
				{
					position_control_out = position_ctrl_params.Control_limit;
				}
				else if(position_control_out < -position_ctrl_params.Control_limit)
				{
					position_control_out = 0 - position_ctrl_params.Control_limit;
				}

				if(!deactivate)
					set_commutation_sinusoidal(c_commutation, position_control_out);
				else
					set_commutation_sinusoidal(c_commutation, 0);

				#ifdef DEBUG
				xscope_probe_data(0, actual_position);
				#endif

				previous_error = error_position;

				break;

				/* acq target position from etherCAT */

			case POSITION_CTRL_READ(command):

				if(command == SET_POSITION_TOKEN)
				{
					POSITION_CTRL_READ(target_position);
				}
				else if(command == GET_POSITION_TOKEN)
				{
					POSITION_CTRL_WRITE(actual_position);
				}
				else if(command == CHECK_BUSY)
				{
					POSITION_CTRL_WRITE(init_state);
				}
				else if(command == SET_CTRL_PARAMETER)
				{
					POSITION_CTRL_READ(position_ctrl_params.Kp_n);
					POSITION_CTRL_READ(position_ctrl_params.Kp_d);
					POSITION_CTRL_READ(position_ctrl_params.Ki_n);
					POSITION_CTRL_READ(position_ctrl_params.Ki_d);
					POSITION_CTRL_READ(position_ctrl_params.Kd_n);
					POSITION_CTRL_READ(position_ctrl_params.Kd_d);
					POSITION_CTRL_READ(position_ctrl_params.Integral_limit);
				}
				else if(command == SENSOR_SELECT)
				{
					POSITION_CTRL_READ(sensor_used);
					if(sensor_used == HALL)
					{
						precision_factor = position_factor(hall_params.gear_ratio, 1, hall_params.pole_pairs, sensor_used);
						precision = HALL_PRECISION;
					}
					else if(sensor_used == QEI)
					{
						precision_factor = position_factor(qei_params.gear_ratio, qei_params.real_counts, 1, sensor_used);
						precision = QEI_PRECISION;
					}
				}
				else if(command == SHUTDOWN_POS)
					POSITION_CTRL_READ(deactivate);

				else if(command == ENABLE_POS)
					POSITION_CTRL_READ(deactivate);

				else if(command == SET_POS_CTRL_HALL)
				{
					c_position_ctrl :> hall_params.gear_ratio;
					c_position_ctrl :> hall_params.pole_pairs;
				}
				else if(command == SET_POS_CTRL_QEI)
				{
					c_position_ctrl :> qei_params.gear_ratio;
					c_position_ctrl :> qei_params.index;
					c_position_ctrl :> qei_params.real_counts;
					c_position_ctrl :> qei_params.max_count;
				}
				break;
		}

	}
}
