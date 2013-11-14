#include <position_ctrl_client.h>
#include <print.h>
#include <drive_config.h>

//#define DEBUG
//#define debug_print


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


void set_position_ctrl_param(ctrl_par &position_ctrl_params, chanend c_position_ctrl)
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

void set_position_ctrl_hall_param(hall_par &hall_params, chanend c_position_ctrl)
{
	c_position_ctrl <: SET_POSITION_CTRL_HALL;
	c_position_ctrl <: hall_params.gear_ratio;
	c_position_ctrl <: hall_params.pole_pairs;
}

void set_position_ctrl_qei_param(qei_par &qei_params, chanend c_position_ctrl)
{
	c_position_ctrl <: SET_POSITION_CTRL_QEI;
	c_position_ctrl <: qei_params.gear_ratio;
	c_position_ctrl <: qei_params.index;
	c_position_ctrl <: qei_params.real_counts;
	c_position_ctrl <: qei_params.max_count;
}


void set_position_sensor(int sensor_used, chanend c_position_ctrl)
{
	POSITION_CTRL_WRITE(SENSOR_SELECT);
	POSITION_CTRL_WRITE(sensor_used);
}

void shutdown_position_ctrl(chanend c_position_ctrl)
{
	POSITION_CTRL_WRITE(SHUTDOWN_POSITION);
	POSITION_CTRL_WRITE(1);
}

void enable_position_ctrl(chanend c_position_ctrl)
{
	POSITION_CTRL_WRITE(ENABLE_POSITION);
	POSITION_CTRL_WRITE(0);
}