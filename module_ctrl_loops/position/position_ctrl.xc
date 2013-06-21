#include <position_ctrl.h>
#include <xscope.h>

#define DEBUG

#define SET_POSITION_TOKEN 40
#define HALL 1
#define QEI 2
#define HALL_PRECISION		2
#define QEI_PRECISION		512

//180, Kd = 100, Ki = 50; //  (Kp*error_position)/2000 + (Ki*error_position_I)/102000 + (Kd*error_position_D)/10000;
#define POSITION_Kp_NUMERATOR 	 	180
#define POSITION_Kp_DENOMINATOR  	2000
#define POSITION_Ki_NUMERATOR    	50
#define POSITION_Ki_DENOMINATOR  	102000
#define POSITION_Kd_NUMERATOR    	100
#define POSITION_Kd_DENOMINATOR  	10000
#define POSITION_CONTROL_LOOP_TIME 	1			//in 1 ms (default do not change)

extern int position_factor(int gear_ratio, int qei_max_real, int pole_pairs, int sensor_used);


//internal functions
void set_position(int target_position, chanend c_position_ctrl)
{
	POSITION_CTRL_WRITE(SET_POSITION_TOKEN); //c_position_ctrl <: SET_POSITION_TOKEN;	//
	POSITION_CTRL_WRITE(target_position);     //c_position_ctrl <: target_position;  	//
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
	set_position(position_limit( (target_position + position_offset) * csp_params.base.polarity , \
								  csp_params.max_position_limit*1000, csp_params.min_position_limit*1000), c_position_ctrl);
}


void init_position_control(ctrl_par &position_ctrl_params)
{

	position_ctrl_params.Kp_n = POSITION_Kp_NUMERATOR;
	position_ctrl_params.Kp_d = POSITION_Kp_DENOMINATOR;
	position_ctrl_params.Ki_n = POSITION_Ki_NUMERATOR;
	position_ctrl_params.Ki_d = POSITION_Ki_DENOMINATOR;
	position_ctrl_params.Kd_n = POSITION_Kd_NUMERATOR;
	position_ctrl_params.Kd_d = POSITION_Kd_DENOMINATOR;
	position_ctrl_params.Loop_time = POSITION_CONTROL_LOOP_TIME * MSEC_STD;  //units - core timer value //CORE 2/1/0

	position_ctrl_params.Control_limit = 13739; 							 //default do not change

	if(position_ctrl_params.Ki_n != 0)    									 //auto calculated using control_limit
		position_ctrl_params.Integral_limit = (position_ctrl_params.Control_limit * position_ctrl_params.Ki_d)/position_ctrl_params.Ki_n ;
	else
		position_ctrl_params.Integral_limit = 0;

	return;
}

void position_control(ctrl_par position_ctrl_params, hall_par hall_params, qei_par qei_params, int sensor_used, \
		              chanend c_hall, chanend c_qei, chanend c_signal, chanend c_position_ctrl, chanend c_commutation)
{
	int actual_position = 0;
	int target_position = 0;

	int error_position = 0;
	int error_position_D = 0;
	int error_position_I = 0;
	int previous_error = 0;
	int position_control_out = 0;

	//int Kp = 180, Kd = 100, Ki = 50;int max_integral = (13739*102000)/Ki;  (Kp*error_position)/2000 + (Ki*error_position_I)/102000 + (Kd*error_position_D)/10000;

	timer ts;
	unsigned int time;

	int command;
	int direction = 0;

	int precision;
	int precision_factor;

	//check init signal from commutation level
	while (1)
	{
		unsigned received_command = 0;
		select
		{
			case c_signal :> command: 			//SIGNAL_READ(command):
				received_command = 1;
				break;
			default:
				break;
		}
		if(received_command == 1)
		{
			printstrln(" init commutation");
			break;
		}
	}

	ts:> time;
	ts when timerafter(time+SEC_STD) :> time;

	POSITION_CTRL_ENABLE();						 //start


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


	while(1)
	{
		#pragma ordered
		select
		{
			case ts when timerafter(time + position_ctrl_params.Loop_time) :> time: //1khz

				/* acq actual position hall/qei */

				if(sensor_used == HALL)
				{   /* 100/(500*819) ~ 1/4095 appr (hall)  - to keep position info from hall in same range as qei*/
					actual_position = ( ( ( (get_hall_absolute_pos(c_hall)/500)*precision_factor)/precision )/819)*100;
				}
				else if(sensor_used == QEI)
				{
					{actual_position, direction} =  get_qei_position_count(c_qei);
					actual_position = (actual_position * precision_factor)/precision;
				}

				/* Controller */

				error_position = (target_position - actual_position);
				error_position_I = error_position_I + error_position;
				error_position_D = error_position - previous_error;

				if(error_position_I > position_ctrl_params.Integral_limit)
					error_position_I = position_ctrl_params.Integral_limit;
				else if(error_position_I < -position_ctrl_params.Integral_limit)
					error_position_I = 0 - position_ctrl_params.Integral_limit;

				position_control_out = (position_ctrl_params.Kp_n * error_position)/position_ctrl_params.Kp_d   \
									 + (position_ctrl_params.Ki_n * error_position_I)/position_ctrl_params.Ki_d \
									 + (position_ctrl_params.Kd_n * error_position_D)/position_ctrl_params.Kd_d;

				if(position_control_out > position_ctrl_params.Control_limit)
					position_control_out = position_ctrl_params.Control_limit;
				else if(position_control_out < -position_ctrl_params.Control_limit)
					position_control_out = 0-position_ctrl_params.Control_limit;


				set_commutation_sinusoidal(c_commutation, position_control_out);

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
				break;
		}

	}
}
