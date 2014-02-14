#include <homing.h>
#include <xscope.h>
{int, int, int, int} get_home_state(chanend c_home)
{
	int position;
	int direction;
	int home_state;
	int safety_state;
	c_home <: GET_HOME_STATE;
	c_home :> home_state;
	c_home :> safety_state;
	c_home :> position;
	c_home :> direction;
	return {home_state, safety_state, position, direction};
}

void set_home_switch_type(chanend c_home, int switch_type)
{
	c_home <: SET_SWITCH_TYPE;
	c_home <: switch_type;
}

void track_home_positon(port in p_ifm_ext_d0, port in p_ifm_ext_d1, chanend c_home, chanend c_qei, chanend c_hall)
{
	timer t;
	unsigned int time, time1;
	int home_switch;
	int home_switch_1;
	int home_state = 0;
	int safety_state = 0;
	int safety_switch;
	int safety_switch_1;
	int position = 0;
	int direction;
	int command;
	int active_state = 4;

	int switch_type = 1; //active high switch = 1/ active low switch = 2

	int switch_setup_flag = 0;
	while(1)
	{
		select
		{
			case c_home :> command:
				if(command == SET_SWITCH_TYPE)
				{
					c_home :> switch_type;
					switch_setup_flag = 1;
				}
				break;
		}
		if(switch_setup_flag == 1)
		{
			break;
		}
	}

	if(switch_type == 1) //active high
	{
		set_port_pull_down(p_ifm_ext_d0);
		set_port_pull_down(p_ifm_ext_d1);
		active_state = 1;
	}
	else if(switch_type == 2) //active low (ports can only be pulled high with pull-up resister)
	{
		active_state = 0;
	}
	t:>time;

	//xscope_initialise_1();
/*	{
			xscope_register(2, XSCOPE_CONTINUOUS, "0 actual_velocity", XSCOPE_INT,	"n",
								XSCOPE_CONTINUOUS, "1 target_velocity", XSCOPE_INT, "n");

			xscope_config_io(XSCOPE_IO_BASIC);
		}*/
	p_ifm_ext_d0 :> home_switch;   //once
	p_ifm_ext_d1 :> safety_switch; //once
	if(home_switch == active_state)
	{
		home_state = 1;
		//printstrln("home");
		//xscope_probe_data(0, home_state);
	}
	if(safety_switch == active_state)
	{
		safety_state = 1;
		//printstrln("home");
		//xscope_probe_data(0, home_state);
	}
	while(1)
	{

		//xscope_probe_data(0, home_switch);
		//xscope_probe_data(1, safety_switch);

#pragma ordered
		select
		{
			case c_home :> command:   //case upon request send home state, safety state with position info too
				if(command == GET_HOME_STATE)
				{
					c_home <: home_state;
					c_home <: safety_state;
					c_home <: position;
					c_home <: direction;
				}
				else if(command == SET_SWITCH_TYPE)
				{
					c_home :> switch_type;
					//switch_setup_flag = 1;
				}
				break;

			case p_ifm_ext_d0 when pinsneq(home_switch) :> home_switch:
				if(home_switch == active_state)
				{	//register pos data immediately
					{position, direction} = get_qei_position_absolute(c_qei);
					t :> time1;
					t when timerafter(time1 + 3333) :> time1; //3khz
					p_ifm_ext_d0 :> home_switch_1;
					if(home_switch_1 == active_state)
					{
						t when timerafter(time1 + 3333) :> time1;
						p_ifm_ext_d0 :> home_switch_1;
						if(home_switch_1 == active_state)
						{
							home_state = 1;
							// xscope_probe_data(0, home_state);
							//printstrln("home");    // confirm home position
						}
					}
				}
				else
				{
					home_state = 0;
					//xscope_probe_data(0, home_state);
				}
				break;

			case p_ifm_ext_d1 when pinsneq(safety_switch) :> safety_switch:
				if(safety_switch == active_state)
				{
					t :> time1;
					t when timerafter(time1 + 3333) :> time1;
					p_ifm_ext_d1 :> safety_switch_1;
					if(safety_switch_1 == active_state)
					{
						t when timerafter(time1 + 3333) :> time1;
						p_ifm_ext_d1 :> safety_switch_1;
						if(safety_switch_1 == active_state)
						{
							safety_state = 1;
							//xscope_probe_data(0, safety_state);  // confirm home position
						}
					}
				}
				else
				{
					safety_state = 0;
				}
				break;



		}
	}
}
