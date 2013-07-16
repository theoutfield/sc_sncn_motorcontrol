/**
 * drive_sys.xc
 *
 *  Created on: Jul 12, 2013
 *      Author: pkanajar
 */

#include <drive_config.h>

/**
 *
 */
int update_statusword(int current_status, int state_reached)   //int current_status
{
	int status_word;
	switch(state_reached)
	{
		case 1:
			status_word = current_status & ~ready_to_switch_on_state;  //make 0
			break;

		case 7:
			status_word = (current_status & ~operation_enabled_state & ~switched_on_state & ~voltage_enabled_state) | ready_to_switch_on_state;  //make 1
			break;

		case 2:
			status_word = ( current_status  & ready_to_switch_on_state ) | switch_on_disabled_state;
			break;

		case 3:
			status_word = (current_status & ~switch_on_disabled_state)| switched_on_state | voltage_enabled_state;
			break;

		case 4:
			status_word = current_status | operation_enabled_state;
			break;

		case 5:
			status_word = current_status | fault_state;
			break;

		case 6:
			status_word = current_status | quick_stop_state;
			break;

	}
	return status_word;//(current_status | status_reached);
}

int get_next_values(int in_state, int check, int ctrl_input, int fault)
{
	int out_state;
	switch(in_state)
	{
		case 1:
			if(check == 0)
				out_state = 2;
			else if(check == 1)
				out_state = 7;
			break;

		case 2:
			if(check == 0)
				out_state = 1;
			else if(check == 1)
				out_state = 7;
			break;

		case 7:
			if(check == 0)
				out_state = 7;
			else if(check == 1)
				if(ctrl_input == 0)
					out_state = 7;
				else if(ctrl_input == 1)
					out_state = 3;
			break;

		case 3:
			if(check == 0)
				out_state = 3;
			else if(check == 1)
				if(ctrl_input == 0)
					out_state = 3;
				else if(ctrl_input == 1)
					out_state = 4;
			break;

		case 4:
			if(ctrl_input == 0)
				out_state = 4;
			else if(ctrl_input == 1) //quick stop
				out_state = 6;
			break;

		case 6:
			out_state = 2;
			break;

		default:
			if(fault == 1)
				out_state = 5;
			break;
	}
	return out_state;
}


//step(self, input)
//{
//	(s, o) = get_next_values(self.state, check, ctrl_input);
//	self.state = s; 	// update internal state
//	return o;			//return output
//}
