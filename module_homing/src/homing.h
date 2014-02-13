#include <hall_client.h>
#include <qei_client.h>
#include <xs1.h>
#include <platform.h>

#define GET_HOME_STATE  	1
#define SET_SWITCH_TYPE 	3

void set_home_switch_type(chanend c_home, int switch_type);

{int, int, int, int} get_home_state(chanend c_home);

void track_home_positon(port in p_ifm_ext_d0, port in p_ifm_ext_d1, chanend c_home, chanend c_qei, chanend c_hall);
