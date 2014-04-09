
/**
 *
 * \file test.xc
 *	 Unit Testing Position control.
 *
 */

typedef struct S_CMD {
	int set_torque;
	int set_position;
	int set_velocity;
	int mode; 			//1 torque, 2 velocity, 3 position
	int exit_mode; 		// 1 exit 0 continue
	int shutdown;
	int enable;
	int sensor_select;
	int activate;
	int max_position;
	int min_position;
} in_data;



#ifdef __XC__
void qei_unit_test();
void position_ctrl_unit_test(chanend c_position_ctrl, chanend c_qei, chanend c_hall);
void velocity_ctrl_unit_test(chanend c_velocity_ctrl, chanend c_qei, chanend c_hall);
void enable_motor_test(chanend c_commutation);
int input_torq( in_data &d );
int input_pos(in_data &d);
int input_vel(in_data &d);
int input_mode(in_data &d);
int input_shutdown(in_data &d);
int input_activate(in_data &d);
int input_activate_vel(in_data &d);
int input_max_position(in_data &d);
int input_min_position(in_data &d);
void set_torque_test(chanend c_torque_ctrl, chanend c_velocity_ctrl);

#endif
extern int sine_func(int arg);
