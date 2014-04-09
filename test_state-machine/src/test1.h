
typedef struct S_CMD {
	int set_torque;
	int set_position;
	int set_velocity;
	int set_state;
	int check;
	int fault;
	int ctrl_input;
	int op_mode;
} in_data;



#ifdef __XC__
void ecat_motor_test(chanend pdo_out, chanend pdo_in, chanend coe_out, chanend c_signal, chanend c_hall,\
		chanend c_qei, chanend c_home, chanend c_torque_ctrl, chanend c_velocity_ctrl, chanend c_position_ctrl);
int input_torq( in_data &d );
int input_pos(in_data &d);
int input_vel(in_data &d);
int input_new_state(in_data &d);
int input_cw(in_data &d );
int test_get_next_state(in_data &d);
void set_torque_test(chanend c_torque) ;
#endif
