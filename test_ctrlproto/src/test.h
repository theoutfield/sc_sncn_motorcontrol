
typedef struct S_CMD {
	int set_torque;
	int set_position;
	int set_velocity;
	int set_state;
	int check;
	int ctrl_input;
	int fault;
} in_data;



#ifdef __XC__
int input_torq( in_data &d );
int input_pos(in_data &d);
int input_vel(in_data &d);
int input_new_state(in_data &d);
int test_get_next_state(in_data &d);
void set_torque_test(chanend c_torque) ;
#endif
