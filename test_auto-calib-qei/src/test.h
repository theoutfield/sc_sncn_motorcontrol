
typedef struct S_CMD {
	int set_torque;
	int set_position;
	int set_velocity;
	int mode; //1 torque, 2 velocity, 3 position
	int exit_mode; // 1 exit 0 continue
	int shutdown;
} in_data;



#ifdef __XC__
void enable_motor_test(chanend c_commutation);
int input_torq( in_data &d );
int input_pos(in_data &d);
int input_vel(in_data &d);
int input_mode(in_data &d);
int input_shutdown(in_data &d);
void set_torque_test(chanend c_torque_ctrl, chanend c_velocity_ctrl);

#endif
extern int sine_func(int arg);
