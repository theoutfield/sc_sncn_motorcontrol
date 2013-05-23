
typedef struct S_CMD {
	int set_torque;
	int set_position;
	int set_speed;
} in_data;



#ifdef __XC__
int input_cmd( in_data &d );
int input_pos(in_data &d);
#endif
