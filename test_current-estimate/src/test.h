typedef struct S_CMD {
	int set_torque;
} in_data;

#ifdef __XC__
int input_cmd( in_data &d );
#endif
