

typedef struct S_CMD_VAR {
	int iMotPar[32];
	int iMotValues[32];
	int iMotCommand[16];
	int iMotInfo[64];
	int varx;
	int var1;
} cmd_data;



typedef struct S_C_Var {
	int var1;
} tor_data;


#ifdef __XC__
int input_cmd( cmd_data &d );
int input_tor_cmd( tor_data &d );
#endif
