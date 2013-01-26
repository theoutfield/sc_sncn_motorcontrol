

typedef struct S_CMD_VAR {
	int iSetValueSpeedRpm;
	int iAngleVariable;
	int iPwmOnOff;
	int varx;
	int var1;
	int var2;
	int var3;
	int var4;
	int var5;
	int var6;
	int var7;
	int var8;
	int var9;
	int var10;
	int var11;
	int var12;
	int var13;
	int var14;
	int var15;
} cmd_data;

#ifdef __XC__
int input_cmd( cmd_data &d );
#endif
