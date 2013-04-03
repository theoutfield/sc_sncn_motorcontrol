
void SaveValueToArray(); void SaveInfosToArray(); void SetParameterValue(); void InitParameter();

void    function_UmotControl();
void    function_SpeedControl_F1();   // without FOC
void    function_SpeedControl_F2();   // angle with FOC
void    function_TorqueControl();

void 	function_PositionControl();
void    function_SensorLessControl();

void 	SpeedControl(); void CalcUmotForSpeed(); void CalcRampForSpeed();
void 	CalcSetUserSpeed(int iSpeedValue);
void 	CalcDiffSpeed();
void 	CalcCurrentValues();
int 	CalcUmotProfile();

void FOC_ClarkeAndPark();
void FOC_FilterDiffValue();
void FOC_Integrator();
void FOC_InversPark();









