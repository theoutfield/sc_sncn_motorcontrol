
void SaveValueToArray(); void SetParameterValue(); void InitParameter();

void    function_SpeedControl();
void    function_TorqueControl();
void 	function_PositionControl();
void    function_SensorLessControl();

void 	SpeedControl(); void CalcUmotForSpeed(); void CalcRampForSpeed();
void 	CalcSetInternSpeed(int iSpeedValue);
void 	CalcCurrentValues();

void FOC_ClarkeAndPark();
void FOC_FilterDiffValue();
void FOC_Integrator();
void FOC_InversPark();









