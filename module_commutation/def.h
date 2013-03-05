



void SaveValueToArray(); void SetParameterValue(); void InitParameter();
void    function_SpeedControl();
void    function_TorqueControl();
void 	function_PositionControl();
void 	SpeedControl(); void CalcUmotForSpeed(); void CalcRampForSpeed();
void 	CalcSetInternSpeed(int iSpeedValue);
void 	CalcCurrentValues();

void FOC_ClarkeAndPark();
void FOC_FilterDiffValue();
void FOC_InversPark();

#define defParRpmMotorMax		3742
#define defParDefSpeedMax		4000
#define defParRPMreference		4000
#define defParAngleUser 		 560
#define defParAngleFromRPM 		 150
#define defParUmotBoost  		 100
#define defParUmotStart 		 120
#define defParSpeedKneeUmot 	3500
#define defParAngleCorrVal         1
#define defParAngleCorrMax		 300


#define defParRmsLimit			1500  // 66*4 = 264Bits/A
#define defParRmsMaxPwmOff      4000


#define defParHysteresisPercent	    5
#define defParDiffSpeedMax		  150
#define defParUmotIntegralLimit	 2048
#define defParPropGain			   64
#define defParIntegralGain		   64

#define defParTorquePropGain	   64
#define defParTorqueIntegralGain   64

//-------------------------------------
#define defRampAcc  65536
#define defRampDec  65536
#define defRampSmoothFactor 4

#define defParPositionSpeedMax   800   // RPM
#define defParPositionSpeedMin   200   // RPM






