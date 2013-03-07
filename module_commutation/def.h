
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
void FOC_InversPark();

#define defParRpmMotorMax		3742
#define defParDefSpeedMax		4000
#define defParRPMreference		4000
#define defParAngleUser 		 560

#define defParUmotBoost  		 100
#define defParUmotStart 		 120
#define defParSpeedKneeUmot 	3500

//======== current limits ===============
#define defParRmsLimit			1500  // 66*4 = 264Bits/A
#define defParRmsMaxPwmOff      4000  //

//============ speed_control ============
#define defParHysteresisPercent	    5
#define defParDiffSpeedMax		  150
#define defParUmotIntegralLimit	 2048
#define defParPropGain			   64
#define defParIntegralGain		   64


#define defParTorquePropGain	   64
#define defParTorqueIntegralGain   64

//-------------------------------------
#define defParHallEncoder          0    // 0->Hall 1->Encoder
#define defParEncoderResolution 4096
#define defParEncoderZeroPoint     0
//-------------------------------------
#define defParRampAcc  65536		 //  RPM/sec
#define defParRampDec  65536         //
#define defParRampSmoothFactor 4     //

#define defParPositionSpeedMax   800   // RPM
#define defParPositionSpeedMin   100   // RPM






