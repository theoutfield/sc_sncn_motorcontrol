
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


#define defParRpmMotorMax		4000
#define defParRpmUmotMax	 	9500
#define defParUmotBoost  		  60
#define defParUmotStart 		  70

#define defParAngleUser 		 640


//======== current limits ===============
#define defParRmsLimit			1500  // 66*4 = 264Bits/A
#define defParRmsMaxPwmOff      4000  //

//============ speed_control ============
#define defParHysteresisPercent	    5
#define defParDiffSpeedMax		  150
#define defParUmotIntegralLimit	 2048
#define defParPropGain			    8
#define defParIntegralGain		    8

#define defParTorquePropGain	   64
#define defParTorqueIntegralGain   64

//-------------------------------------
#define defParEncoderResolution 4000
#define defParEncoderZeroPoint   570
//-------------------------------------
#define defParRampAcc  65536/16		 //  RPM/sec
#define defParRampDec  65536/16         //
#define defParRampSmoothFactor 4     //

#define defParPositionSpeedMax   800   // RPM
#define defParPositionSpeedMin   100   // RPM






