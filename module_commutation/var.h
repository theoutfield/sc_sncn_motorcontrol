
int iUmotSquare;
int iUmotLinear;



//------------- values from hall_input.xc ---------
int iHallActualSpeed,iHallAngle,iHallPositionAbsolut,iHallPinState;
int iHallSpeedValueIsNew;
int iHallActualSpeed;
int iHallPositionAbsolut;


int iEncoderActualSpeed,iEncoderAngle,iEncoderPositionAbsolut,iEncoderPinState;
int iEncoderSpeedValueIsNew;
int iEncoderActualSpeed;
int iEncoderPositionAbsolut;
int iEncoderNullReference = 0;

int iActualSpeed;
int iSpeedValueIsNew	=  0;  // if speed from hall is a new value
int iDiffAngleRotor;

//-----------------------------------------------------------
int iMotPar[32];
int iMotValue[32];
int iMotCommand[16];


//--------- parameters ---------------------------------------
int iParRpmMotorMax;
int iParRpmUmotMax;
int iParUmotStart;
int iParUmotBoost;

int iParAngleUser;
int iParHysteresisPercent;
int iParDiffSpeedMax;
int iParAngleFromRPM;
int iParUmotIntegralLimit;
int iParPropGain;
int iParIntegralGain;
int iParRMS_RampLimit;
int iParRMS_PwmOff;

int iUpdateFlag=0;

//=========== motor values ===================================
int iUmotProfile;
int iUmotIntegrator=0;
int iUmotP;
int iUmotRpmLimit;
int iUmotMotor = 0;   	// follows iUmotResult

int iTorqueDiff1;
int iTorqueDiff2;
int iTorqueDiffSum;
int iTorqueSet=200;

int iFieldDiff1;
int iFieldDiff2;
int iFieldDiffSum;
int iFieldSet=0;

int adc_a1,adc_a2,adc_a3,adc_a4;
int adc_b1,adc_b2,adc_b3,adc_b4;
int iAngleRotorDiffNew;
int iAngleRotorDiffOld;
int iAngleRotorDiffCalculated;


unsigned a1RMS,a2RMS,a3RMS;
int ia1RMSMax=0;
int iSetLoopSpeed=0;
int iActualSpeed=0;
int idiffSpeed;
int idiffSpeed2; /* idiffSpeed with hyteresis*/
int iIdPeriod2=0;
int iIqPeriod2=0;
int iAngleDiffPeriod;
int iPowerMotor = 0;
int iStep1 =0;
int iMotHoldingTorque=0;
int iAngleRotor;


//----------------------------
int iFilterSumSpeed = 0;
int iSetValueSpeed	=  0;
int iSetInternSpeed	=  0;
int iSetInternSpeed2=  0;
int iSetSpeedRamp  	=  0;
int iSetSpeedSum    =  0;
int iSetSpeedNew    = 0;
int iMotDirection  	=  0;
int iControlFOC   	=  1;
int iEncoderOnOff   =  0;
//=======================================================
int iCountx;



int iUmotBoost  = 0;
int iUmotResult = 0;
int iUmotLast   = 0;
int iRampBlocked  = 0;
int iIndexPWM;
int iAngleRotorOld=0;

int iAnglePWM;
int iAnglePWMFromHall;
int iAnglePWMFromFOC;
int iAngleDiffSum;
int iAngleDiff;
int iAngleLast;
int iIntegralGain;

int iCountRMS  =0;
int iTriggerRMS=0;
int iCountAngle=0;
unsigned a1Square=0;
unsigned a2Square=0;
unsigned a1SquareMean=0;
unsigned a2SquareMean=0;
int a1=0;
int a2=0;
int iPhase1		=0;
int iPhase2		=0;
int iPhase1Sum	=0;
int iPhase2Sum	=0;

int iAlpha,iBeta;
int iAngleCurrent;
int iId;
int iIdPeriod;
int iIq;
int iIqPeriod;
int iFieldIntegral,iFieldProp;
int iTorqIntegral,iTorqProp;

int VsdRef1, VsqRef1;		// invers park
int VsdRef2, VsqRef2;		// invers park
int VsaRef, VsbRef;
int iIqProportional;
int iAngleInvPark;
int iVectorInvPark;
int sinx,cosx;
unsigned theta;  // angle
unsigned iVectorCurrent;
//----------------------------------------------
int iLoopCount=0;
int iCountDivFactor;

char cTriggerPeriod=0;  // one complete hall period
int iPwmOnOff 		=  1;
int iTorqueF0=0;

//============================================
int iPwmAddValue;
int iPwmIndexHigh;
int iAngleSensorLessPWM;
//============================================

int iPositionAbsolut=0;
int iPositionReferenz=0;
//int iPulsCountAcc;
int iPositionAbsolutNew;

//int iPositionAcc;
//int iPositionDec;
