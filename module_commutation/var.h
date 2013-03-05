int iDiffAngleHall;
int iAngleXXX;
int iUmotSquare;
int iUmotLinear;


int iMotPar[32];
int iMotValue[32];
int iMotCommand[16];

//--------- parameters ---------------------------------------
int iParRpmMotorMax;
int iParDefSpeedMax;
int iParRPMreference;
int iParSpeedKneeUmot;
int iParAngleUser;
int iParHysteresisPercent;
int iParDiffSpeedMax;
int iParAngleFromRPM;
int iParUmotIntegralLimit;
int iParPropGain;
int iParIntegralGain;
int iParUmotSocket;
int iParUmotBoost;
int iParUmotStart;
int iParRMS_RampLimit;
int iParRMS_PwmOff;
int iUpdateFlag=0;

//=========== motor values ===================================
int iUmotProfile;
int iUmotIntegrator=0;
int iUmotP;
int iUmotMotor = 0;   	// follows iUmotResult

int iTorqueDiff1;
int iTorqueDiff2;
int iTorqueDiffSum;
int iTorqueSet=200;

int iFieldDiff1;
int iFieldDiff2;
int iFieldDiffSum;
int iFieldSet=0;

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
//int iParAngleCorrVal;
//int iParAngleCorrMax;
//------------- values from hall_input.xc ---------
int iActualSpeedHall;
int iAngleFromHall;
int iHallPositionAbsolut;
int iPinStateHall;

int iActualSpeedEncoder;
int iAngleFromEncoder;
int iEncoderPositionAbsolut;
int iEncoderPositionZero=0;
int iPinStateEncoder;
//----------------------------
int iSetValueSpeed	=  0;
int iSetInternSpeed	=  0;
int iSetInternSpeed2=  0;
int iSetSpeedRamp  	=  0;
int iSetSpeedSum    =  0;
int iSetSpeedNew    = 0;
int iMotDirection  	=  0;
int iControlFOC   	=  1;
//=======================================================
int iCountx;
//int iStepRamp=0;


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
//-----------------------------------------------
int iLoopCount=0;
int iCountDivFactor;

char cTriggerPeriod=0;  // one complete hall period
int iPwmOnOff 		= 1;
int iSpeedValueNew	=  0;  // if speed from hall is a new value
int iTorqueF0=0;

//============================================
int iPwmAddValue,iPwmIndexHigh;
int iHallNullPosition=0;
int iEncoderNullPosition=0;

int iPositionAbsolut=0;
int iPositionReferenz=0;
int iPulsCountAcc;
int iPositionAbsolutNew;
int iPositionAcc;
int iPositionDec;
