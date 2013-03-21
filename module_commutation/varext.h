//------------- values from hall_input.xc ---------
extern int iHallActualSpeed,iHallAngle,iHallPositionAbsolut,iHallPinState;
extern int iHallSpeedValueIsNew;
extern int iHallActualSpeed;
extern int iHallPositionAbsolut;
extern int iEncoderActualSpeed,iEncoderAngle,iEncoderPositionAbsolut,iEncoderPinState;
extern int iEncoderSpeedValueIsNew;
extern int iEncoderActualSpeed;
extern int iEncoderPositionAbsolut;
extern int iEncoderNullReference ;
extern int iActualSpeed;
extern int iSpeedValueIsNew	;
extern int iDiffAngleRotor;
//-----------------------------------------------------------
extern int iMotPar[32];
extern int iMotValue[32];
extern int iMotCommand[16];
//--------- parameters ---------------------------------------
extern int iParRpmMotorMax;
extern int iParRpmUmotMax;
extern int iParUmotStart;
extern int iParUmotBoost;
extern int iParAngleUser;
extern int iParHysteresisPercent;
extern int iParDiffSpeedMax;
extern int iParAngleFromRPM;
extern int iParUmotIntegralLimit;
extern int iParPropGain;
extern int iParIntegralGain;
extern int iParRMS_RampLimit;
extern int iParRMS_PwmOff;
extern int iUpdateFlag;
//=========== motor values ===================================
extern int iUmotProfile;
extern int iUmotIntegrator;
extern int iUmotP;
extern int iUmotRpmLimit;
extern int iUmotMotor ;
extern int iUmotSquare;
extern int iUmotLinear;
extern int iUmotBoost  ;
extern int iUmotResult ;
extern int iUmotLast   ;
//------------------------------
extern int adc_a1,adc_a2,adc_a3,adc_a4;
extern int adc_b1,adc_b2,adc_b3,adc_b4;
extern unsigned a1RMS,a2RMS,a3RMS;
extern int ia1RMSMax;
extern int iPowerMotor ;
extern int iStep1 ;
extern int iMotDirection  	;
extern int iControlFOC   	;
extern int iEncoderOnOff   ;
//----------------------------
extern int iSetSpeed;
extern int iActualSpeed;
extern int idiffSpeed1;
extern int idiffSpeed2; /* idiffSpeed1 with hyteresis*/
extern int iFilterSumSpeed ;
//int iSetValueSpeed	=  0;
extern int iSetUserSpeed	;
//int iSetUserSpeed2=  0;
extern int iSetSpeedRamp  	;
extern int iSetSpeedSum    ;
extern int iSetSpeedNew    ;
//=======================================================
extern int iLoopCount;
extern int iCountDivFactor;
extern char cTriggerPeriod;
extern int iPwmOnOff 		;
extern int iCountx;
extern int iRampBlocked  ;
extern int iIndexPWM;
extern int iAngleRotorDiffNew;
extern int iAngleRotorDiffOld;
extern int iAngleRotorDiffCalculated;
extern int iAngleDiffPeriod;
extern int iAngleRotor;
extern int iAngleRotorOld;
extern int iAnglePWM;
extern int iAnglePWMFromHall;
extern int iAnglePWMFromFOC;
extern int iAngleDiffSum;
extern int iAngleDiff;
extern int iAngleLast;
extern int iIntegralGain;
extern int iCountRMS  ;
extern int iTriggerRMS;
extern int iCountAngle;
extern unsigned a1Square;
extern unsigned a2Square;
extern unsigned a1SquareMean;
extern unsigned a2SquareMean;
extern int a1;
extern int a2;
extern int iPhase1		;
extern int iPhase2		;
extern int iPhase1Sum	;
extern int iPhase2Sum	;
extern int iAlpha,iBeta;
extern int iAngleCurrent;
//--------------------------------------
extern int iMotHoldingTorque;
extern int iId;
extern int iIdPeriod;
extern int iFieldDiff1;
extern int iFieldDiff2;
extern int iFieldDiffSum;
extern int iFieldSet;
extern int iFieldIntegrator,iFieldProp;
extern int iFieldReferenz; // invers park
extern int iIq;
extern int iIqPeriod;
extern int iTorqueDiff1;
extern int iTorqueDiff2;
extern int iTorqueDiffSum;
extern int iTorqueSet;
extern int iIdPeriod2;
extern int iIqPeriod2;
extern int iTorqueUmotIntegrator;
extern int iTorqueProp;
extern int iTorqueLimit;
extern int iTorqueF0;
extern int iTorqueUser;
extern int iTorqueReferenz; // invers park
//----------------------------------------------
extern int VsaRef, VsbRef;
extern int iIqProportional;
extern int iAngleInvPark;
extern int iVectorInvPark;
extern int sinx,cosx;
extern unsigned theta;  // angle
extern unsigned iVectorCurrent;
//============================================
extern int iPwmAddValue;
extern int iPwmIndexHigh;
extern int iAngleSensorLessPWM;
//============================================
extern int iPositionAbsolut;
extern int iPositionReferenz;
extern int iPositionAbsolutNew;

