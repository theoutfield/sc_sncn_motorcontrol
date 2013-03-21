#include "varext.h"
#include "def.h"



int CheckIfNewPosition();
void CalcRampForPosition();


int CheckIfNewPosition()
{
int iTemp1;

	iTemp1 = iPositionAbsolutNew - iPositionReferenz;
	if(iTemp1 < 0) iTemp1 = -iTemp1;
	if(iTemp1 < 5) return(1);

	iTemp1 = iPositionAbsolutNew - iPositionReferenz;

	if(iPositionAbsolutNew < iPositionAbsolut)
	return(10);
	else
	return(5);
}


int CalcSpeedForPosition()
{
int iTemp1;
int iSpeed;
int iDirection;

	if(iPositionAbsolutNew < iPositionAbsolut)	iDirection=-1;
	else iDirection=1;

    iTemp1 = iPositionAbsolutNew - 	iPositionAbsolut;
	if(iTemp1 == 0) return(0);

	if(iTemp1 < 0) iTemp1 = -iTemp1;

	iSpeed = iTemp1;
	if(iSpeed > iMotPar[29])  iSpeed = iMotPar[28];       // defParPositionSpeedMax
	if(iSpeed < iMotPar[29])  iSpeed = iMotPar[29];       // defParPositionSpeedMin

	if(iDirection < 0)	iSpeed = -iSpeed;

return (iSpeed*65536);
}



void function_PositionControl()
{

}



void CalcRampForPosition(){

if(iRampBlocked==0)
{
  if(iSetUserSpeed >  iSetSpeedRamp) { iSetSpeedRamp += iMotPar[24]/8;  if(iSetSpeedRamp > iSetUserSpeed)  iSetSpeedRamp = iSetUserSpeed;}

  if(iSetUserSpeed <  iSetSpeedRamp) { iSetSpeedRamp -= iMotPar[25]/8;  if(iSetSpeedRamp < iSetUserSpeed)  iSetSpeedRamp = iSetUserSpeed;}
}
iRampBlocked = 0;


iSetSpeedSum -= iSetSpeedNew;
iSetSpeedSum += iSetSpeedRamp;

iSetSpeedNew = iSetSpeedSum;
if(iMotPar[26])
iSetSpeedNew /= iMotPar[26];   // smoothing factor

 iSetSpeed = iSetSpeedNew/65536;

}//end CalcRampForSpeed
