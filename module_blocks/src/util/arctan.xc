#include "sine_table_big.h"

int arctg1(int iReell, int iImm)
{
	int iAngleResult;
	int AbsX,AbsY;
	unsigned char cFlagOver45;
	int Mag;

	AbsX = iReell;
	AbsY = iImm;

	if(AbsX <0)  AbsX = -AbsX;
	if(AbsY <0)  AbsY = -AbsY;

	#define defFactor 65536/64

	if(AbsY < AbsX)
	{
	  cFlagOver45 = 0;
	  AbsY *= defFactor;
	  if(AbsX==0) Mag = defFactor;
	  else
	  {
	   if    (AbsY==0) Mag = 0;
	   else  Mag = AbsY / AbsX;
	  }
	}
	else
	{
	  cFlagOver45 = 1;
	  AbsX *= defFactor;
	  if(AbsY==0) Mag = defFactor;
	  else
	  {
	   if    (AbsX==0) Mag = 0;
	   else  Mag = AbsX / AbsY;
	  }
	}

	if(Mag > 1023) Mag = 1023;
	iAngleResult = arctg_table[Mag];

	if(cFlagOver45) iAngleResult = 1024 - iAngleResult;

	if(iReell >= 0)
	{                                                   		// 1. or 4. quadrant
		if(iImm  <  0)  iAngleResult = 4096 - iAngleResult;     // 4. quadrant
																// if 1.quadrant all okay
	}
	else
	{      // if negativ 2. or 3. quadrant
		if(iImm < 0)    iAngleResult += 2048;                   // 3. Q
		else            iAngleResult  = 2048 - iAngleResult;    // 2. Quadrant
	}
	return  iAngleResult;
}
