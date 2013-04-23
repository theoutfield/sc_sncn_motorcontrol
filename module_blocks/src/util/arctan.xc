/**
 * \file arctan.xc
 *	Arc tangent calculations
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Authors:  Ludwig Orgler <orgler@tin.it> & Martin Schwarz <mschwarz@synapticon.com>
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/

#include "sine_table_big.h"

int arctg1(int Real, int Imag)
{
	int iAngleResult;
	int AbsX,AbsY;
	unsigned char cFlagOver45;
	int Mag;

	AbsX = Real;
	AbsY = Imag;

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

	if(Real >= 0)
	{                                                   		// 1. or 4. quadrant
		if(Imag  <  0)  iAngleResult = 4096 - iAngleResult;     // 4. quadrant
																// if 1.quadrant all okay
	}
	else
	{      // if negativ 2. or 3. quadrant
		if(Imag < 0)    iAngleResult += 2048;                   // 3. Q
		else            iAngleResult  = 2048 - iAngleResult;    // 2. Quadrant
	}
	return  iAngleResult;
}
