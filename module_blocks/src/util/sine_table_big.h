/**
 * \file sine_table_big.h
 *	Sine Table Declarations
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Authors:  Martin Schwarz <mschwarz@synapticon.com> & Ludwig Orgler <orgler@tin.it>
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/

#define SPACE_VECTOR

extern short arctg_table[1024+6];

extern short sine_table[256];

#ifndef SPACE_VECTOR
extern short sine_third[256];
#else
//** space vector table with 1024 base points ** Umax = 6944
extern short SPACE_TABLE[1024];
#endif
