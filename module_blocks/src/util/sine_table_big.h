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

int arctg1(int Real, int Imag);

extern short arctg_table[];

extern short sine_third[257];

extern short sine_table[257];

int sine_third_expanded(int angle);

int sine_table_expanded(int angle);
