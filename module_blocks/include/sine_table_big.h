/**
 * @file sine_table_big.h
 * @brief Sine Loopup table declarations
 * @author Ludwig Orgler <lorgler@synapticon.com>
 * @author Martin Schwarz <mschwarz@synapticon.com>
 */

#pragma once

#include <stdint.h>

int arctg1(int Real, int Imag);

extern uint16_t arctg_table[];

extern uint16_t sine_third[257];

extern uint16_t sine_table[257];

int sine_third_expanded(int angle);

int sine_table_expanded(int angle);
