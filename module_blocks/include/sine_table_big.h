/**
 * @file sine_table_big.h
 * @brief Sine Lookup table declarations
 * @author Ludwig Orgler <lorgler@synapticon.com>
 * @author Martin Schwarz <mschwarz@synapticon.com>
 */

#pragma once

#include <stdint.h>

int sine_third_expanded(int angle);

int sine_table_expanded(int angle);
