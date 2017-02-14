/**
 * @file biss_config.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <biss_struct.h>

// number of 32 bit bytes needed to save the biss frame before processing
// it must be enough to hold: some bits at start (3), ack and start bits, multiturn and singleturn data, error and warning bits and the crc (6)
// 2 bytes should be enough for a sensor up to 50 bits of position data (3 + 3 + 50 + 2 + 6 = 63)
#define BISS_FRAME_BYTES    2
