
#pragma once

#define HALL  1
#define QEI   2
#define QEI_1 3

enum { UNSET, SET };
enum { ERROR, SUCCESS };

enum { ACTIVE_HIGH=1, ACTIVE_LOW };

enum { STAR_WINDING=1, DELTA_WINDING };
enum { HOMING_NEGATIVE_SWITCH=1, HOMING_POSITIVE_SWITCH };
enum { QEI_WITH_NO_INDEX, QEI_WITH_INDEX };
