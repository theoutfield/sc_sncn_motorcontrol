
#pragma once

#define HALL  1
#define QEI   2

/* FIXME: replace with enum once duplicate definitions are removed */
//#define QEI_WITH_NO_INDEX 0
//#define QEI_WITH_INDEX    1
//enum { QEI_WITH_NO_INDEX, QEI_WITH_INDEX };
#define QEI_WITH_NO_INDEX 0
#define QEI_WITH_INDEX    1

enum { UNSET, SET };

enum { ACTIVE_HIGH=1, ACTIVE_LOW };

enum { STAR_WINDING=1, DELTA_WINDING };
enum { HOMING_NEGATIVE_SWITCH=1, HOMING_POSITIVE_SWITCH };

