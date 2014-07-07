/**
 * @brief xC compatible boolean type definition
 */

#pragma once

#ifdef __XC__

typedef int bool;
#define false 0
#define true 1

#else

#include <stdbool.h>

#endif

