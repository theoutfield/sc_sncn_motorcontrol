/**
 * \file delay.xc
 * \brief Implementation of delay functions
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \author Pavan Kanajar <pkanajar@synapticon.com>
*/

#include <refclk.h>

void wait_micro_s(unsigned microseconds, int tile_id, timer t)
{
    unsigned time;

    t :> time;
    if(tile_id == 3) {
        t when timerafter(time + (microseconds * USEC_FAST)) :> time;
    } else {
        t when timerafter(time + (microseconds * USEC_STD)) :> time;
    }
    return;
}

void wait_ms(unsigned milliseconds, int tile_id, timer t)
{
    unsigned time;

    t :> time;
    if(tile_id == 3) {
        t when timerafter(time + milliseconds * MSEC_FAST) :> time;
    } else {
        t when timerafter(time + milliseconds * MSEC_STD) :> time;
    }
    return;
}

void wait_s(unsigned seconds, int tile_id, timer t)
{
    unsigned time;

    t :> time;
    if(tile_id == 3) {
        t when timerafter(time + seconds * SEC_FAST) :> time;
    } else {
        t when timerafter(time + seconds * SEC_STD) :> time;
    }
    return;
}
