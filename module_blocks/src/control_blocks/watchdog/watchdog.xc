/**
 * @file watchdog.xc
 * @brief Watchdog Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <xs1.h>

enum { WD_CMD_EN_MOTOR,
       WD_CMD_DIS_MOTOR,
       WD_CMD_TICK,
       WD_CMD_START };

void watchdog_start(chanend c_watchdog)
{
    c_watchdog <: WD_CMD_START;
}

void run_watchdog(chanend c_watchdog, out port p_wd_tick, out port p_shared_leds_wden)
{
    unsigned int cmd, wd_enabled = 1, shared_out = 0xe, tick_out = 0;
    unsigned int ts, ts2;
    timer t;

    t :> ts;

    // Loop forever processing commands
    while (1) {
        select {
            // Get a command from the out loop
        case c_watchdog :> cmd:
            switch (cmd) {
            case WD_CMD_START: // produce a rising edge on the WD_EN
                shared_out &= ~0x1;
                p_shared_leds_wden <: shared_out; // go low
                t :> ts2;
                t when timerafter(ts2+25000) :> ts2;
                shared_out |= 0x1;
                p_shared_leds_wden <: shared_out; // go high
                break;

                // if the watchdog is enabled, kick it
            case WD_CMD_DIS_MOTOR:
                // mark that the watchdog should now not run
                wd_enabled = 0;
                break;
            }
            break;

        case t when timerafter(ts + 250000) :> ts:
            if (wd_enabled == 1) {
                tick_out ^= 0x1;
            }

            break;

            // Do a case: Separate the LED bits from the port XS1_PORT_4B. LSB enable watchdog.
        }

        // Send out the new value to the shared port
        p_shared_leds_wden <: shared_out;
        p_wd_tick <: tick_out;
    }
}
