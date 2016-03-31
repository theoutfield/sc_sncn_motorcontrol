#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>

#include <pwm_service.h>
#include <pwm_test.h>

#define BRAKE_ENABLE 1

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;

int main (void)
{
    chan c_pwm_ctrl;            /* PWM control channel */

    par {
        /* PWM service */
        on tile[IFM_TILE]: {
            timer t;
            unsigned ts;

            t :> ts;
            t when timerafter (ts + 42000) :> void;
            pwm_service(pwm_ports, c_pwm_ctrl, BRAKE_ENABLE);
        }

        /* PWM client */
        on tile[IFM_TILE]: {
            do_pwm_test(c_pwm_ctrl);
        }
    }

    return 0;
}
