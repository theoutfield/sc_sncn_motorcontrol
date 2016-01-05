#include <CORE_C22-rev-a.bsp>
#include <IFM_DC100-rev-b.bsp>

#include <stdlib.h>

#include <pwm_service.h>
#include <pwm_test.h>

on tile[IFM_TILE]: PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;

int main (void)
{
    chan c_pwm_ctrl;            /* PWM control channel */
    chan c_adc_trigger;         /* PWM serivce will send out a control token on this channel
                                   which can be used to trigger ADC sampling during PWM off-time */

    par {
        /* PWM service */
        on tile[IFM_TILE]: {
            timer t;
            unsigned ts;

            t :> ts;
            t when timerafter (ts + 42000) :> void;
            pwm_triggered_service(pwm_ports, c_pwm_ctrl, c_adc_trigger);
        }


        /* PWM client */
        on tile[IFM_TILE]: {
            do_pwm_test(c_pwm_ctrl);
            exit (0);
        }
    }

    return 0;
}
