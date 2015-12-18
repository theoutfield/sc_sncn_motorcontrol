#include <CORE_C22-rev-a.inc>
#include <IFM_DC100-rev-b.inc>

#include <stdlib.h>

#include <pwm_service.h>
#include <pwm_test.h>

on tile[3]: PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;

/* ADC */
on stdcore[3]: out port p_adc_conv = XS1_PORT_1A;

int main (void)
{
    chan c_pwm_ctrl;            /* PWM control channel */
    chan c_adc_trigger;         /* PWM serivce will send out a control token on this channel
                                   which can be used to trigger ADC sampling during PWM off-time */

    par {
        /* PWM service */
        on stdcore[3]: {
            timer t;
            unsigned ts;

            t :> ts;
            t when timerafter (ts + 42000) :> void;
            pwm_triggered_service(pwm_ports, c_pwm_ctrl, c_adc_trigger);
        }


        /* PWM client */
        on stdcore[3]: {
            do_pwm_test(c_pwm_ctrl);
            exit (0);
        }


        /* Using the ADC trigger */
        on stdcore[3]: {
            unsigned char ct;   /* control token received from channel */

            p_adc_conv <: 0;

            while(1) {
                select {
                case inct_byref(c_adc_trigger, ct):
                    if (ct == XS1_CT_END) {
                        /* only output a short pulse for testing purposes;
                           the adc sampling/conversion function would be called here otherwise */
                        p_adc_conv <: 1;
                        p_adc_conv <: 0;
                    }
                    break;
                }
            }
        }
    }

    return 0;
}
