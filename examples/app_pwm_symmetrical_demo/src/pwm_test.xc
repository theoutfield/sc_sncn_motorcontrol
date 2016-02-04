#include <pwm_service_client.h>

#include <stdlib.h>

/* Send new PWM data after 3 periods.
   T_period_pwm = 55555ns, T_period_refclk=4ns (assuming 25MHz refclk) */
#define PWM_CLIENT_UPDATE_DELAY ( (3*56000) / 4 )

/* Shared buffer for communication between PWM client and service */
static t_pwm_control pwm_ctrl;

/* Use PWM client API to send new values to the PWM service */
void do_pwm_test(chanend c_pwm_ctrl)
{
    const unsigned int testdata[] = { 100, 302, 200,
                                      0, 0, 0,
                                      000, 00, 8000,
                                      -1, -1, -1,
                                      PWM_MAX_VALUE/2, PWM_MAX_VALUE/2, PWM_MAX_VALUE/2 };

    /* number of tripples in testdata */
    const size_t num_data = (sizeof (testdata)) / (sizeof (unsigned int) * 3);

    unsigned int pwm_values[3] = { 0, 0, 0 };

    timer t;
    unsigned ts;

        /* share buffer address with pwm service and send initial pwm values */
        pwm_share_control_buffer_address_with_server(c_pwm_ctrl, pwm_ctrl);
        update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm_values);

        t :> ts;
        t when timerafter (ts + PWM_CLIENT_UPDATE_DELAY) :> void;

        while(1){

            for (int i = 0; i < (3*num_data); ) {
                pwm_values[0] = testdata[i++];
                pwm_values[1] = testdata[i++];
                pwm_values[2] = testdata[i++];
                update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm_values);

                ts += PWM_CLIENT_UPDATE_DELAY;
                t when timerafter (ts) :> void;
            }
        }

}
