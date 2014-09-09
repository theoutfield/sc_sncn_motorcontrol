/**
 * @file
 * @brief Brushed Motor Drive Server
 * @author Synapticon GmbH <support@synapticon.com>
*/

#include <brushed_dc_server.h>
#include <xs1.h>
#include <pwm_cli_inv.h>
#include <a4935.h>
#include <sine_table_big.h>
#include <adc_client_ad7949.h>
#include <hall_client.h>
#include <refclk.h>
#include <qei_client.h>
#include <brushed_dc_common.h>
#include <internal_config.h>
#include <watchdog.h>

static int init_state;

static void pwm_init_to_zero(chanend c_pwm_ctrl, t_pwm_control &pwm_ctrl)
{
    unsigned int pwm[3] = {0, 0, 0};  // PWM OFF
    pwm_share_control_buffer_address_with_server(c_pwm_ctrl, pwm_ctrl);
    update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);
}

static select on_client_request(chanend ? c_client, int & voltage, int & shutdown)
{
case !isnull(c_client) => c_client :> int command:

    switch (command) {
    case BDC_CMD_SET_VOLTAGE:
        c_client :> voltage;
        break;

    case BDC_CMD_CHECK_BUSY:    // init signal
        c_client <: init_state;
        break;

    case BDC_CMD_DISABLE_FETS:
        shutdown = 1;
        break;

    case BDC_CMD_ENABLE_FETS:
        shutdown = 0;
        voltage = 0;
        break;

    case BDC_CMD_FETS_STATE:
        c_client <: shutdown;
        break;

    default:
        break;
    }

    break;
}

static void bdc_internal_loop(port p_ifm_ff1, port p_ifm_ff2, port p_ifm_coastn,
                               t_pwm_control &pwm_ctrl,
                               chanend c_pwm_ctrl, chanend c_signal,
                               chanend ? c_voltage_p1, chanend ? c_voltage_p2, chanend ? c_voltage_p3)
{
    unsigned int command;
    unsigned int pwm[3] = { 0, 0, 0 };
    int voltage = 0;
    int shutdown = 0; // Disable FETS

    while (1) {
        if (shutdown == 1) {
            pwm[0] = -1;
            pwm[1] = -1;
            pwm[2] = -1;
        } else {
            if (voltage >= 0) {
                pwm[0] = voltage;
                pwm[1] = 0;
            } else {
                pwm[0] = 0;
                pwm[1] = -voltage;
            }
            pwm[2] = 0;
        }

        /* Limiting PWM values (and suppression of short pulses) is done in
         * update_pwm_inv() */
        update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);

/* FIXME: uncomment #pragma ordered once it is possible to use it in combination with
   select-functions */
//#pragma ordered
        select {
        case on_client_request(c_voltage_p1, voltage, shutdown);
        case on_client_request(c_voltage_p2, voltage, shutdown);
        case on_client_request(c_voltage_p3, voltage, shutdown);

        case c_signal :> command:
            if (command == CHECK_BUSY) { // init signal
                c_signal <: init_state;
            }
            break;
        }
    }
}


void bdc_loop(chanend c_watchdog, chanend c_signal,
              chanend ? c_voltage_p1, chanend ? c_voltage_p2, chanend ? c_voltage_p3,
              chanend c_pwm_ctrl,
              out port p_ifm_esf_rstn_pwml_pwmh, port p_ifm_coastn, port p_ifm_ff1, port p_ifm_ff2)
{
    const unsigned t_delay = 300*USEC_FAST;
    timer t;
    unsigned int ts;
    t_pwm_control pwm_ctrl;
    
    init_state = INIT_BUSY;

    pwm_init_to_zero(c_pwm_ctrl, pwm_ctrl);

    // enable watchdog
    t :> ts;
    t when timerafter (ts + 250000*4):> ts;
    watchdog_start(c_watchdog);

    t :> ts;
    t when timerafter (ts + t_delay) :> ts;

    a4935_initialize(p_ifm_esf_rstn_pwml_pwmh, p_ifm_coastn, A4935_BIT_PWML | A4935_BIT_PWMH);
    t when timerafter (ts + t_delay) :> ts;

    p_ifm_coastn :> init_state;

    bdc_internal_loop(p_ifm_ff1, p_ifm_ff2, p_ifm_coastn, pwm_ctrl, c_pwm_ctrl,
                      c_signal, c_voltage_p1, c_voltage_p2, c_voltage_p3);
}

