/**
 * @file
 * @brief Brushed Motor Drive Server
 * @author Synapticon GmbH <support@synapticon.com>
*/

#include <bdc_motorcontrol.h>
#include <pwm_service_client.h>
#include <a4935.h>

static int init_state;

static void pwm_init_to_zero(chanend c_pwm_ctrl, t_pwm_control &pwm_ctrl)
{
    unsigned int pwm[3] = {0, 0, 0};  // PWM OFF
    pwm_share_control_buffer_address_with_server(c_pwm_ctrl, pwm_ctrl);
    update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);
}

[[combinable]]
static void bdc_internal_loop(FetDriverPorts &fet_driver_ports,
                               t_pwm_control &pwm_ctrl,
                               chanend c_pwm_ctrl,
                               MotorcontrolConfig &motorcontrol_config,
                               interface MotorcontrolInterface server i_motorcontrol[4])
{
    timer t;
    unsigned int ts;

    unsigned int pwm[3] = { 0, 0, 0 };
    int voltage = 0;
    int shutdown = 0; // Disable FETS
    int pwm_half = (PWM_MAX_VALUE - PWM_DEAD_TIME) >> 1;

    int notification = MOTCTRL_NTF_EMPTY;

    t :> ts;
    while (1) {
        select {

        case t when timerafter(ts + USEC_FAST * motorcontrol_config.commutation_loop_period) :> ts:

                if (shutdown == 1) {
                    pwm[0] = -1;
                    pwm[1] = -1;
                    pwm[2] = -1;
                } else {
                    if (voltage >= 0) {
                        if(voltage <= pwm_half)
                        {
                            pwm[0] = pwm_half + voltage;
                            pwm[1] = pwm_half;
                        }
                        else if(voltage > pwm_half && voltage < BDC_PWM_CONTROL_LIMIT)
                        {
                            pwm[0] = pwm_half + pwm_half;
                            pwm[1] = pwm_half - (voltage - pwm_half);
                        }
                        else if(voltage >= BDC_PWM_CONTROL_LIMIT)
                        {
                            pwm[0] = pwm_half + pwm_half;
                            pwm[1] = PWM_MIN_LIMIT;
                        }
                    } else {
                        if(-voltage <= pwm_half)
                        {
                            pwm[0] = pwm_half;
                            pwm[1] = pwm_half - voltage;
                        }
                        else if(-voltage > pwm_half && -voltage < BDC_PWM_CONTROL_LIMIT)
                        {
                            pwm[0] = pwm_half - (-voltage - pwm_half);
                            pwm[1] = pwm_half + pwm_half;
                        }
                        else if(-voltage >= BDC_PWM_CONTROL_LIMIT)
                        {
                            pwm[0] = PWM_MIN_LIMIT;
                            pwm[1] = pwm_half + pwm_half;
                        }
                    }
                    pwm[2] = pwm_half;
                }

                /* Limiting PWM values (and suppression of short pulses) is done in
                 * update_pwm_inv() */
                update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);

                break;

        case i_motorcontrol[int i].get_notification() -> int out_notification:

            out_notification = notification;
            break;

        case i_motorcontrol[int i].set_voltage(int new_voltage):

            voltage = new_voltage;
            break;

        case i_motorcontrol[int i].check_busy() -> int state_return:

                  state_return = init_state;
                  break;

        case i_motorcontrol[int i].set_calib(int flag) -> int out_offset:
                break;
/*
        case i_motorcontrol[int i].disable_fets():

                shutdown = 1;
                break;
*/
        case i_motorcontrol[int i].set_fets_state(int new_state):

                if(new_state == 0){
                    shutdown = 1;
                }else{
                    shutdown = 0;
                    voltage = 0;
                }

                break;

        case i_motorcontrol[int i].get_fets_state() -> int fets_state:
                fets_state = !shutdown;
                break;
        case i_motorcontrol[int i].set_sensor(int new_sensor):
                break;
        case i_motorcontrol[int i].set_sensor_offset(int in_offset):
                break;
        case i_motorcontrol[int i].set_config(MotorcontrolConfig new_config):
                break;
        case i_motorcontrol[int i].get_config() -> MotorcontrolConfig out_config:

                  out_config = motorcontrol_config;
                  break;
        case i_motorcontrol[int i].set_all_parameters(HallConfig in_hall_config,
                                                            QEIConfig in_qei_config,
                                                            MotorcontrolConfig in_commutation_config):
               break;
        }
    }
}

[[combinable]]
void bdc_loop(chanend c_pwm_ctrl,
                interface WatchdogInterface client i_watchdog,
                interface MotorcontrolInterface server i_commutation[4],
                FetDriverPorts &fet_driver_ports,
                MotorcontrolConfig &motorcontrol_config)
{
    const unsigned t_delay = 300*USEC_FAST;
    timer t;
    unsigned int ts, check_fet;
    t_pwm_control pwm_ctrl;
    
    init_state = INIT_BUSY;

    pwm_init_to_zero(c_pwm_ctrl, pwm_ctrl);

    // enable watchdog
    t :> ts;
    t when timerafter (ts + 250000*4):> ts;
    i_watchdog.start();

    t :> ts;
    t when timerafter (ts + t_delay) :> ts;

    if(!isnull(fet_driver_ports.p_esf_rst_pwml_pwmh) && !isnull(fet_driver_ports.p_coast)){
        a4935_initialize(fet_driver_ports.p_esf_rst_pwml_pwmh, fet_driver_ports.p_coast, A4935_BIT_PWML | A4935_BIT_PWMH);
        t when timerafter (ts + t_delay) :> ts;
        fet_driver_ports.p_coast :> init_state;
    }

    if(!isnull(fet_driver_ports.p_coast)){
        fet_driver_ports.p_coast :> check_fet;
        init_state = check_fet;
    }
    else {
        init_state = 1;
    }

    t :> ts;
    //This while + select is just to make it combinable
    while(1){

        select{

            case t when timerafter(ts+0) :> void:
                    bdc_internal_loop(fet_driver_ports, pwm_ctrl, c_pwm_ctrl, motorcontrol_config, i_commutation);
                    break;
        }
    }
}
