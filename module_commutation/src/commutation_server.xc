/**
 * @file comm_loop_server.xc
 * @brief Commutation Loop based on sinusoidal commutation method
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <commutation_server.h>
#include <commutation_common.h>
#include <watchdog.h>
#include <xs1.h>
#include <stdlib.h>
#include <pwm_config.h>
#include <pwm_cli_inv.h>
#include <a4935.h>
#include <sine_table_big.h>
#include <adc_client_ad7949.h>
#include <refclk.h>
#include <qei_client.h>
#include <stdio.h>
#include <internal_config.h>


static void commutation_init_to_zero(chanend c_pwm_ctrl, t_pwm_control & pwm_ctrl)
{
    unsigned int pwm[3] = {0, 0, 0};  // PWM OFF (break mode; short all phases)
    pwm_share_control_buffer_address_with_server(c_pwm_ctrl, pwm_ctrl);
    update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);
}


/* Sinusoidal based commutation functions */

/*FixMe: restore select functions when sopported in combinable functions or replace by array of interfaces
 *
static select on_client_request(chanend ? c_client, commutation_par & commutation_params, int & voltage,
                                int & sensor_select, int init_state, int & shutdown)
{
case !isnull(c_client) => c_client :> int command:
    switch (command) {
    case COMMUTATION_CMD_SET_VOLTAGE:                // set voltage
        c_client :> voltage;    //STAR_WINDING
        if (commutation_params.winding_type == DELTA_WINDING) {
            voltage = -voltage;
        }
        break;

    case COMMUTATION_CMD_SET_PARAMS:
        c_client :> commutation_params.angle_variance;
        c_client :> commutation_params.nominal_speed;
        c_client :> commutation_params.hall_offset_clk;
        c_client :> commutation_params.hall_offset_cclk;
        c_client :> commutation_params.winding_type;
        break;

    case COMMUTATION_CMD_SENSOR_SELECT:
        c_client :> sensor_select;
        break;

    case CHECK_BUSY:            // init signal
        c_client <: init_state;
        break;

    case COMMUTATION_CMD_DISABLE_FETS:
        shutdown = 1;
        break;

    case COMMUTATION_CMD_ENABLE_FETS:
        shutdown = 0;
        voltage = 0;
        break;

    case COMMUTATION_CMD_FETS_STATE:
        c_client <: shutdown;
        break;

    default:
        break;
    }
    break;
}
*/

[[combinable]]
void commutation_sinusoidal(chanend ?c_hall, chanend ?c_qei, client interface AMS ?i_ams, chanend ?c_signal, chanend ? c_watchdog,
                            chanend ? c_commutation_p1, chanend ? c_commutation_p2,
                            chanend ? c_commutation_p3, chanend c_pwm_ctrl,
                            out port ? p_ifm_esf_rstn_pwml_pwmh, port ? p_ifm_coastn,
                            port ? p_ifm_ff1, port ? p_ifm_ff2,
                            hall_par & hall_params, qei_par & qei_params,
                            commutation_par &commutation_params)
{
    const unsigned t_delay = 300*USEC_FAST;
    timer t;
    unsigned int ts;
    t_pwm_control pwm_ctrl;
    int check_fet;
    int init_state = INIT_BUSY;
    unsigned int command;
    unsigned int pwm[3] = { 0, 0, 0 };
    int angle_pwm = 0;
    int angle = 0;
    int voltage = 0;
    int pwm_half = PWM_MAX_VALUE>>1;
    int max_count_per_hall = qei_params.real_counts/hall_params.pole_pairs;
    int angle_offset = (4096 / 6) / (2 * hall_params.pole_pairs);

    int fw_flag = 0;
    int bw_flag = 0;
    int zero_offset_tuning_flag = 0;
    int nominal_speed;
    int shutdown = 0; //Disable FETS
    int sensor_select = ABS_AMS;//HALL;
    qei_velocity_par qei_velocity_params;

    timer t_loop;
    unsigned int start_time, end_time;

    init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED);

    printf("*************************************\n    COMMUTATION SERVER STARTING\n*************************************\n");

    commutation_init_to_zero(c_pwm_ctrl, pwm_ctrl);

    // enable watchdog
    t :> ts;
    t when timerafter (ts + 250000*4):> ts; /* FIXME: replace with constant */
    if(!isnull(c_watchdog))
        watchdog_start(c_watchdog);

    t :> ts;
    t when timerafter (ts + t_delay) :> ts;

    if (!isnull(p_ifm_esf_rstn_pwml_pwmh) && !isnull(p_ifm_coastn)){
        a4935_initialize(p_ifm_esf_rstn_pwml_pwmh, p_ifm_coastn, A4935_BIT_PWML | A4935_BIT_PWMH);
        t when timerafter (ts + t_delay) :> ts;
    }

    if(!isnull(p_ifm_coastn)){
        p_ifm_coastn :> check_fet;
        init_state = check_fet;
    }
    else {
        if(!isnull(c_watchdog))
            watchdog_enable_motors(c_watchdog);
        init_state = 1;
    }


    init_qei_velocity_params(qei_velocity_params);

    while (1) {

  //      t_loop :> start_time;

        select {

            case t when timerafter(ts + USEC_FAST*(1000/COMMUTATION_LOOP_FREQUENCY_KHZ)) :> ts: //XX kHz commutation loop
                if (sensor_select == HALL && !isnull(c_hall)) {
                    //hall only
                    if (zero_offset_tuning_flag == 1){
                        angle = 0;
                    }else{
                        angle = get_hall_position(c_hall);
                    }
                } else if (sensor_select == QEI && !isnull(c_qei)) {
                    if (zero_offset_tuning_flag == 1){
                        angle = 0;
                    }else{
                        { angle, fw_flag, bw_flag } = get_qei_sync_position(c_qei);
                        angle = (angle << 12) / max_count_per_hall;
                        if ((voltage >= 0 && fw_flag == 0) || (voltage < 0 && bw_flag == 0)) {
                            angle = get_hall_position(c_hall);
                        }
                    }
                } else if (sensor_select == ABS_AMS && !isnull(i_ams)){
                    if (zero_offset_tuning_flag == 1){
                        angle = 0;
                    }else{
                        angle = i_ams.get_angle_electrical();
                    }
                } else {
                    /* stop PWM */
                    pwm[0] = -1;
                    pwm[1] = -1;
                    pwm[2] = -1;
                    break;
                }

                if (shutdown == 1) {    /* stop PWM */
                    pwm[0] = -1;
                    pwm[1] = -1;
                    pwm[2] = -1;
                } else {
                    if (voltage >= 0) {
                        if (sensor_select == HALL) {
                            angle_pwm = ((angle + commutation_params.hall_offset_clk) >> 2) & 0x3ff;
                        } else if (sensor_select == QEI) {
                            angle_pwm = ((angle + commutation_params.qei_forward_offset) >> 2) & 0x3ff; //512
                        } else if (sensor_select == ABS_AMS){
                            angle_pwm = (angle >> 2) & 0x3ff;
                        }
                        pwm[0] = ((sine_third_expanded(angle_pwm)) * voltage) / pwm_half + pwm_half; // 6944 -- 6867range
                        angle_pwm = (angle_pwm + 341) & 0x3ff; /* +120 degrees (sine LUT size divided by 3) */
                        pwm[1] = ((sine_third_expanded(angle_pwm)) * voltage) / pwm_half + pwm_half;
                        angle_pwm = (angle_pwm + 342) & 0x3ff;
                        pwm[2] = ((sine_third_expanded(angle_pwm)) * voltage) / pwm_half + pwm_half;
                    } else { /* voltage < 0 */
                        if (sensor_select == HALL) {
                            angle_pwm = ((angle + commutation_params.hall_offset_cclk) >> 2) & 0x3ff;
                        } else if (sensor_select == QEI) {
                            angle_pwm = ((angle + commutation_params.qei_backward_offset) >> 2) & 0x3ff; //3100
                        } else if (sensor_select == ABS_AMS && !isnull(i_ams)){
                            angle_pwm = ((angle + 2048) >> 2) & 0x3ff;
                        }
                        pwm[0] = ((sine_third_expanded(angle_pwm)) * -voltage) / pwm_half + pwm_half;
                        angle_pwm = (angle_pwm + 341) & 0x3ff;
                        pwm[1] = ((sine_third_expanded(angle_pwm)) * -voltage) / pwm_half + pwm_half;
                        angle_pwm = (angle_pwm + 342) & 0x3ff;
                        pwm[2] = ((sine_third_expanded(angle_pwm)) * -voltage) / pwm_half + pwm_half;
                    }

                }

                /* Limiting PWM values (and suppression of short pulses) is done in
                 * update_pwm_inv() */
                update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);
                break;

            case !isnull(c_commutation_p1) => c_commutation_p1 :> int command:
                switch (command) {
                    case COMMUTATION_CMD_SET_VOLTAGE:                // set voltage
                        c_commutation_p1 :> voltage;    //STAR_WINDING
                        if (commutation_params.winding_type == DELTA_WINDING) {
                            voltage = -voltage;
                        }
                        zero_offset_tuning_flag = 0;
                        break;

                    case COMMUTATION_CMD_SET_PARAMS:
                        c_commutation_p1 :> commutation_params.angle_variance;
                        c_commutation_p1 :> commutation_params.nominal_speed;
                        c_commutation_p1 :> commutation_params.hall_offset_clk;
                        c_commutation_p1 :> commutation_params.hall_offset_cclk;
                        c_commutation_p1 :> commutation_params.winding_type;
                        break;

                    case COMMUTATION_CMD_SENSOR_SELECT:
                        c_commutation_p1 :> sensor_select;
                        break;

                    case CHECK_BUSY:            // init signal
                        c_commutation_p1 <: init_state;
                        break;

                    case COMMUTATION_CMD_DISABLE_FETS:
                        shutdown = 1;
                        break;

                    case COMMUTATION_CMD_ENABLE_FETS:
                        shutdown = 0;
                        voltage = 0;
                        break;

                    case COMMUTATION_CMD_FETS_STATE:
                        c_commutation_p1 <: shutdown;
                        break;

                    case COMMUTATION_CMD_SET_TO_ZERO_ANGLE:
                        c_commutation_p1 :> voltage;    //STAR_WINDING
                        if (commutation_params.winding_type == DELTA_WINDING) {
                            voltage = -voltage;
                        }
                        zero_offset_tuning_flag = 1;
                        break;

                    default:
                        break;
                }
                break;

                case !isnull(c_commutation_p2) => c_commutation_p2 :> int command:
                    switch (command) {
                        case COMMUTATION_CMD_SET_VOLTAGE:                // set voltage
                            c_commutation_p2 :> voltage;    //STAR_WINDING
                            if (commutation_params.winding_type == DELTA_WINDING) {
                                voltage = -voltage;
                            }
                            zero_offset_tuning_flag = 0;
                            break;

                        case COMMUTATION_CMD_SET_PARAMS:
                            c_commutation_p2 :> commutation_params.angle_variance;
                            c_commutation_p2 :> commutation_params.nominal_speed;
                            c_commutation_p2 :> commutation_params.hall_offset_clk;
                            c_commutation_p2 :> commutation_params.hall_offset_cclk;
                            c_commutation_p2 :> commutation_params.winding_type;
                            break;

                        case COMMUTATION_CMD_SENSOR_SELECT:
                            c_commutation_p2 :> sensor_select;
                            break;

                        case CHECK_BUSY:            // init signal
                            c_commutation_p2 <: init_state;
                            break;

                        case COMMUTATION_CMD_DISABLE_FETS:
                            shutdown = 1;
                            break;

                        case COMMUTATION_CMD_ENABLE_FETS:
                            shutdown = 0;
                            voltage = 0;
                            break;

                        case COMMUTATION_CMD_FETS_STATE:
                            c_commutation_p2 <: shutdown;
                            break;

                        case COMMUTATION_CMD_SET_TO_ZERO_ANGLE:
                            c_commutation_p1 :> voltage;    //STAR_WINDING
                            if (commutation_params.winding_type == DELTA_WINDING) {
                                voltage = -voltage;
                            }
                            zero_offset_tuning_flag = 1;
                            break;

                        default:
                            break;
                    }
                    break;

                    case !isnull(c_commutation_p3) => c_commutation_p3 :> int command:
                        switch (command) {
                            case COMMUTATION_CMD_SET_VOLTAGE:                // set voltage
                                c_commutation_p3 :> voltage;    //STAR_WINDING
                                if (commutation_params.winding_type == DELTA_WINDING) {
                                    voltage = -voltage;
                                }
                                zero_offset_tuning_flag = 0;
                                break;

                            case COMMUTATION_CMD_SET_PARAMS:
                                c_commutation_p3 :> commutation_params.angle_variance;
                                c_commutation_p3 :> commutation_params.nominal_speed;
                                c_commutation_p3 :> commutation_params.hall_offset_clk;
                                c_commutation_p3 :> commutation_params.hall_offset_cclk;
                                c_commutation_p3 :> commutation_params.winding_type;
                                break;

                            case COMMUTATION_CMD_SENSOR_SELECT:
                                c_commutation_p3 :> sensor_select;
                                break;

                            case CHECK_BUSY:            // init signal
                                c_commutation_p3 <: init_state;
                                break;

                            case COMMUTATION_CMD_DISABLE_FETS:
                                shutdown = 1;
                                break;

                            case COMMUTATION_CMD_ENABLE_FETS:
                                shutdown = 0;
                                voltage = 0;
                                break;

                            case COMMUTATION_CMD_FETS_STATE:
                                c_commutation_p3 <: shutdown;
                                break;

                            case COMMUTATION_CMD_SET_TO_ZERO_ANGLE:
                                c_commutation_p1 :> voltage;    //STAR_WINDING
                                if (commutation_params.winding_type == DELTA_WINDING) {
                                    voltage = -voltage;
                                }
                                zero_offset_tuning_flag = 1;
                                break;

                            default:
                                break;
                        }
                        break;
/*FixMe: restore select functions when supported in combinable functions or replace by array of interfaces
 *
            case on_client_request(c_commutation_p1, commutation_params, voltage, sensor_select,
                                   init_state, shutdown);
            case on_client_request(c_commutation_p2, commutation_params, voltage, sensor_select,
                                   init_state, shutdown);
            case on_client_request(c_commutation_p3, commutation_params, voltage, sensor_select,
                                   init_state, shutdown);
*/
            case !isnull(c_signal) => c_signal :> command:
                if (command == CHECK_BUSY) {      // init signal
                    c_signal <: init_state;
                } else if (command == SET_COMM_PARAM_ECAT) {
                    c_signal :> hall_params.pole_pairs;
                    c_signal :> qei_params.index;
                    c_signal :> qei_params.max_ticks_per_turn;
                    c_signal :> qei_params.real_counts;
                    c_signal :> nominal_speed;
                    c_signal :> commutation_params.hall_offset_clk;
                    c_signal :> commutation_params.hall_offset_cclk;
                    c_signal :> commutation_params.winding_type;
                    commutation_params.angle_variance = (60 * 4096) / (hall_params.pole_pairs * 2 * 360);
                    if (hall_params.pole_pairs < 4) {
                        commutation_params.nominal_speed = nominal_speed * 4;
                    } else if (hall_params.pole_pairs >= 4) {
                        commutation_params.nominal_speed = nominal_speed;
                    }
                    commutation_params.qei_forward_offset = 0;
                    commutation_params.qei_backward_offset = 0;
                    voltage = 0;
                    max_count_per_hall = qei_params.real_counts / hall_params.pole_pairs;
                    angle_offset = (4096 / 6) / (2 * hall_params.pole_pairs);
                    fw_flag = 0;
                    bw_flag = 0;
                }
                break;
            }

 //       t_loop :> end_time;
 //       printf("%i kHz\n", 250000/(end_time - start_time));
    }

}

//TODO rename to validate_commutation_param
void init_commutation_param(commutation_par & commutation_params,
                            hall_par & hall_params,
                            int nominal_speed)
{
    commutation_params.angle_variance = (60 * 4096) / (hall_params.pole_pairs * 2 * 360);
    if (hall_params.pole_pairs < 4) {
        commutation_params.nominal_speed = nominal_speed * 4;
    } else if (hall_params.pole_pairs >= 4) {
        commutation_params.nominal_speed = nominal_speed;
    }
    commutation_params.hall_offset_clk =  COMMUTATION_OFFSET_CLK;
    commutation_params.hall_offset_cclk = COMMUTATION_OFFSET_CCLK;
    commutation_params.winding_type = WINDING_TYPE;
    commutation_params.qei_forward_offset = 0;
    commutation_params.qei_backward_offset = 0;
}
