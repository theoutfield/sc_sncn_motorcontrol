/**
 * @file  torque_ctrl_server.xc
 * @brief Torque Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <torque_ctrl_service.h>
#include <math.h>
#include <stdlib.h>
#include <refclk.h>
#include <xscope.h>
#include <print.h>

#include <qei_service.h>
#include <sine_table_big.h>
#include <a4935.h>
#include <motorcontrol_service.h>
#include <filter_blocks.h>

#define CURRENT_CONTROL

#include <string.h>

void init_buffer(int buffer[], int length)
{
    memset(buffer, 0, length);
}

void init_torque_control(interface TorqueControlInterface client i_torque_control)
{
    int ctrl_state;

    while (1) {
        ctrl_state = i_torque_control.check_busy();
        if (ctrl_state == INIT_BUSY) {
            i_torque_control.enable_torque_ctrl();
        }

        if (ctrl_state == INIT) {
#ifdef debug_print
            printstrln("torque_ctrl_service: torque control initialized");
#endif
            break;
        }
    }
}

int torque_limit(int torque, int max_torque_limit)
{
    if (torque > max_torque_limit) { //adc range // (5 * DC900_RESOLUTION)/2
        torque = max_torque_limit;
    } else if (torque < -max_torque_limit) {
        torque = -max_torque_limit;
    }
    return torque;
}

void current_filter(interface ADCInterface client adc_if, chanend c_current)
{
    int phase_a_raw;
    int phase_b_raw;
    int actual_speed = 0;
    int command;
    int buffer_phase_a[FILTER_LENGTH_ADC] = {0};
    int buffer_phase_b[FILTER_LENGTH_ADC] = {0};

    timer t;
    unsigned int time;

    int buffer_index = 0;
    int phase_a_filtered = 0;
    int phase_b_filtered = 0;
    int i = 0;
    int j = 0;
    int filter_length = FILTER_LENGTH_ADC;
    int filter_length_variance = 3;
    int mod = 0;
    int abs_speed = 0;
    int filter_count = 0;

    t :> time;

    while (1) {
#pragma ordered
        select {
            case t when timerafter(time + 5556) :> time: // .05 ms
                { phase_a_raw, phase_b_raw } = adc_if.get_currents();
                //xscope_probe_data(0, phase_a_raw);
                buffer_phase_a[buffer_index] = phase_a_raw;
                buffer_phase_b[buffer_index] = phase_b_raw;

                buffer_index = (buffer_index + 1) % filter_length;
                phase_a_filtered = 0;
                phase_b_filtered = 0;
                j=0;
                for (i = 0; i < filter_length_variance; i++) {
                    mod = (buffer_index - 1 - j) % filter_length;
                    if (mod < 0) {
                        mod = filter_length + mod;
                    }
                    phase_a_filtered += buffer_phase_a[mod];
                    phase_b_filtered += buffer_phase_b[mod];
                    j++;
                }
                phase_a_filtered /= filter_length_variance;
                phase_b_filtered /= filter_length_variance;
                /*xscope_probe_data(0, phase_a_filtered);
                  xscope_probe_data(1, phase_b_filtered);
                  xscope_probe_data(2, phase_a_raw);
                  xscope_probe_data(3, phase_b_raw);*/
                filter_count++;
                if (filter_count == 10) {
                    filter_count = 0;

                    //xscope_probe_data(0, actual_speed);
                    abs_speed = abs(actual_speed);

                    //ToDo: this needs to be adapted depending on the IFM device type, use fixed length in meanwhile
/*
                    if (abs_speed <= 100) {
                        filter_length_variance = 50;
                    } else if (abs_speed > 100 && abs_speed <= 800) {
                        filter_length_variance = 20;
                    } else if (abs_speed >= 800) {
                        filter_length_variance = 3;
                    }
*/
                    filter_length_variance = 50;
                }
                break;

            case c_current :> command:
                if (command == 2) {
                    c_current :> actual_speed;
                    c_current <: phase_a_filtered;
                    c_current <: phase_b_filtered;
                } else if (command == 11) {
                    init_buffer(buffer_phase_a, FILTER_LENGTH_ADC);
                    init_buffer(buffer_phase_b, FILTER_LENGTH_ADC);
                    actual_speed = 0;
                    buffer_index = 0;
                    i = 0;
                    j = 0;
                    filter_length_variance = 3;
                    mod = 0;
                    filter_count = 0;
                }
                break;
        }
    }
}

void torque_ctrl_loop(ControlConfig &torque_control_config,
                      chanend c_current,
                      interface HallInterface client ?i_hall,
                      interface QEIInterface client ?i_qei,
                      interface BISSInterface client ?i_biss,
                      interface MotorcontrolInterface client i_motorcontrol,
                      interface TorqueControlInterface server i_torque_control[3])
{
    int actual_speed = 0;
    int command;

    timer tc;
    unsigned int time;

    timer tc2;
    unsigned int start_time1, end_time1, start_time2, end_time2;
    int elapsed_time1, elapsed_time2;

    int phase_a_filtered = 0;
    int phase_b_filtered = 0;

    // Torque control variables
    int angle = 0;
    int sin = 0;
    int cos = 0;
    int alpha = 0;
    int beta = 0;
    int Id = 0;
    int Iq = 0;
    int phase_a = 0;
    int phase_b = 0;
    int filter_length = FILTER_LENGTH_TORQUE;
    int buffer_Id[FILTER_LENGTH_TORQUE] = {0};
    int buffer_Iq[FILTER_LENGTH_TORQUE] = {0};
    int current_control_buffer[FILTER_LENGTH_TORQUE];
    int index = 0;

    int id_filtered = 0;
    int iq_filtered = 0;
    int buffer_index = 0;
    int filter_length_variance;

    int actual_torque = 0;
    int actual_torque_filtered = 0;
    int target_torque = 0;
    int absolute_torque = 0;

    int error_torque = 0;
    int error_torque_integral = 0;
    int error_torque_derivative = 0;
    int error_torque_previous = 0;
    int torque_control_output = 0;

    int torque_control_output_limit = 0;
    int error_torque_integral_limit = 0;

    int compute_flag = 0;
    int qei_counts_per_hall;

    int start_flag = 0;
    int offset_fw_flag = 0;
    int offset_bw_flag = 0;
    int activate = 0;

    int max_ph_a = 0, max_ph_a_actual = 0;
    int max_ph_b = 0, max_ph_b_actual = 0;
    int min_ph_a = 0, min_ph_a_actual = 0;
    int min_ph_b = 0, min_ph_b_actual = 0;
    int phase_a_prev = 0;
    int phase_b_prev = 0;
    int reset1 = 0, reset2 = 0, reset3 = 0, reset4 = 0;

    MotorcontrolConfig motorcontrol_config;

    int config_update_flag = 1;

    printstr(">>   SOMANET TORQUE CONTROL SERVICE STARTING...\n");

    tc :> time;

    while(1) {
#pragma ordered
        select {
            case tc when timerafter(time + USEC_STD * torque_control_config.control_loop_period) :> time:

                if (config_update_flag) {
                    HallConfig hall_config;
                    QEIConfig qei_config;
                    BISSConfig biss_config;
                    motorcontrol_config = i_motorcontrol.get_config();

                    //Limits
                    if (motorcontrol_config.motor_type == BLDC_MOTOR) {
                        torque_control_output_limit = BLDC_PWM_CONTROL_LIMIT;
                    } else if (motorcontrol_config.motor_type == BDC_MOTOR) {
                        torque_control_output_limit = BDC_PWM_CONTROL_LIMIT;
                    }

                    // The Hall configuration for BLDC motor must always be loaded because of qei_counts_per_hall computation
                    if (isnull(i_hall)) {
                        if(motorcontrol_config.motor_type == BLDC_MOTOR){
                            printstrln("torque_ctrl_service: ERROR: Interface for Hall Service not provided");
                        }
                    } else {
                        hall_config = i_hall.get_hall_config();
                    }

                    if (torque_control_config.feedback_sensor == QEI_SENSOR) {
                        if (isnull(i_qei)) {
                            printstrln("torque_ctrl_service: ERROR: Interface for QEI Service not provided");
                        } else {
                            qei_config = i_qei.get_qei_config();
                        }
                    }

                    if (torque_control_config.feedback_sensor == BISS_SENSOR) {
                        if (isnull(i_biss)) {
                            printstrln("torque_ctrl_service: ERROR: Interface for BISS Service not provided");
                        } else {
                            biss_config = i_biss.get_biss_config();
                        }
                    }

                    if (torque_control_config.feedback_sensor != HALL_SENSOR
                           && torque_control_config.feedback_sensor != QEI_SENSOR
                           && torque_control_config.feedback_sensor != BISS_SENSOR) {
                        torque_control_config.feedback_sensor = motorcontrol_config.commutation_sensor;
                    }

                    if (torque_control_config.feedback_sensor == QEI_SENSOR && motorcontrol_config.motor_type == BLDC_MOTOR) {
                        qei_counts_per_hall = (qei_config.ticks_resolution * 4) / hall_config.pole_pairs;
                    }

                    if(torque_control_config.Ki_n != 0) {
                        error_torque_integral_limit = (torque_control_output_limit * PID_DENOMINATOR) / torque_control_config.Ki_n;
                    }

                    if (torque_control_config.control_loop_period < MIN_TORQUE_CONTROL_LOOP_PERIOD) {
                        torque_control_config.control_loop_period = MIN_TORQUE_CONTROL_LOOP_PERIOD;
                        printstrln("torque_ctrl_service: ERROR: Loop period to small, set to 100 us");
                    }

                    if(torque_control_config.feedback_sensor == BISS_SENSOR) {
                        filter_length_variance = filter_length / biss_config.pole_pairs;
                    } else {
                        filter_length_variance = filter_length / hall_config.pole_pairs;
                    }

                    if (filter_length_variance < 10) {
                        filter_length_variance = 10;
                    }

                    config_update_flag = 0;
                }

                if (compute_flag == 1) {
                    if (torque_control_config.feedback_sensor == HALL_SENSOR && !isnull(i_hall)) {
                        angle = (i_hall.get_hall_position() >> 2) & 0x3ff; //  << 10 ) >> 12 /
                        actual_speed = i_hall.get_hall_velocity();
                    } else if (torque_control_config.feedback_sensor == QEI_SENSOR && !isnull(i_qei)) {
                        { angle, offset_fw_flag, offset_bw_flag } = i_qei.get_qei_sync_position();
                        if(motorcontrol_config.motor_type == BLDC_MOTOR){//angle is irrelevant for BDC motor
                            angle = ((angle << 10) / qei_counts_per_hall ) & 0x3ff;
                        }
                        actual_speed = i_qei.get_qei_velocity();
                    } else if (torque_control_config.feedback_sensor == BISS_SENSOR && !isnull(i_biss)) {
                        if(motorcontrol_config.motor_type == BLDC_MOTOR){//angle is irrelevant for BDC motor
                            angle = i_biss.get_biss_angle() >> 2; //  << 10 ) >> 12 /
                        }
                        actual_speed = i_biss.get_biss_velocity();
                    }

                    c_current <: 2;
                    c_current <: actual_speed;
                    c_current :> phase_a_filtered;
                    c_current :> phase_b_filtered;

                    phase_a = - phase_a_filtered;
                    phase_b = - phase_b_filtered;

#ifdef ENABLE_xscope_torq
                    xscope_int(PHASE_A_FILTERED, phase_a_filtered);
#endif
                    if (motorcontrol_config.motor_type == BDC_MOTOR) {
                        actual_torque = abs(phase_a);
                    } else if (motorcontrol_config.motor_type == BLDC_MOTOR) {
#ifndef CURRENT_CONTROL
                        //xscope_probe_data(1, phase_b_filtered);
                        alpha = phase_a;
                        beta = (phase_a + 2*phase_b);  // beta = (a1 + 2*a2)/1.732 0.57736 --> invers from 1.732
                        beta *= 37838;
                        beta /= 65536;
                        beta = -beta;

                        /* Park transform */
                        sin = sine_table_expanded(angle);
                        cos = sine_table_expanded((256 - angle) & 0x3ff);

                        Id = ( alpha * cos + beta * sin ) / 16384;
                        Iq = ( beta * cos  - alpha * sin ) / 16384;

                        buffer_Id[buffer_index] = Id;
                        buffer_Iq[buffer_index] = Iq;
                        buffer_index = (buffer_index + 1) % filter_length;

                        id_filtered = Id;
                        iq_filtered = Iq;
        /*
                        j1 = 0;
                        for (i1 = 0; i1 < filter_length_variance; i1++) {
                            mod1 = (buffer_index - 1 - j1) % filter_length;
                            if (mod1 < 0)
                                mod1 = filter_length + mod1;
                            id_filtered += buffer_Id[mod1];
                            iq_filtered += buffer_Iq[mod1];
                            j1++;
                        }
                        id_filtered /= filter_length_variance;
                        iq_filtered /= filter_length_variance;
        */
                        actual_torque = abs(Iq); // round( sqrt( iq_filtered * iq_filtered + id_filtered * id_filtered ) );//
#else
                        //Find current magnitudes
                        if (phase_a > max_ph_a) {
                            max_ph_a = phase_a;
                        }
                        if (phase_b > max_ph_b) {
                            max_ph_b = phase_b;
                        }
                        if (phase_a < min_ph_a) {
                            min_ph_a = phase_a;
                        }
                        if (phase_b < min_ph_b) {
                            min_ph_b = phase_b;
                        }

                        //detect zero crossings for phase_a and phase_b
                        if ((phase_a < 0) && (phase_a_prev > 0)) {
                            tc2 :> start_time1;
                            if (!reset1) {
                                //xscope_int(PERIOD, 1000);
                                max_ph_a_actual = max_ph_a;
                                max_ph_a = 0;
                                reset1 = 1;
                            }
                        } else if ((phase_a > 0) && (phase_a_prev < 0)) {
                            tc2 :> end_time1;
                            if (!reset2) {
                                //xscope_int(PERIOD, 2000);
                                min_ph_a_actual = min_ph_a;
                                min_ph_a = 0;
                                reset2 = 1;
                            }
                        } else {
                            //xscope_int(PERIOD, 0);
                        }

                        if ((phase_b < 0) && (phase_b_prev > 0)) {
                            tc2 :> start_time2;
                            if (!reset3) {
                                max_ph_b_actual = max_ph_b;
                                max_ph_b = 0;
                                reset3 = 1;
                            }
                        }
                        else if ((phase_b > 0) && (phase_b_prev < 0)) {
                            tc2 :> end_time2;
                            if (!reset4) {
                                min_ph_b_actual = min_ph_b;
                                min_ph_b = 0;
                                reset4 = 1;
                            }
                        }

                        //calculate time between zero crosings of phase_a and phase_b to reject false positives
                        elapsed_time1 = end_time1 - start_time1;
                        elapsed_time2 = end_time2 - start_time2;

                        //for low velocities phase currents are becoming linear if there is a load
                        if (abs(actual_speed) < 100) {
                            max_ph_a_actual = max_ph_a;
                            min_ph_a_actual = min_ph_a;
                            max_ph_b_actual = max_ph_b;
                            min_ph_b_actual = min_ph_b;
                            reset1 = 0;
                            reset2 = 0;
                            reset3 = 0;
                            reset4 = 0;
                            max_ph_a = 0;
                            min_ph_a = 0;
                            max_ph_b = 0;
                            min_ph_b = 0;
                        } else if (elapsed_time1 > 10 * MSEC_STD*torque_control_config.control_loop_period / (abs(actual_speed) + 1)) {
                            reset1 = 0;
                            reset2 = 0;
                            elapsed_time1 = 0;
                        } else if(elapsed_time2 > 10 * MSEC_STD*torque_control_config.control_loop_period / (abs(actual_speed) + 1)) {
                            reset3 = 0;
                            reset4 = 0;
                            elapsed_time2 = 0;
                        }

                        phase_a_prev = phase_a;
                        phase_b_prev = phase_b;

                        int temp = ((max_ph_a_actual - min_ph_a_actual) / 2 + (max_ph_b_actual - min_ph_b_actual) / 2) / 2;//calculate mean of two phase currents amplitudes
                        actual_torque_filtered = filter(current_control_buffer, index, FILTER_LENGTH_TORQUE, temp);
                        actual_torque = actual_torque_filtered;
#endif
                    }
                }

                if (activate == 1) {
#ifdef ENABLE_xscope_torq
                    xscope_int(ACTUAL_TORQUE, actual_torque);
#endif
                    compute_flag = 1;

                    absolute_torque = target_torque;
                    if (target_torque < 0) {
                        absolute_torque = 0 - target_torque;
                    }

                    error_torque = absolute_torque - actual_torque; // 350
                    error_torque_integral = error_torque_integral + error_torque;
                    error_torque_derivative = error_torque - error_torque_previous;

                    if (error_torque_integral > error_torque_integral_limit) {
                        error_torque_integral = error_torque_integral_limit;
                    } else if (error_torque_integral < -error_torque_integral_limit) {
                        error_torque_integral = -error_torque_integral_limit;
                    }

                    torque_control_output = (torque_control_config.Kp_n * error_torque) +
                                            (torque_control_config.Ki_n * error_torque_integral) +
                                            (torque_control_config.Kd_n * error_torque_derivative);

                    torque_control_output /= PID_DENOMINATOR;

                    error_torque_previous = error_torque;

                    if (target_torque >= 0) {
                        if (torque_control_output >= torque_control_output_limit) {
                            torque_control_output = torque_control_output_limit;
                        } else if (torque_control_output < 0) {
                            torque_control_output = 0;
                        }
                    } else {
                        torque_control_output = 0 - torque_control_output;
                        if (torque_control_output > 0) {
                            torque_control_output = 0;
                        }

                        if (torque_control_output < -torque_control_output_limit) {
                            torque_control_output = -torque_control_output_limit;
                        }
                    }

                    i_motorcontrol.set_voltage(torque_control_output);
                }
                break;

            case !isnull(i_hall) => i_hall.notification():

                switch (i_hall.get_notification()) {
                    case MOTCTRL_NTF_CONFIG_CHANGED:
                        config_update_flag = 1;
                        break;
                    default:
                        break;
                }
                break;

            case !isnull(i_qei) => i_qei.notification():

                switch (i_qei.get_notification()) {
                    case MOTCTRL_NTF_CONFIG_CHANGED:
                        config_update_flag = 1;
                        break;
                    default:
                        break;
                }
                break;

            case !isnull(i_biss) => i_biss.notification():

                switch (i_biss.get_notification()) {
                    case MOTCTRL_NTF_CONFIG_CHANGED:
                        config_update_flag = 1;
                        break;
                    default:
                        break;
                }
                break;

            case i_motorcontrol.notification():

                switch (i_motorcontrol.get_notification()) {
                    case MOTCTRL_NTF_CONFIG_CHANGED:
                        config_update_flag = 1;
                        break;
                    default:
                        break;
                }
                break;

            case i_torque_control[int i].set_torque(int in_torque):

                target_torque = in_torque;
#ifdef ENABLE_xscope_torq
                xscope_int(TARGET_TORQUE, target_torque);
#endif
                break;

            case i_torque_control[int i].get_torque() -> int out_torque:

                if (torque_control_output >= 0) {
                    out_torque = actual_torque;
                } else {
                    out_torque = -actual_torque;
                }
                //out_torque *= hall_config.sensor_polarity; //it seems like the polarity is needed here.

                break;

            case i_torque_control[int i].get_target_torque() -> int out_target_torque:

                out_target_torque = target_torque;
                break;

            case i_torque_control[int i].set_torque_control_config(ControlConfig in_params):

                torque_control_config = in_params;
                config_update_flag = 1;
                break;

            case i_torque_control[int i].get_torque_control_config() -> ControlConfig out_config:

                out_config = torque_control_config;
                break;

            case i_torque_control[int i].set_torque_sensor(int in_sensor):

                torque_control_config.feedback_sensor = in_sensor;
                target_torque = actual_torque;
                if (!compute_flag) {
                    compute_flag = 1;
                }
                config_update_flag = 1;
                break;

            case i_torque_control[int i].check_busy() -> int out_state:

                out_state = activate;
                break;

            case i_torque_control[int i].enable_torque_ctrl():

                activate = 1;
                int init_state = i_motorcontrol.check_busy(); //__check_commutation_init(c_commutation);
                if (init_state == INIT) {
#ifdef debug_print
                    printstrln("torque_ctrl_service: commutation initialized");
#endif
                    if (i_motorcontrol.get_fets_state() == 0) { //check_fet_state(c_commutation);
                        i_motorcontrol.set_fets_state(1); //enable_motor(c_commutation);
                        delay_milliseconds(2);
                        // wait_ms(2, 1, tc);
                    }

                    if (compute_flag == 0) {
                        init_state = INIT_BUSY;
                        // c_current <: 1;
                    }
                }
#ifdef debug_print
                printstrln("torque_ctrl_service: torque control activated");
#endif
                break;

            case c_current :> command:
                //printstrln("torque_ctrl_service: adc calibrated");
                start_flag = 1;
                break;

            case i_torque_control[int i].disable_torque_ctrl():

                activate = 0;
                error_torque = 0;
                error_torque_integral = 0;
                error_torque_derivative = 0;
                error_torque_previous = 0;
                torque_control_output = 0;
                i_motorcontrol.set_voltage(0);//set_commutation_sinusoidal(c_commutation, 0);
                i_motorcontrol.set_fets_state(0); //disable_motor(c_commutation);
                delay_milliseconds(30);
                //wait_ms(30, 1, tc);
                break;

        }
    }
}

void torque_control_service(ControlConfig &torque_control_config,
                            interface ADCInterface client adc_if,
                            interface HallInterface client ?i_hall,
                            interface QEIInterface client ?i_qei,
                            interface BISSInterface client ?i_biss,
                            interface MotorcontrolInterface client i_motorcontrol,
                            interface TorqueControlInterface server i_torque_control[3])
{
    chan c_current;

    par {
        current_filter(adc_if, c_current);
        torque_ctrl_loop(torque_control_config, c_current, i_hall, i_qei, i_biss, i_motorcontrol, i_torque_control);
    }
}
