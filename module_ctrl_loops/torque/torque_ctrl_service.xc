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
//#include <statemachine.h>
//#include <drive_modes.h>

//#include <qei_client.h>
#include <qei_service.h>
#include <sine_table_big.h>
#include <a4935.h>
#include <motorcontrol_service.h>

//#define ENABLE_xscope_torq //don't forget to set up the config.xscope file
//#define debug_print

void init_buffer(int buffer[], int length)
{
    int i;
    for (i = 0; i < length; i++) {
        buffer[i] = 0;
    }
    return;
}

/*
void init_torque_control_param(ControlConfig &torque_ctrl_params)
{
    torque_ctrl_params.Kp_n = TORQUE_Kp_NUMERATOR;
    torque_ctrl_params.Kp_d = TORQUE_Kp_DENOMINATOR;
    torque_ctrl_params.Ki_n = TORQUE_Ki_NUMERATOR;
    torque_ctrl_params.Ki_d = TORQUE_Ki_DENOMINATOR;
    torque_ctrl_params.Kd_n = TORQUE_Kd_NUMERATOR;
    torque_ctrl_params.Kd_d = TORQUE_Kd_DENOMINATOR;
    torque_ctrl_params.Loop_time = 1 * MSEC_STD; // units - for CORE 2/1/0 only default

    torque_ctrl_params.Control_limit = BLDC_PWM_CONTROL_LIMIT; // PWM resolution

    if(torque_ctrl_params.Ki_n != 0) {
        // auto calculated using control_limit
        torque_ctrl_params.Integral_limit = (torque_ctrl_params.Control_limit * torque_ctrl_params.Ki_d) / torque_ctrl_params.Ki_n;
    } else {
        torque_ctrl_params.Integral_limit = 0;
    }

    return;
}
*/
int init_torque_control(interface TorqueControlInterface client i_torque_control)
{
    int ctrl_state = INIT_BUSY;

    while (1) {
        ctrl_state = i_torque_control.check_torque_ctrl_state();
        if (ctrl_state == INIT_BUSY) {
            i_torque_control.enable_torque_ctrl();
        }

        if (ctrl_state == INIT) {
#ifdef debug_print
            printstrln("torque control intialized");
#endif
            break;
        }
    }
    return ctrl_state;
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

void set_torque_cst(CyclicSyncTorqueConfig &cst_params, int target_torque, int torque_offset, interface TorqueControlInterface client i_torque_control)
{
    i_torque_control.set_torque( torque_limit( (target_torque + torque_offset) * cst_params.polarity,
                              cst_params.max_torque));
}

void current_filter(interface ADCInterface client adc_if, chanend c_current, chanend c_speed)
{
#define FILTER_LENGTH_ADC 80
    int phase_a_raw = 0;
    int phase_b_raw = 0;
    int actual_speed = 0;
    int command;
    int buffer_phase_a[FILTER_LENGTH_ADC] = {0};
    int buffer_phase_b[FILTER_LENGTH_ADC] = {0};
    timer ts;
    timer tc;
    unsigned int time;
    unsigned int time1;
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
   // int adc_calib_start = 0;
   // calib_data I_calib;
/*
    while (1) {
        select {
        case c_current :> command:
            //printstrln("start adc calibration");
            adc_calib_start = 1;
            break;
        }

        if (adc_calib_start == 1)
            break;
    }

    c_current <: 1; // adc calib done */
    //init_buffer(buffer_phase_a, FILTER_LENGTH_ADC);
    //init_buffer(buffer_phase_b, FILTER_LENGTH_ADC);
    ts :> time;
    tc :> time1;

    while (1) {
#pragma ordered
        select {
        case ts when timerafter(time+5556) :> time: // .05 ms
        { phase_a_raw, phase_b_raw } = adc_if.get_currents();//get_adc_calibrated_current_ad7949(c_adc, I_calib);
            //xscope_probe_data(0, phase_a_raw);
            buffer_phase_a[buffer_index] = phase_a_raw;
            buffer_phase_b[buffer_index] = phase_b_raw;

            buffer_index = (buffer_index+1)%filter_length;
            phase_a_filtered = 0;
            phase_b_filtered = 0;
            j=0;
            for (i = 0; i < filter_length_variance; i++) {
                mod = (buffer_index - 1 - j) % filter_length;
                if (mod < 0)
                    mod = filter_length + mod;
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

                if (abs_speed <= 100) {
                    filter_length_variance = 50;
                } else if (abs_speed > 100 && abs_speed <= 800) {
                    filter_length_variance = 20;
                } else if (abs_speed >= 800) {
                    filter_length_variance = 3;
                }
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




void torque_ctrl_loop(ControlConfig &torque_ctrl_params, HallConfig &hall_config, QEIConfig &qei_params,
                        chanend c_current, chanend c_speed,
                        interface MotorcontrolInterface client commutation_interface,
                        interface HallInterface client i_hall,
                        interface QEIInterface client i_qei,
                        interface TorqueControlInterface server i_torque_control[3])
{
#define FILTER_LENGTH_TORQUE 80
    int actual_speed = 0;
    int command;

    timer tc;
    unsigned int time1;
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

  //  int i1 = 0;
  //  int j1 = 0;
  //  int mod1 = 0;

    int iq_filtered = 0;
    int id_filtered = 0;
    int buffer_index = 0;
    int filter_length_variance = filter_length / hall_config.pole_pairs;


    int actual_torque = 0;
    int target_torque = 0;
    int absolute_torque = 0;

    int error_torque = 0;
    int error_torque_integral = 0;
    int error_torque_derivative = 0;
    int error_torque_previous = 0;
    int torque_control_output = 0;

    int init_state = INIT_BUSY;

    int compute_flag = 0;
    int qei_counts_per_hall;
   // qei_velocity_par qei_velocity_params;
    int start_flag = 0;
    int offset_fw_flag = 0;
    int offset_bw_flag = 0;
    int fet_state = 0;
    int activate = 0;

    printstr("*************************************\n    TORQUE CONTROLLER STARTING\n*************************************\n");

    //init_qei_velocity_params(qei_velocity_params);

    filter_length_variance = filter_length/hall_config.pole_pairs;
    if (filter_length_variance < 10) {
        filter_length_variance = 10;
    }

    if (torque_ctrl_params.sensor_used == QEI)
        qei_counts_per_hall= qei_params.real_counts/ hall_config.pole_pairs;

    //init_buffer(buffer_Id, filter_length);
    //init_buffer(buffer_Iq, filter_length);


    tc :> time1;
    time1 += MSEC_STD - 100;

    while(1) {
#pragma ordered
        select {

        case tc when timerafter(time1) :> void:
            time1 += MSEC_STD - 100;
            if (compute_flag == 1) {
                if (torque_ctrl_params.sensor_used == HALL) {
                    angle = (i_hall.get_hall_position() >> 2) & 0x3ff; //  << 10 ) >> 12 //get_hall_position(c_hall)
                    //xscope_probe_data(0, angle);
                    actual_speed = i_hall.get_hall_velocity();//get_hall_velocity(c_hall);
                } else if (torque_ctrl_params.sensor_used == QEI) {
                    { angle, offset_fw_flag, offset_bw_flag } = i_qei.get_qei_sync_position();
                    angle = ((angle <<10)/qei_counts_per_hall ) & 0x3ff;
                    //{qei_count_velocity, qei_direction_velocity} = get_qei_position_absolute(c_qei);
                    actual_speed = i_qei.get_qei_velocity(); //calculate_qei_velocity(qei_count_velocity,qei_params, qei_velocity_params);
                    //get_qei_velocity(c_qei, qei_params, qei_velocity_params);
                }

                c_current <: 2;
                c_current <: actual_speed;
                c_current :> phase_a_filtered;
                c_current :> phase_b_filtered;

                phase_a = 0 - phase_a_filtered;
                phase_b = 0 - phase_b_filtered;

#ifdef ENABLE_xscope_torq
                xscope_int(PHASE_A_FILTERED, phase_a_filtered);
#endif
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
                actual_torque = abs(Iq);//round( sqrt( iq_filtered * iq_filtered + id_filtered * id_filtered ) );//
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

                error_torque = absolute_torque - actual_torque; //350
                error_torque_integral = error_torque_integral + error_torque;
                error_torque_derivative = error_torque - error_torque_previous;

                if (error_torque_integral > torque_ctrl_params.Integral_limit) {
                    error_torque_integral = torque_ctrl_params.Integral_limit;
                } else if (error_torque_integral < 0-torque_ctrl_params.Integral_limit) {
                    error_torque_integral = 0 - torque_ctrl_params.Integral_limit;
                }


                torque_control_output = ( (torque_ctrl_params.Kp_n * error_torque)/torque_ctrl_params.Kp_d +
                                          (torque_ctrl_params.Ki_n * error_torque_integral)/torque_ctrl_params.Ki_d +
                                          (torque_ctrl_params.Kd_n  * error_torque_derivative)/torque_ctrl_params.Kd_d );

                error_torque_previous = error_torque;

                if (target_torque >= 0) {
                    if (torque_control_output >= torque_ctrl_params.Control_limit) {
                        torque_control_output = torque_ctrl_params.Control_limit;
                    } else if (torque_control_output < 0) {
                        torque_control_output = 0;
                    }
                } else {
                    torque_control_output = 0 - torque_control_output;
                    if (torque_control_output > 0) {
                        torque_control_output = 0;
                    }

                    if (torque_control_output <= -torque_ctrl_params.Control_limit) {
                        torque_control_output = 0 - torque_ctrl_params.Control_limit;
                    }
                }
                commutation_interface.setVoltage(torque_control_output);
                //set_commutation_sinusoidal(c_commutation, torque_control_output);
            }
            break;

        case i_torque_control[int i].set_torque(int in_torque):

            target_torque = in_torque;
#ifdef ENABLE_xscope_torq
                xscope_int(TARGET_TORQUE, target_torque);
#endif
            break;

        case i_torque_control[int i].get_set_torque() -> int out_set_torque:

                out_set_torque = target_torque;
                break;

        case i_torque_control[int i].get_torque() -> int out_torque:

                if (torque_control_output >= 0) {
                    out_torque = actual_torque;
                } else {
                    out_torque = 0-actual_torque;
                }
                out_torque *= hall_config.sensor_polarity; //it seems like the polarity is needed here.

                break;

        case i_torque_control[int i].check_busy() -> int out_state:

                out_state = activate;
                break;

        case i_torque_control[int i].set_torque_ctrl_param(ControlConfig in_params):

            torque_ctrl_params.Kp_n = in_params.Kp_n;
            torque_ctrl_params.Kp_d = in_params.Kp_d;
            torque_ctrl_params.Ki_n = in_params.Ki_n;
            torque_ctrl_params.Ki_d = in_params.Ki_d;
            torque_ctrl_params.Kd_n = in_params.Kd_n;
            torque_ctrl_params.Kd_d = in_params.Kd_d;
            torque_ctrl_params.Integral_limit = in_params.Integral_limit;

                break;

        case i_torque_control[int i].set_torque_ctrl_hall_param(HallConfig in_config):

            hall_config.pole_pairs = in_config.pole_pairs;
            hall_config.max_ticks = in_config.max_ticks;
            hall_config.max_ticks_per_turn = in_config.max_ticks_per_turn;

            filter_length_variance =  filter_length/hall_config.pole_pairs;
            if (filter_length_variance < 10) {
                filter_length_variance = 10;
            }

                break;

        case i_torque_control[int i].set_torque_ctrl_qei_param(QEIConfig in_params):

           qei_params.index = in_params.index;
           qei_params.real_counts = in_params.real_counts;
           qei_params.max_ticks_per_turn = in_params.max_ticks_per_turn;
           qei_params.poles = in_params.poles;
           qei_params.max_ticks = in_params.max_ticks;

           qei_counts_per_hall = qei_params.real_counts/ qei_params.poles;
           filter_length_variance =  filter_length/qei_params.poles;
           if (filter_length_variance < 10) {
               filter_length_variance = 10;
           }

                break;

        case i_torque_control[int i].set_torque_sensor(int in_sensor):

            torque_ctrl_params.sensor_used = in_sensor;

            if (torque_ctrl_params.sensor_used == HALL) {
                filter_length_variance =  filter_length/hall_config.pole_pairs;
                if (filter_length_variance < 10)
                    filter_length_variance = 10;
                target_torque = actual_torque;
            } else if (torque_ctrl_params.sensor_used == QEI) {
                qei_counts_per_hall = qei_params.real_counts/ qei_params.poles;
                filter_length_variance =  filter_length/qei_params.poles;
                if (filter_length_variance < 10)
                    filter_length_variance = 10;
                target_torque = actual_torque;
            }
            if (!compute_flag)
            {
               // enable_adc(c_current);
                compute_flag = 1;
            }
            break;

        case i_torque_control[int i].enable_torque_ctrl():
                activate = SET;
                  init_state = commutation_interface.checkBusy(); //__check_commutation_init(c_commutation);
                  if (init_state == INIT) {
#ifdef debug_print
                      printstrln("commutation intialized");
#endif
                      fet_state = commutation_interface.getFetsState(); //check_fet_state(c_commutation);
                      if (fet_state == 1) {
                          commutation_interface.enableFets(); //enable_motor(c_commutation);
                          delay_milliseconds(2);
                         // wait_ms(2, 1, tc);
                      }

                      if (compute_flag == 0) {
                          init_state = INIT_BUSY;
                         // c_current <: 1;
                      }
                  }
#ifdef debug_print
              printstrln("torque control activated");
#endif
              break;

        case c_current :> command:
            //printstrln("adc calibrated");
            start_flag = 1;
            break;

        case i_torque_control[int i].shutdown_torque_ctrl():

               activate = UNSET;
               error_torque = 0;
               error_torque_integral = 0;
               error_torque_derivative = 0;
               error_torque_previous = 0;
               torque_control_output = 0;
               commutation_interface.setVoltage(0);//set_commutation_sinusoidal(c_commutation, 0);
               commutation_interface.disableFets(); //disable_motor(c_commutation);
               delay_milliseconds(30);
               //wait_ms(30, 1, tc);
               break;

        case i_torque_control[int i].check_torque_ctrl_state() -> int out_state:

                out_state = activate;

                 break;
        }
    }
}

/* TODO: do we really need 2 threads for this? */
void torque_control_service(ControlConfig &torque_ctrl_params,
                    interface ADCInterface client adc_if,
                    interface MotorcontrolInterface client commutation_interface,
                    interface HallInterface client i_hall,
                    interface QEIInterface client ?i_qei,
                    interface TorqueControlInterface server i_torque_control[3])
{
    chan c_current, c_speed;
    HallConfig hall_config = i_hall.getHallConfig();

    QEIConfig qei_config;
    if(torque_ctrl_params.sensor_used == QEI && !isnull(i_qei)){
        qei_config = i_qei.getQEIConfig();
    }


    par {
        current_filter(adc_if, c_current, c_speed);
        torque_ctrl_loop(torque_ctrl_params, hall_config, qei_config, c_current,
                c_speed, commutation_interface, i_hall, i_qei, i_torque_control);

    }
}

void enable_adc(chanend c_current)
{
    int command, enabled = 0;
    c_current <: 1;
    while (!enabled)
    {
        select {
           case c_current :> command: //enable adc
                enabled = 1;
                break;
          }
    }
}
