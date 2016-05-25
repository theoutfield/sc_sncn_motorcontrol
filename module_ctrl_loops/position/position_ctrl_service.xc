/**
 * @file  position_ctrl_server.xc
 * @brief Position Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/
#include <xs1.h>
#include <xscope.h>
#include <print.h>
#include <stdlib.h>

#include <position_ctrl_service.h>
#include <a4935.h>
#include <mc_internal_constants.h>
#include <hall_service.h>
#include <qei_service.h>
#include <filters_lib.h>
#include <stdio.h>



void init_position_control(interface PositionControlInterface client i_position_control)
{
    int ctrl_state;

    while (1) {
        ctrl_state = i_position_control.check_busy();
        if (ctrl_state == INIT_BUSY) {
            i_position_control.enable_position_ctrl();
        }

        if (ctrl_state == INIT) {
#ifdef debug_print
            printstrln("position_ctrl_service: position control initialized");
#endif
            break;
        }
    }
}

int position_limit(int position, int max_position_limit, int min_position_limit)
{
    if (position > max_position_limit) {
        position = max_position_limit;
    } else if (position < min_position_limit) {
        position = min_position_limit;
    }
    return position;
}

void position_control_service(ControlConfig &position_control_config,
                              interface HallInterface client ?i_hall,
                              interface QEIInterface client ?i_qei,
                              interface BISSInterface client ?i_biss,
                              interface AMSInterface client ?i_ams,
                              interface MotorcontrolInterface client i_motorcontrol,
                              interface PositionControlInterface server i_position_control[3])
{
    int actual_position = 0;
    int target_position = 0;
    int error_position = 0;
    int error_position_D = 0;
    int error_position_I = 0;
    int previous_error = 0;
    int position_control_out = 0;

    int position_control_out_limit = 0;
    int error_position_I_limit = 0;

    /* for cascaded control */
    int adc_a, adc_b;
    int velocity = 0;
    //General Control Variables
//    int first_loop_counter = 0;
    unsigned T_k = 0, T_k_1n = 0;
    unsigned T_s_desired = 1000; //us
    unsigned T_s = T_s_desired;

    //Joint Torque Control
    int i1_torque_j_ref = 0;
    int f1_torque_j_lim = 50000;
    float f1_torque_j_sens_measured = 0;
    int i1_torque_j_sens_offset = 0;
    int i1_torque_j_sens_offset_accumulator = 0;
    float f1_torque_j_sens = 0;
    int i1_torque_j_sens = 0;
    int i1_torque_j_sens_1n = 0;
    float f1_torque_j_sens_1n = 0;
    float f1_torque_j_sens_2n = 0;
    SecondOrderLPfilterParam torque_sensor_SO_LP_filter_param;
    int torque_j_error = 0;
    int torque_j_derivative = 0;
    int torque_j_Kp = 0;
    int torque_j_Ki = 0;
    int torque_j_Kd = 0;
    int torque_j_ctrl_cmd = 0;
    int torque_j_error_integral = 0;
    int torque_j_error_integral_limit = 0;
    int torque_j_ctrl_cmd_lim = 200;

    // Position Control
    int position_sens = 0;
    float f1_velocity_sens = 0;
    float f1_velocity_sens_1n = 0;
    float f1_velocity_sens_2n = 0;
    float f1_velocity_sens_measured = 0;
    SecondOrderLPfilterParam velocity_sensor_SO_LP_filter_param;
    int i1_velocity_sens = 0;
    int position_error = 0;
    int position_error_integral = 0;
    int position_error_limited = 0;
    int position_Kp = 0;
    int position_Ki = 0;
    int position_Kd = 0;
    int position_ctrl_cmd = 0;
    int position_ctrl_cmd_lim = 0;
    int position_error_integral_limit = 0;

    int torque_ref=100;

    // A2
//    second_order_LP_filter_init(/*f_c=*//*18*/120, /*T_s=*/T_s_desired, torque_sensor_SO_LP_filter_param);
//    second_order_LP_filter_init(/*f_c=*//*80*/140, /*T_s=*/T_s_desired, velocity_sensor_SO_LP_filter_param);

    // A6
    second_order_LP_filter_init(/*f_c=*/18, /*T_s=*/T_s_desired, torque_sensor_SO_LP_filter_param);
    second_order_LP_filter_init(/*f_c=*/80, /*T_s=*/T_s_desired, velocity_sensor_SO_LP_filter_param);
    /* end cascaded */

    timer t;
    unsigned int ts;

    int activate = 0;

    int config_update_flag = 1;

    int i=0;
    int offset=0;

    MotorcontrolConfig motorcontrol_config;

    printstr(">>   SOMANET POSITION CONTROL SERVICE STARTING...\n");


    if (position_control_config.cascade_with_torque == 1) {
        i_motorcontrol.set_torque(0);
        delay_milliseconds(10);
        for (int i=0 ; i<2000; i++) {
            delay_milliseconds(1);
            i1_torque_j_sens_offset_accumulator += (i_motorcontrol.get_torque_actual());
        }
        i1_torque_j_sens_offset = i1_torque_j_sens_offset_accumulator / 2000;
        i_motorcontrol.set_voltage(0);
        printstrln(">>   POSITION CONTROL CASCADED WITH TORQUE");
    }

    t :> ts;
    while(1)
    {

        printf("\n\n\n\n\nsending offset_detection command ...\n");
        i_motorcontrol.set_offset_detection_enabled();
        delay_milliseconds(10000);


        offset=i_motorcontrol.set_calib(0);
        printf("detected offset is: %i\n", offset);
        delay_milliseconds(2000);


        printf("setting offset to (detected_offset+10) ...\n");
        i_motorcontrol.set_offset_value(offset+10);
        delay_milliseconds(2000);


        offset=i_motorcontrol.set_calib(0);
        printf("set offset is: %i\n", offset);
        delay_milliseconds(2000);


        printf("Enabling torque controller ...\n");
        i_motorcontrol.set_torque_control_enabled();
        delay_milliseconds(5000);


        for(int i=0;i<=10;i++)
        {
            torque_ref = 100;
            printf("sending positive torque command ...\n");
            i_motorcontrol.set_torque(torque_ref);
            delay_milliseconds(200);

            torque_ref = -100;
            printf("sending negative torque command ...\n");
            i_motorcontrol.set_torque(torque_ref);
            delay_milliseconds(200);
        }


        printf("sending zero torque command ...\n");
        torque_ref = 0;
        i_motorcontrol.set_torque(torque_ref);
        delay_milliseconds(5000);


        printf("Disabling torque controller ...\n");
        i_motorcontrol.set_torque_control_disabled();
        delay_milliseconds(5000);

        delay_milliseconds(20000);


    }
//
//    while(1) {
//#pragma ordered
//        select {
//            case t when timerafter(ts + USEC_STD * position_control_config.control_loop_period) :> ts:
//                if (config_update_flag) {
//                    motorcontrol_config = i_motorcontrol.get_config();
//
//                    //Limits
//                    if (motorcontrol_config.motor_type == BLDC_MOTOR) {
//                        if(motorcontrol_config.commutation_method == FOC){
//                            position_control_out_limit = 4096;//FOC control range [-4096:4096]
//                        } else {
//                            position_control_out_limit = BLDC_PWM_CONTROL_LIMIT;
//                        }
//                    } else if(motorcontrol_config.motor_type == BDC_MOTOR) {
//                        position_control_out_limit = BDC_PWM_CONTROL_LIMIT;
//                    }
//
//                    if (position_control_config.feedback_sensor != HALL_SENSOR
//                           && position_control_config.feedback_sensor != QEI_SENSOR
//                           && position_control_config.feedback_sensor != BISS_SENSOR
//                           && position_control_config.feedback_sensor != AMS_SENSOR) {
//                        position_control_config.feedback_sensor = motorcontrol_config.commutation_sensor;
//                    }
//
//                    if (position_control_config.Ki_n != 0) {
//                        error_position_I_limit = position_control_out_limit * PID_DENOMINATOR / position_control_config.Ki_n;
//                    }
//
//                    if (position_control_config.feedback_sensor == QEI_SENSOR && !isnull(i_qei)) {
//                        actual_position = i_qei.get_qei_position_absolute();
//                    } else {
//                        actual_position = i_motorcontrol.get_position_actual();
//                    }
//
//                    config_update_flag = 0;
//                }
//
//                if (activate == 1) {
//                    /* acquire actual position hall/qei/sensor */
//                    if (position_control_config.feedback_sensor == QEI_SENSOR) {
//                        if (!isnull(i_qei)) {
//                            actual_position = i_qei.get_qei_position_absolute();
//                        } else {
//                            printstrln("position_ctrl_service: ERROR: Encoder interface is not provided but requested");
//                            exit(-1);
//                        }
//                    } else {
//                        actual_position = i_motorcontrol.get_position_actual();
//                        velocity = i_motorcontrol.get_velocity_actual();
//                    }
//                    adc_b = i_motorcontrol.get_torque_actual();
//                    /*
//                     * Or any other sensor interfaced to the IFM Module
//                     * place client functions here to acquire position
//                     */
//
//                    /* cascaded control */
//                    if(position_control_config.cascade_with_torque == 1 && motorcontrol_config.commutation_method == FOC){
//                        t :> T_k;
//                        T_s = (T_k - T_k_1n) / USEC_STD;
//                        if ((T_k & 0x80000000) != (T_k_1n & 0x80000000))
//                            T_s += 0xffffffff;
//                        T_k_1n = T_k;
//
////                        { adc_a, adc_b } = i_adc.get_external_inputs();
//                        adc_a = 0;
//
////                        if (first_loop_counter > 5000) {
////
////                        } else if (first_loop_counter < 3000) {
////                            enable_motor_command = 0;
////                            T_s = T_s_desired;
////                            first_loop_counter++;
////                        } else if (first_loop_counter < 5000) {
////                            i1_torque_j_sens_offset_accumulator += (adc_b - adc_a);
////                            enable_motor_command = 0;
////                            T_s = T_s_desired;
////                            first_loop_counter++;
////                        } else if (first_loop_counter == 5000) {
////                            printstrln("cascaded");
////                            enable_motor_command = 1;
////                            i1_torque_j_sens_offset = i1_torque_j_sens_offset_accumulator / 2000;
////                            first_loop_counter++;
////                        }
//
//                        f1_torque_j_sens_measured = 10 * ((float) (adc_b - adc_a - i1_torque_j_sens_offset));
//
//                        second_order_LP_filter_update(&f1_torque_j_sens,
//                                &f1_torque_j_sens_1n,
//                                &f1_torque_j_sens_2n,
//                                &f1_torque_j_sens_measured, T_s, torque_sensor_SO_LP_filter_param);
//
//                        f1_velocity_sens_measured = ((float) velocity);
//                        second_order_LP_filter_update(&f1_velocity_sens,
//                                &f1_velocity_sens_1n,
//                                &f1_velocity_sens_2n,
//                                &f1_velocity_sens_measured, T_s, velocity_sensor_SO_LP_filter_param);
//                        i1_velocity_sens = ((int) f1_velocity_sens);
//
//                        i1_torque_j_sens = (int) f1_torque_j_sens;
//
//                        position_sens = (actual_position/1000);
//
//                        position_error = target_position/1000 - position_sens;
//                        if (position_error > 100)
//                            position_error = 100;
//                        else if (position_error < -100)
//                            position_error = -100;
//
//                        position_error_limited = position_error;
//                        if (position_error_limited > 2)
//                            position_error_limited = 2;
//                        else if (position_error_limited < -2)
//                            position_error_limited = -2;
//
//                        // A2
////                        position_Kp = 700;
////                        position_Ki = 69;
////                        position_Kd = 200;
//
//                        // A6
//                        position_Kp = 50;
//                        position_Ki = 1;
//                        position_Kd = 22;
//
//                        position_error_integral += position_error_limited;
//                        position_error_integral_limit = 4000 / (position_Ki+1);
//                        if (position_error_integral > position_error_integral_limit) {
//                            position_error_integral = position_error_integral_limit;
//                        } else if (position_error_integral < -position_error_integral_limit) {
//                            position_error_integral = -position_error_integral_limit;
//                        }
//
//                        position_ctrl_cmd = position_Kp * position_error + (position_Ki+1) * position_error_integral
//                                                        - position_Kd * i1_velocity_sens;
//
//                        //limit command value
//                        position_ctrl_cmd_lim = 50000;
//                        if (position_ctrl_cmd > position_ctrl_cmd_lim) {
//                            position_ctrl_cmd = position_ctrl_cmd_lim;
//                        }else if (position_ctrl_cmd < -position_ctrl_cmd_lim) {
//                            position_ctrl_cmd = -position_ctrl_cmd_lim;
//                        }
//
//
//                        i1_torque_j_ref = position_ctrl_cmd;
//
//
//                        //limit torque on the secondary side of the gearbox
//                        //            f1_torque_j_lim = 50000; //50000 is the maximum with which the motor torque has been tested
//                        if (i1_torque_j_ref > f1_torque_j_lim) {
//                            i1_torque_j_ref = f1_torque_j_lim;
//                        }else if (i1_torque_j_ref < -f1_torque_j_lim) {
//                            i1_torque_j_ref = -f1_torque_j_lim;
//                        }
//
//                        adc_b = i1_torque_j_sens;
//
//                        torque_j_error = i1_torque_j_ref - i1_torque_j_sens;
//                        torque_j_error_integral = torque_j_error_integral + torque_j_error;
//                        torque_j_error_integral_limit = 1;
//                        if (torque_j_error_integral > torque_j_error_integral_limit) {
//                            torque_j_error_integral = torque_j_error_integral_limit;
//                        } else if (torque_j_error_integral < -torque_j_error_integral_limit) {
//                            torque_j_error_integral = -torque_j_error_integral_limit;
//                        }
//                        torque_j_derivative = i1_torque_j_sens - i1_torque_j_sens_1n;
//
//                        torque_j_Kp = 500;
//                        torque_j_Ki = 0;
//                        torque_j_Kd = 600;
//
//                        torque_j_ctrl_cmd = torque_j_Kp * torque_j_error + torque_j_Ki * torque_j_error_integral - torque_j_Kd * torque_j_derivative;
//
//                        torque_j_ctrl_cmd /= 10000;
//
//                        //limit command value
//                        torque_j_ctrl_cmd_lim = 400; //400 is the maximum with which the motor torque has been tested
//                        if (torque_j_ctrl_cmd > torque_j_ctrl_cmd_lim) {
//                            torque_j_ctrl_cmd = torque_j_ctrl_cmd_lim;
//                        }else if (torque_j_ctrl_cmd < -torque_j_ctrl_cmd_lim) {
//                            torque_j_ctrl_cmd = -torque_j_ctrl_cmd_lim;
//                        }
//                        i_motorcontrol.set_torque(torque_j_ctrl_cmd);
//
//                        i1_torque_j_sens_1n = i1_torque_j_sens;
//
//
//                        second_order_LP_filter_shift_buffers(&f1_torque_j_sens,
//                                &f1_torque_j_sens_1n,
//                                &f1_torque_j_sens_2n);
//                        second_order_LP_filter_shift_buffers(&f1_velocity_sens,
//                                &f1_velocity_sens_1n,
//                                &f1_velocity_sens_2n);
//
//                        velocity = i1_velocity_sens;
//                    } else {
//                        /* PID Controller */
//
//                        error_position = (target_position - actual_position);
//                        error_position_I = error_position_I + error_position;
//                        error_position_D = error_position - previous_error;
//                        previous_error = error_position;
//
//                        if (error_position_I > error_position_I_limit) {
//                            error_position_I = error_position_I_limit;
//                        } else if (error_position_I < -error_position_I_limit) {
//                            error_position_I = - error_position_I_limit;
//                        }
//
//                        position_control_out = (position_control_config.Kp_n * error_position) +
//                                (position_control_config.Ki_n * error_position_I) +
//                                (position_control_config.Kd_n * error_position_D);
//
//                        position_control_out /= PID_DENOMINATOR;
//
//                        if (position_control_out > position_control_out_limit) {
//                            position_control_out = position_control_out_limit;
//                        } else if (position_control_out < -position_control_out_limit) {
//                            position_control_out =  -position_control_out_limit;
//                        }
//
//                        i_motorcontrol.set_voltage(position_control_out); //in case of FOC sets Q value, torque controller in FOC is disabled
//                    } // end control
////                    xscope_int(POSITION_CONTROL_OUT, position_control_out);
//
//#ifdef DEBUG
//                    xscope_int(ACTUAL_POSITION, actual_position);
//                    xscope_int(TARGET_POSITION, target_position);
//#endif
//                    //xscope_int(TARGET_POSITION, target_position);
//                } // end control activated
//
//#ifdef USE_XSCOPE
////                        xscope_int(TARGET_POSITION, target_position);
////                        xscope_int(ACTUAL_POSITION, actual_position);
////                        xscope_int(VELOCITY, velocity);
////                        xscope_int(TORQUE, adc_b);
//#endif
//
//                break;
//
//            case !isnull(i_hall) => i_hall.notification():
//
//                switch (i_hall.get_notification()) {
//                    case MOTCTRL_NTF_CONFIG_CHANGED:
//                        config_update_flag = 1;
//                        break;
//                    default:
//                        break;
//                }
//                break;
//
//            case !isnull(i_qei) => i_qei.notification():
//
//                switch (i_qei.get_notification()) {
//                    case MOTCTRL_NTF_CONFIG_CHANGED:
//                        config_update_flag = 1;
//                        break;
//                    default:
//                        break;
//                }
//                break;
//
//            case i_motorcontrol.notification():
//
//                switch (i_motorcontrol.get_notification()) {
//                    case MOTCTRL_NTF_CONFIG_CHANGED:
//                        config_update_flag = 1;
//                        break;
//                    default:
//                        break;
//                }
//                break;
//
//            case i_position_control[int i].set_position(int in_target_position):
//
//                target_position = in_target_position;
//
//                break;
//
//            case i_position_control[int i].set_torque_limit(int in_torque_limit):
//
//                f1_torque_j_lim = in_torque_limit;
//
//                break;
//
//            case i_position_control[int i].get_position() -> int out_position:
//
//                out_position = actual_position;
//                break;
//
//            case i_position_control[int i].get_target_position() -> int out_target_position:
//
//                out_target_position = target_position;
//                break;
//
//            case i_position_control[int i].set_position_control_config(ControlConfig in_params):
//
//                position_control_config = in_params;
//                config_update_flag = 1;
//                break;
//
//            case i_position_control[int i].get_position_control_config() ->  ControlConfig out_config:
//
//                out_config = position_control_config;
//                break;
//
//            case i_position_control[int i].set_position_sensor(int in_sensor_used):
//
//                position_control_config.feedback_sensor = in_sensor_used;
//                target_position = actual_position;
//                config_update_flag = 1;
//
//                break;
//
//            case i_position_control[int i].check_busy() -> int out_activate:
//
//                out_activate = activate;
//                break;
//
//            case i_position_control[int i].enable_position_ctrl():
//
//                if (position_control_config.feedback_sensor == QEI_SENSOR && !isnull(i_qei)) {
//                    actual_position = i_qei.get_qei_position_absolute();
//                } else {
//                    actual_position = i_motorcontrol.get_position_actual();
//                }
//                target_position = actual_position;
//                while (1) {
//                    if (i_motorcontrol.check_busy() == INIT) { //__check_commutation_init(c_commutation);
//#ifdef debug_print
//                        printstrln("position_ctrl_service: commutation initialized");
//#endif
//                        if (i_motorcontrol.get_fets_state() == 0) { // check_fet_state(c_commutation);
//                            i_motorcontrol.set_fets_state(1);
//                            delay_milliseconds(2);
//                        } else {
//                            i_motorcontrol.set_fets_state(1);
//                        }
//
//                        break;
//                    }
//                }
//                if (activate == 0) {
//                    t :> ts;
//                    T_k_1n = ts - 500*USEC_STD;// fake older T_k_1n to avoid error in filters
//                    ts -= USEC_STD * position_control_config.control_loop_period; //run the control loop right affter
//                }
//                activate = 1;
//#ifdef debug_print
//                printstrln("position_ctrl_service: position control activated");
//#endif
//                break;
//
//            case i_position_control[int i].disable_position_ctrl():
//
//                activate = 0;
//                i_motorcontrol.set_voltage(0); //set_commutation_sinusoidal(c_commutation, 0);
//                error_position = 0;
//                error_position_D = 0;
//                error_position_I = 0;
//                previous_error = 0;
//                position_control_out = 0;
//                i_motorcontrol.set_fets_state(0); // disable_motor(c_commutation);
//                delay_milliseconds(30); //wait_ms(30, 1, ts); //
//#ifdef debug_print
//                printstrln("position_ctrl_service: position control disabled");
//#endif
//                break;
//        }
//    }
}

