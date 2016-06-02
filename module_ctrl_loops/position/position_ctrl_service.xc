/**
 * @file  position_ctrl_server.xc
 * @brief Position Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/
#include <xs1.h>
#include <xscope.h>
#include <print.h>
#include <stdlib.h>

#include <controllers_lib.h>
#include <filters_lib.h>

#include <position_ctrl_service.h>
#include <a4935.h>
#include <mc_internal_constants.h>
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

    unsigned T_s_desired = 1000; //us


    //Joint Torque Control
//    int i1_torque_j_ref = 0;
    int f1_torque_j_lim = 50000;
//    float f1_torque_j_sens_measured = 0;
    int i1_torque_j_sens_offset = 0;
    int i1_torque_j_sens_offset_accumulator = 0;

    // velocity controller
    PIDparam velocity_control_pid_param;
    SecondOrderLPfilterParam velocity_SO_LP_filter_param;
    SecondOrderLPfilterParam velocity_d_SO_LP_filter_param;
    int int32_velocity_k = 0;
    float float_velocity_measured_k = 0;
    float float_velocity_k = 0;
    float float_velocity_k_1n = 0;
    float float_velocity_k_2n = 0;
    float float_velocity_d_measured_k = 0;
    float float_velocity_d_k = 0;
    float float_velocity_d_k_1n = 0;
    float float_velocity_d_k_2n = 0;
    int int16_velocity_k = 0;
    int int16_velocity_d_k = 0;
    int int16_velocity_ref_k = 0;
    int int16_velocity_cmd_k = 0;
    int int16_velocity_temp1 = 0;
    int int16_velocity_temp2 = 0;

    // position controller
    int int32_position_k = 0;
    int int16_position_k = 0;
    int int16_position_ref_k = 0;
    int int16_position_cmd_k = 0;

    timer t;
    unsigned int ts;

    int activate = 0;

    int config_update_flag = 1;

//    int i=0;
//    int offset=0;

//    MotorcontrolConfig motorcontrol_config;

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
        printstrln(">>   POSITION CONTROL CASCADED WITH TORQUE");
    }

    t :> ts;

    second_order_LP_filter_init(/*f_c=*/100, /*T_s=*/1000, velocity_SO_LP_filter_param);
    second_order_LP_filter_init(/*f_c=*/70, /*T_s=*/1000, velocity_d_SO_LP_filter_param);

    pid_init(/*i1_P*/0, /*i1_I*/0, /*i1_D*/0, /*i1_P_error_limit*/0,
             /*i1_I_error_limit*/0, /*i1_itegral_limit*/0, /*i1_cmd_limit*/0, /*i1_T_s*/1000, velocity_control_pid_param);


    i_motorcontrol.set_offset_value(3040);
    delay_milliseconds(1000);
    i_motorcontrol.set_torque_control_enabled();
    delay_milliseconds(1000);

    while(1) {
#pragma ordered
        select {
            case t when timerafter(ts + USEC_STD * 1000/*position_control_config.control_loop_period*/) :> ts:

                int32_velocity_k = i_motorcontrol.get_velocity_actual();
                int32_position_k = i_motorcontrol.get_position_actual();

                if (activate == 1) {

                    // velocity controller
                    float_velocity_measured_k = int32_velocity_k * 20;//the received velocity is smaller than the int16 range and I just multiply by a big number to expand the range.
                    float_velocity_d_measured_k = float_velocity_measured_k;

                    second_order_LP_filter_update(&float_velocity_k,
                                                  &float_velocity_k_1n,
                                                  &float_velocity_k_2n,
                                                  &float_velocity_measured_k, 1000, velocity_SO_LP_filter_param);
                    int16_velocity_k = ((int) float_velocity_k);

                    second_order_LP_filter_update(&float_velocity_d_k,
                                                  &float_velocity_d_k_1n,
                                                  &float_velocity_d_k_2n,
                                                  &float_velocity_d_measured_k, 1000, velocity_d_SO_LP_filter_param);
                    int16_velocity_d_k = ((int) float_velocity_d_k);

                    int16_velocity_temp2 = int16_velocity_d_k - velocity_control_pid_param.int16_feedback_d_filter_1n;

                    int16_velocity_ref_k = int16_position_ref_k;

                    int16_velocity_cmd_k = pid_update(int16_velocity_ref_k, int16_velocity_k, int16_velocity_d_k, 1000, velocity_control_pid_param);


                    // position controller
                    int16_position_k = int32_position_k / 1000;
                    int16_position_cmd_k = int16_velocity_cmd_k;
                    i_motorcontrol.set_torque(int16_position_cmd_k);


                    second_order_LP_filter_shift_buffers(&float_velocity_k,
                                                         &float_velocity_k_1n,
                                                         &float_velocity_k_2n);

                    second_order_LP_filter_shift_buffers(&float_velocity_d_k,
                                                         &float_velocity_d_k_1n,
                                                         &float_velocity_d_k_2n);
                } // end control activated

                xscope_int(VELOCITY_REF, int16_velocity_ref_k);
                xscope_int(VELOCITY, int32_velocity_k * 20);//int16_velocity_k);
                xscope_int(VELOCITY_CMD, int16_velocity_k);//int16_velocity_cmd_k);
                xscope_int(VELOCITY_TEMP1, int16_velocity_d_k);
                xscope_int(VELOCITY_TEMP2, int16_velocity_temp2);

                xscope_int(POSITION_REF, int16_velocity_cmd_k);//int16_position_ref_k);
                xscope_int(POSITION, int16_position_k);
                xscope_int(POSITION_CMD, int16_velocity_cmd_k);


                break;

            case i_motorcontrol.notification():
                break;

            case i_position_control[int i].set_position(int in_target_position):
                    int16_position_ref_k = in_target_position;
                break;

            case i_position_control[int i].set_velocity_pid_coefficients(int int8_Kp, int int8_Ki, int int8_Kd):
                pid_set_coefficients(int8_Kp, int8_Ki, int8_Kd, velocity_control_pid_param);
                break;

            case i_position_control[int i].set_velocity_pid_limits(int int16_P_error_limit, int int16_I_error_limit, int int16_itegral_limit, int int16_cmd_limit):
                pid_set_limits(int16_P_error_limit, int16_I_error_limit, int16_itegral_limit, int16_cmd_limit, velocity_control_pid_param);
                break;

            case i_position_control[int i].set_torque_limit(int in_torque_limit):
                break;

            case i_position_control[int i].get_position() -> int out_position:
                break;

            case i_position_control[int i].get_target_position() -> int out_target_position:
                break;

            case i_position_control[int i].set_position_control_config(ControlConfig in_params):
                break;

            case i_position_control[int i].get_position_control_config() ->  ControlConfig out_config:
                break;

            case i_position_control[int i].set_position_sensor(int in_sensor_used):
                break;

            case i_position_control[int i].check_busy() -> int out_activate:
                break;

            case i_position_control[int i].enable_position_ctrl():
                    activate = 1;
                break;

            case i_position_control[int i].disable_position_ctrl():
                    activate = 0;
                break;
        }
    }
}
