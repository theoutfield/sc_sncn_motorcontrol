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



void init_position_velocity_control(interface PositionVelocityCtrlInterface client i_position_control)
{

}


void position_velocity_control_service(PosVelocityControlConfig &pos_velocity_ctrl_config,
                              interface MotorcontrolInterface client i_motorcontrol,
                              interface PositionVelocityCtrlInterface server i_position_control[3])
{

    // position controller
    int int1_position_enable_flag = 0;
    PIDparam position_control_pid_param;
    int int20_position_k_sens = 0;
    int int20_position_k = 0; // range: -7FFFF to 7FFFF  OR  -524287 to 524287
    int int20_position_ref_k = 0;
    int int20_position_ref_k_in = 0;
    int int31_position_cmd_k = 0;

    // velocity controller
    int int1_velocity_enable_flag = 0;
    PIDparam velocity_control_pid_param;
    SecondOrderLPfilterParam velocity_SO_LP_filter_param;
    SecondOrderLPfilterParam velocity_d_SO_LP_filter_param;
    int int16_velocity_k = 0;
    float flt20_velocity_measured_k = 0;
    float flt20_velocity_k = 0;
    float flt20_velocity_k_1n = 0;
    float flt20_velocity_k_2n = 0;
    float flt20_velocity_d_measured_k = 0;
    float flt20_velocity_d_k = 0;
    float flt20_velocity_d_k_1n = 0;
    float flt20_velocity_d_k_2n = 0;
    int int20_velocity_k = 0;
    int int20_velocity_d_k = 0;
    int int20_velocity_ref_k = 0;
    int int20_velocity_ref_k_in = 0;
    int int32_velocity_cmd_k = 0;
    int int16_velocity_temp1 = 0;
    int int16_velocity_temp2 = 0;

    // torque
    int int1_torque_enable_flag = 0;
    int int16_torque_ref = 0;


    timer t;
    unsigned int ts;


    second_order_LP_filter_init(/*f_c=*/100, /*T_s=*/1000, velocity_SO_LP_filter_param);
    second_order_LP_filter_init(/*f_c=*/100, /*T_s=*/1000, velocity_d_SO_LP_filter_param);

    pid_init(pos_velocity_ctrl_config.int9_P_velocity, pos_velocity_ctrl_config.int9_I_velocity, pos_velocity_ctrl_config.int9_D_velocity,
             pos_velocity_ctrl_config.int21_P_error_limit_velocity, pos_velocity_ctrl_config.int21_I_error_limit_velocity,
             pos_velocity_ctrl_config.int22_integral_limit_velocity, pos_velocity_ctrl_config.int32_cmd_limit_velocity,
             pos_velocity_ctrl_config.control_loop_period, velocity_control_pid_param);

    pid_init(pos_velocity_ctrl_config.int9_P_position, pos_velocity_ctrl_config.int9_I_position, pos_velocity_ctrl_config.int9_D_position,
             pos_velocity_ctrl_config.int21_P_error_limit_position, pos_velocity_ctrl_config.int21_I_error_limit_position,
             pos_velocity_ctrl_config.int22_integral_limit_position, pos_velocity_ctrl_config.int32_cmd_limit_position,
             pos_velocity_ctrl_config.control_loop_period, position_control_pid_param);


    i_motorcontrol.set_offset_value(3040);
    delay_milliseconds(1000);
    i_motorcontrol.set_torque_control_enabled();
    delay_milliseconds(1000);

//    int20_position_ref_k = i_motorcontrol.get_position_actual();

    printstr(">>   SOMANET POSITION CONTROL SERVICE STARTING...\n");
    t :> ts;
    while(1) {
#pragma ordered
        select {
            case t when timerafter(ts + USEC_STD * 1000/*position_control_config.control_loop_period*/) :> ts:

                int16_velocity_k = i_motorcontrol.get_velocity_actual();
                int20_position_k_sens = i_motorcontrol.get_position_actual();

                // position control
                if (int1_position_enable_flag == 1) {
                    int20_position_k = int20_position_k_sens;// / 100; //100 ro bayad hazf konim
                    int20_position_ref_k = int20_position_ref_k_in;
                    if(int20_position_ref_k > pos_velocity_ctrl_config.int21_target_max_position)
                        int20_position_ref_k = pos_velocity_ctrl_config.int21_target_max_position;
                    else if (int20_position_ref_k < pos_velocity_ctrl_config.int21_target_min_position)
                        int20_position_ref_k = pos_velocity_ctrl_config.int21_target_min_position;

                    // PID parameters should be int8
                    int31_position_cmd_k = pid_update(int20_position_ref_k, int20_position_k, int20_position_k, 1000, position_control_pid_param);
                    int20_velocity_ref_k = int31_position_cmd_k / 2048; //use 11 times shift to right
                }
                else if (int1_position_enable_flag == 0)
                    int20_velocity_ref_k = int20_velocity_ref_k_in;



                // velocity control
                if (int1_velocity_enable_flag == 1 || int1_position_enable_flag == 1) {
                    if (int20_velocity_ref_k > pos_velocity_ctrl_config.int21_target_max_velocity) //int21_target_max_velocity should be int20
                        int20_velocity_ref_k = pos_velocity_ctrl_config.int21_target_max_velocity;
                    else if (int20_velocity_ref_k < pos_velocity_ctrl_config.int21_target_min_velocity)
                        int20_velocity_ref_k = pos_velocity_ctrl_config.int21_target_min_velocity;

                    flt20_velocity_measured_k = int16_velocity_k * 16; //use 4 times shit to left
                    flt20_velocity_d_measured_k = flt20_velocity_measured_k;

                    second_order_LP_filter_update(&flt20_velocity_k,
                                                  &flt20_velocity_k_1n,
                                                  &flt20_velocity_k_2n,
                                                  &flt20_velocity_measured_k, 1000, velocity_SO_LP_filter_param);
                    int20_velocity_k = ((int) flt20_velocity_k);

                    second_order_LP_filter_update(&flt20_velocity_d_k,
                                                  &flt20_velocity_d_k_1n,
                                                  &flt20_velocity_d_k_2n,
                                                  &flt20_velocity_d_measured_k, 1000, velocity_d_SO_LP_filter_param);
                    int20_velocity_d_k = ((int) flt20_velocity_d_k);

                    // PID parameters should be int9
                    int32_velocity_cmd_k = pid_update(int20_velocity_ref_k, int20_velocity_k, int20_velocity_d_k, 1000, velocity_control_pid_param);

                    if(int32_velocity_cmd_k > pos_velocity_ctrl_config.int21_target_max_torque) //int21_target_max_torque should be int32
                        int32_velocity_cmd_k = pos_velocity_ctrl_config.int21_target_max_torque;
                    else if (int32_velocity_cmd_k < pos_velocity_ctrl_config.int21_target_min_torque)
                        int32_velocity_cmd_k = pos_velocity_ctrl_config.int21_target_min_torque;

                    i_motorcontrol.set_torque((int32_velocity_cmd_k / 65535 /*16 times shift*/ )/4); //when Ramin fixes the range to int16 remove the /4



                    second_order_LP_filter_shift_buffers(&flt20_velocity_k,
                                                         &flt20_velocity_k_1n,
                                                         &flt20_velocity_k_2n);

                    second_order_LP_filter_shift_buffers(&flt20_velocity_d_k,
                                                         &flt20_velocity_d_k_1n,
                                                         &flt20_velocity_d_k_2n);
                }
                else if (int1_velocity_enable_flag == 0 && int1_torque_enable_flag == 1)
                    i_motorcontrol.set_torque(int16_torque_ref/4); //when Ramin fixes the range to int16 remove the /4



                xscope_int(POSITION_REF, int20_position_k_sens / 100);//int20_position_ref_k);
                xscope_int(POSITION, int20_position_k);
                xscope_int(POSITION_CMD, int31_position_cmd_k);
//                    xscope_int(POSITION_TEMP1, 0);
//                    xscope_int(POSITION_TEMP2, 0);
                xscope_int(VELOCITY_REF, int20_velocity_ref_k);
                xscope_int(VELOCITY, int16_velocity_k);//int20_velocity_k);
                xscope_int(VELOCITY_CMD, int32_velocity_cmd_k);
//                xscope_int(VELOCITY_TEMP1, 0);
//                xscope_int(VELOCITY_TEMP2, 0);


                break;






            case i_position_control[int i].enable_position_ctrl():
                    int1_position_enable_flag = 1;
                break;
            case i_position_control[int i].disable_position_ctrl():
                    int1_position_enable_flag = 0;
                break;
            case i_position_control[int i].set_position(int in_target_position):
                    int20_position_ref_k_in = in_target_position;
                break;
            case i_position_control[int i].set_position_pid_coefficients(int int8_Kp, int int8_Ki, int int8_Kd):
                pid_set_coefficients(int8_Kp, int8_Ki, int8_Kd, position_control_pid_param);
                break;
            case i_position_control[int i].set_position_pid_limits(int int16_P_error_limit, int int16_I_error_limit, int int16_itegral_limit, int int16_cmd_limit):
                pid_set_limits(int16_P_error_limit, int16_I_error_limit, int16_itegral_limit, int16_cmd_limit, position_control_pid_param);
                break;
            case i_position_control[int i].set_position_limits(int position_min_limit, int position_max_limit):
                pos_velocity_ctrl_config.int21_target_min_position = position_min_limit;
                pos_velocity_ctrl_config.int21_target_max_position = position_max_limit;
                break;



            case i_position_control[int i].enable_velocity_ctrl():
                    int1_velocity_enable_flag = 1;
                break;
            case i_position_control[int i].disable_velocity_ctrl():
                    int1_velocity_enable_flag = 0;
                break;
            case i_position_control[int i].set_velocity(int in_target_velocity):
                    int20_velocity_ref_k_in = in_target_velocity;
                break;
            case i_position_control[int i].set_velocity_pid_coefficients(int int8_Kp, int int8_Ki, int int8_Kd):
                pid_set_coefficients(int8_Kp, int8_Ki, int8_Kd, velocity_control_pid_param);
                break;
            case i_position_control[int i].set_velocity_pid_limits(int int16_P_error_limit, int int16_I_error_limit, int int16_itegral_limit, int int16_cmd_limit):
                pid_set_limits(int16_P_error_limit, int16_I_error_limit, int16_itegral_limit, int16_cmd_limit, velocity_control_pid_param);
                break;
            case i_position_control[int i].set_velocity_limits(int velocity_min_limit, int velocity_max_limit):
                pos_velocity_ctrl_config.int21_target_min_velocity = velocity_min_limit;
                pos_velocity_ctrl_config.int21_target_max_velocity = velocity_max_limit;
                break;

            case i_position_control[int i].enable_torque_ctrl():
                int1_torque_enable_flag = 1;
                break;
            case i_position_control[int i].disable_torque_ctrl():
                int1_torque_enable_flag = 0;
                break;
            case i_position_control[int i].set_torque(int in_target_torque):
                int16_torque_ref = in_target_torque;
                break;
            case i_position_control[int i].set_torque_limits(int torque_min_limit, int torque_max_limit):
                pos_velocity_ctrl_config.int21_target_min_torque = torque_min_limit;
                pos_velocity_ctrl_config.int21_target_max_torque = torque_max_limit;
                break;

            case i_position_control[int i].set_position_velocity_control_config(PosVelocityControlConfig in_config):
                    pos_velocity_ctrl_config = in_config;
                break;

            case i_position_control[int i].get_position_velocity_control_config() ->  PosVelocityControlConfig out_config:
                    out_config = pos_velocity_ctrl_config;
                break;



            case i_position_control[int i].get_position() -> int out_position:
                break;


            case i_position_control[int i].get_velocity() -> int out_velocity:
                break;

        }
    }
}
