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
    position_control_pid_param.scale_factor = 3;
    int int23_position_k_sens = 0;
    int int23_position_k = 0; // range: -7FFFF to 7FFFF  OR  -524287 to 524287
    int int23_position_ref_k = 0;
    int int23_position_ref_k_in = 0;
    int int23_position_cmd_k = 0;
    int int25_position_k_sens = 0;
    int int25_position_k_sens_ofset = 0;

    // velocity controller
    int int1_velocity_enable_flag = 0;
    PIDparam velocity_control_pid_param;
    velocity_control_pid_param.scale_factor = 10;
    SecondOrderLPfilterParam velocity_SO_LP_filter_param;
    SecondOrderLPfilterParam velocity_d_SO_LP_filter_param;
    int int23_velocity_k = 0;
    float flt23_velocity_measured_k = 0;
    float flt23_velocity_k = 0;
    float flt23_velocity_k_1n = 0;
    float flt23_velocity_k_2n = 0;
    float flt23_velocity_d_measured_k = 0;
    float flt23_velocity_d_k = 0;
    float flt23_velocity_d_k_1n = 0;
    float flt23_velocity_d_k_2n = 0;
    int int23_velocity_k_sens = 0;
    int int23_velocity_d_k = 0;
    int int23_velocity_ref_k = 0;
    int int23_velocity_ref_k_in = 0;
    int int23_velocity_cmd_k = 0;
    int int16_velocity_temp1 = 0;
    int int16_velocity_temp2 = 0;
    int int14_velocity_k_sens = 0;

    // torque
    int int1_torque_enable_flag = 0;
    int int16_torque_ref = 0;
    int int23_torque_ref_in = 0;
    int int15_torque_ref = 0; //currently the torque controller accepts int13 (-4095 to 4095) that should be changed to int16 [verified on 06.June.16]
    int int13_torque_ref = 0;


    timer t;
    unsigned int ts;


    second_order_LP_filter_init(/*f_c=*/100, /*T_s=*/1000, velocity_SO_LP_filter_param);
    second_order_LP_filter_init(/*f_c=*/100, /*T_s=*/1000, velocity_d_SO_LP_filter_param);

    pid_init(pos_velocity_ctrl_config.int10_P_velocity, pos_velocity_ctrl_config.int10_I_velocity, pos_velocity_ctrl_config.int10_D_velocity,
             pos_velocity_ctrl_config.int21_P_error_limit_velocity, pos_velocity_ctrl_config.int21_I_error_limit_velocity,
             pos_velocity_ctrl_config.int22_integral_limit_velocity, pos_velocity_ctrl_config.int32_cmd_limit_velocity,
             pos_velocity_ctrl_config.control_loop_period, velocity_control_pid_param);

    pid_init(pos_velocity_ctrl_config.int10_P_position, pos_velocity_ctrl_config.int10_I_position, pos_velocity_ctrl_config.int10_D_position,
             pos_velocity_ctrl_config.int21_P_error_limit_position, pos_velocity_ctrl_config.int21_I_error_limit_position,
             pos_velocity_ctrl_config.int22_integral_limit_position, pos_velocity_ctrl_config.int32_cmd_limit_position,
             pos_velocity_ctrl_config.control_loop_period, position_control_pid_param);


    i_motorcontrol.set_offset_value(3040);
    delay_milliseconds(1000);
    i_motorcontrol.set_torque_control_enabled();
    delay_milliseconds(1000);

    int25_position_k_sens_ofset = i_motorcontrol.get_position_actual();

    printstr(">>   SOMANET POSITION CONTROL SERVICE STARTING...\n");
    t :> ts;
    while(1) {
#pragma ordered
        select {
            case t when timerafter(ts + USEC_STD * 1000/*position_control_config.control_loop_period*/) :> ts:

                int14_velocity_k_sens = i_motorcontrol.get_velocity_actual(); //-8191RPM to 8191RPM
                int23_velocity_k_sens = int14_velocity_k_sens * 512; //-4194303 to 4194303
                int25_position_k_sens = i_motorcontrol.get_position_actual() - int25_position_k_sens_ofset;
                int23_position_k_sens = int25_position_k_sens / 4; //-4194303 to 4194303

                // position control
                if (int1_position_enable_flag == 1) {
                    int23_position_k = int23_position_k_sens;// / 100; //100 ro bayad hazf konim
                    int23_position_ref_k = int23_position_ref_k_in;
                    if(int23_position_ref_k > pos_velocity_ctrl_config.int21_target_max_position)
                        int23_position_ref_k = pos_velocity_ctrl_config.int21_target_max_position;
                    else if (int23_position_ref_k < pos_velocity_ctrl_config.int21_target_min_position)
                        int23_position_ref_k = pos_velocity_ctrl_config.int21_target_min_position;

                    // PID parameters should be int9 -> -255 to 255
                    int23_position_cmd_k = pid_update(int23_position_ref_k, int23_position_k, int23_position_k, 1000, position_control_pid_param);
                    int23_velocity_ref_k = int23_position_cmd_k; //use 14 times shift to right
                }
                else if (int1_position_enable_flag == 0)
                {
                    pid_reset(position_control_pid_param);
                    int25_position_k_sens_ofset = i_motorcontrol.get_position_actual();
                    int23_velocity_ref_k = int23_velocity_ref_k_in; // -524287 to 524287 [verified on 06.June.16]
                }



                // velocity control
                if (int1_velocity_enable_flag == 1 || int1_position_enable_flag == 1) {
                    if (int23_velocity_ref_k > pos_velocity_ctrl_config.int21_target_max_velocity) //int21_target_max_velocity should be int20
                        int23_velocity_ref_k = pos_velocity_ctrl_config.int21_target_max_velocity;
                    else if (int23_velocity_ref_k < pos_velocity_ctrl_config.int21_target_min_velocity)
                        int23_velocity_ref_k = pos_velocity_ctrl_config.int21_target_min_velocity;

                    flt23_velocity_measured_k = int23_velocity_k_sens;
                    flt23_velocity_d_measured_k = flt23_velocity_measured_k;

                    second_order_LP_filter_update(&flt23_velocity_k,
                                                  &flt23_velocity_k_1n,
                                                  &flt23_velocity_k_2n,
                                                  &flt23_velocity_measured_k, 1000, velocity_SO_LP_filter_param);
                    int23_velocity_k = ((int) flt23_velocity_k);

                    second_order_LP_filter_update(&flt23_velocity_d_k,
                                                  &flt23_velocity_d_k_1n,
                                                  &flt23_velocity_d_k_2n,
                                                  &flt23_velocity_d_measured_k, 1000, velocity_d_SO_LP_filter_param);
                    int23_velocity_d_k = ((int) flt23_velocity_d_k);

                    // PID parameters should be int9 -> -255 to 255
                    int23_velocity_cmd_k = pid_update(int23_velocity_ref_k, int23_velocity_k, int23_velocity_d_k, 1000, velocity_control_pid_param);

//                    if(int23_velocity_cmd_k > pos_velocity_ctrl_config.int21_target_max_torque) //int21_target_max_torque should be int32
//                        int23_velocity_cmd_k = pos_velocity_ctrl_config.int21_target_max_torque;
//                    else if (int23_velocity_cmd_k < pos_velocity_ctrl_config.int21_target_min_torque)
//                        int23_velocity_cmd_k = pos_velocity_ctrl_config.int21_target_min_torque;

                    int13_torque_ref = int23_velocity_cmd_k / 1024;
//                    int13_torque_ref = int16_torque_ref / 8;
                    i_motorcontrol.set_torque(int13_torque_ref);



                    second_order_LP_filter_shift_buffers(&flt23_velocity_k,
                                                         &flt23_velocity_k_1n,
                                                         &flt23_velocity_k_2n);

                    second_order_LP_filter_shift_buffers(&flt23_velocity_d_k,
                                                         &flt23_velocity_d_k_1n,
                                                         &flt23_velocity_d_k_2n);
                }
                else if (int1_velocity_enable_flag == 0) {
                    pid_reset(velocity_control_pid_param);
                    if(int1_torque_enable_flag == 1) {
                        int13_torque_ref = int23_torque_ref_in / 1024;
                        //                    int13_torque_ref = int16_torque_ref / 8;
                        i_motorcontrol.set_torque(int13_torque_ref);
                    }
                }



//                xscope_int(POSITION_REF, int23_position_ref_k);
//                xscope_int(POSITION, int23_position_k);
//                xscope_int(POSITION_CMD, int23_velocity_ref_k);
//                    xscope_int(POSITION_TEMP1, 0);
//                    xscope_int(POSITION_TEMP2, 0);
//                xscope_int(VELOCITY_REF, int23_velocity_ref_k);
//                xscope_int(VELOCITY, int23_velocity_k);
//                xscope_int(VELOCITY_CMD, int23_velocity_cmd_k);
//                xscope_int(VELOCITY_TEMP1, int23_torque_ref_in);
//                xscope_int(VELOCITY_TEMP2, 0);


                break;






            case i_position_control[int i].enable_position_ctrl():
                    int1_position_enable_flag = 1;
                break;
            case i_position_control[int i].disable_position_ctrl():
                    int1_position_enable_flag = 0;
                break;
            case i_position_control[int i].set_position(int in_target_position):
                    int23_position_ref_k_in = in_target_position;
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
                    int23_velocity_ref_k_in = in_target_velocity;
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
                int23_torque_ref_in = in_target_torque;
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
