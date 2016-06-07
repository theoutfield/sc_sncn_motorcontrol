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


void position_velocity_control_service(PosVelocityControlConfig &pos_velocity_ctrl_config,
                              interface MotorcontrolInterface client i_motorcontrol,
                              interface PositionVelocityCtrlInterface server i_position_control[3])
{
    UpstreamControlData upstream_control_data;
    DownstreamControlData downstream_control_data;
    int int1_enable_flag = 0;

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


    second_order_LP_filter_init(/*f_c=*/60, /*T_s=*/1000, velocity_SO_LP_filter_param);
    second_order_LP_filter_init(/*f_c=*/60, /*T_s=*/1000, velocity_d_SO_LP_filter_param);

    pid_init(pos_velocity_ctrl_config.int10_P_velocity, pos_velocity_ctrl_config.int10_I_velocity, pos_velocity_ctrl_config.int10_D_velocity,
             pos_velocity_ctrl_config.int21_P_error_limit_velocity, pos_velocity_ctrl_config.int21_I_error_limit_velocity,
             pos_velocity_ctrl_config.int22_integral_limit_velocity, pos_velocity_ctrl_config.int32_cmd_limit_velocity,
             pos_velocity_ctrl_config.control_loop_period, velocity_control_pid_param);

    pid_init(pos_velocity_ctrl_config.int10_P_position, pos_velocity_ctrl_config.int10_I_position, pos_velocity_ctrl_config.int10_D_position,
             pos_velocity_ctrl_config.int21_P_error_limit_position, pos_velocity_ctrl_config.int21_I_error_limit_position,
             pos_velocity_ctrl_config.int22_integral_limit_position, pos_velocity_ctrl_config.int32_cmd_limit_position,
             pos_velocity_ctrl_config.control_loop_period, position_control_pid_param);



//    delay_milliseconds(1000);
//    i_motorcontrol.set_torque_control_enabled();
//    delay_milliseconds(1000);

    downstream_control_data.position_cmd = 0;
    downstream_control_data.velocity_cmd = 0;
    downstream_control_data.torque_cmd = 0;
    downstream_control_data.offset_torque = 0;

//    int25_position_k_sens_ofset = i_motorcontrol.get_position_actual();
    upstream_control_data = i_motorcontrol.update_upstream_control_data();
    int25_position_k_sens = upstream_control_data.position;// - int25_position_k_sens_ofset;
    int23_position_k_sens = int25_position_k_sens / 4; //-4194303 to 4194303
    int23_position_ref_k_in = int23_position_k_sens;

    printstr(">>   SOMANET POSITION CONTROL SERVICE STARTING...\n");
    t :> ts;
    while(1) {
#pragma ordered
        select {
            case t when timerafter(ts + USEC_STD * 1000/*position_control_config.control_loop_period*/) :> ts:

                upstream_control_data = i_motorcontrol.update_upstream_control_data();

                int14_velocity_k_sens = upstream_control_data.velocity;
                int23_velocity_k_sens = int14_velocity_k_sens * 512; //-4194303 to 4194303

                int25_position_k_sens = upstream_control_data.position;// - int25_position_k_sens_ofset;
                int23_position_k_sens = int25_position_k_sens / 4; //-4194303 to 4194303

                if(int1_enable_flag) {
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

                    int23_position_ref_k_in = int23_position_k_sens;
//                    int25_position_k_sens_ofset = i_motorcontrol.get_position_actual();
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
                    if(int1_torque_enable_flag == 1) {
                        int13_torque_ref = int23_torque_ref_in / 1024;
                        //                    int13_torque_ref = int16_torque_ref / 8;
                        i_motorcontrol.set_torque(int13_torque_ref);
                    }
                }

                }

#ifdef XSCOPE_POSITION_CTRL
                xscope_int(POSITION_REF, int23_position_ref_k_in*4);
                xscope_int(POSITION, upstream_control_data.position);
//                xscope_int(POSITION_CMD, int23_velocity_ref_k);
//                    xscope_int(POSITION_TEMP1, 0);
//                    xscope_int(POSITION_TEMP2, 0);
//                xscope_int(VELOCITY_REF, int23_velocity_ref_k);
                xscope_int(VELOCITY, upstream_control_data.velocity);
                xscope_int(VELOCITY_CMD, int13_torque_ref);
                xscope_int(VELOCITY_FILTERED, int23_velocity_k);
                xscope_int(TORQUE, upstream_control_data.computed_torque);
//                xscope_int(VELOCITY_TEMP2, 0);
#endif


                break;



//remove
            case i_position_control[int i].enable():
//                int1_enable_flag = 1;
//                int1_torque_enable_flag = 1;
//                int1_velocity_enable_flag = 0;
//                int1_position_enable_flag = 0;
//                int23_torque_ref_in = 0;
//                i_motorcontrol.set_torque_control_enabled();
                break;

            case i_position_control[int i].disable():
                int1_enable_flag = 0;
                i_motorcontrol.set_torque_control_disabled();
                i_motorcontrol.set_safe_torque_off_enabled();
                break;

            case i_position_control[int i].enable_position_ctrl():
                    int1_enable_flag = 1;
                    int1_position_enable_flag = 1;
                    int1_velocity_enable_flag = 0;
                    int1_torque_enable_flag = 0;
                    upstream_control_data = i_motorcontrol.update_upstream_control_data();
                    int25_position_k_sens = upstream_control_data.position;// - int25_position_k_sens_ofset;
                    int23_position_k_sens = int25_position_k_sens / 4; //-4194303 to 4194303
                    int23_position_ref_k_in = int23_position_k_sens;
                    pid_reset(position_control_pid_param);
                    i_motorcontrol.set_torque_control_enabled();
                break;
            case i_position_control[int i].disable_position_ctrl():
                    int1_position_enable_flag = 0;
                break;
            case i_position_control[int i].set_position(int in_target_position):
                    int23_position_ref_k_in = in_target_position;
                    int1_torque_enable_flag = 0;
                    int1_velocity_enable_flag = 0;
                    int1_position_enable_flag = 1;
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



                //remove
            case i_position_control[int i].enable_velocity_ctrl():
                    int1_enable_flag = 1;
                    int1_position_enable_flag = 0;
                    int1_velocity_enable_flag = 1;
                    int1_torque_enable_flag = 0;
                    int23_velocity_ref_k_in = 0;
                    pid_reset(velocity_control_pid_param);
                    i_motorcontrol.set_torque_control_enabled();
                break;
                //remove
            case i_position_control[int i].disable_velocity_ctrl():
//                    int1_velocity_enable_flag = 0;
                break;
            case i_position_control[int i].set_velocity(int in_target_velocity):
                    int23_velocity_ref_k_in = in_target_velocity;
                    int1_torque_enable_flag = 0;
                    int1_velocity_enable_flag = 1;
                    int1_position_enable_flag = 0;
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

                //remove
            case i_position_control[int i].enable_torque_ctrl():
                    int1_enable_flag = 1;
                    int1_position_enable_flag = 0;
                    int1_velocity_enable_flag = 0;
                    int1_torque_enable_flag = 1;
                    int23_torque_ref_in = 0;
                    i_motorcontrol.set_torque_control_enabled();
                break;
                //remove
            case i_position_control[int i].disable_torque_ctrl():
                int1_torque_enable_flag = 0;
                break;
            case i_position_control[int i].set_torque(int in_target_torque):
                int23_torque_ref_in = in_target_torque;
                int1_torque_enable_flag = 1;
                int1_velocity_enable_flag = 0;
                int1_position_enable_flag = 0;
                break;
            case i_position_control[int i].set_torque_limits(int torque_min_limit, int torque_max_limit):
                pos_velocity_ctrl_config.int21_target_min_torque = torque_min_limit;
                pos_velocity_ctrl_config.int21_target_max_torque = torque_max_limit;
                break;

            case i_position_control[int i].set_position_velocity_control_config(PosVelocityControlConfig in_config):
                    pos_velocity_ctrl_config = in_config;
                    pid_init(pos_velocity_ctrl_config.int10_P_velocity, pos_velocity_ctrl_config.int10_I_velocity, pos_velocity_ctrl_config.int10_D_velocity,
                             pos_velocity_ctrl_config.int21_P_error_limit_velocity, pos_velocity_ctrl_config.int21_I_error_limit_velocity,
                             pos_velocity_ctrl_config.int22_integral_limit_velocity, pos_velocity_ctrl_config.int32_cmd_limit_velocity,
                             pos_velocity_ctrl_config.control_loop_period, velocity_control_pid_param);

                    pid_init(pos_velocity_ctrl_config.int10_P_position, pos_velocity_ctrl_config.int10_I_position, pos_velocity_ctrl_config.int10_D_position,
                             pos_velocity_ctrl_config.int21_P_error_limit_position, pos_velocity_ctrl_config.int21_I_error_limit_position,
                             pos_velocity_ctrl_config.int22_integral_limit_position, pos_velocity_ctrl_config.int32_cmd_limit_position,
                             pos_velocity_ctrl_config.control_loop_period, position_control_pid_param);

                break;

            case i_position_control[int i].get_position_velocity_control_config() ->  PosVelocityControlConfig out_config:
                    out_config = pos_velocity_ctrl_config;
                break;



            case i_position_control[int i].get_position() -> int out_position:
                    out_position = upstream_control_data.position;
                break;


            case i_position_control[int i].get_velocity() -> int out_velocity:
                    out_velocity = upstream_control_data.velocity;
                break;

            case i_position_control[int i].check_busy() -> int out_activate:
                out_activate = int1_position_enable_flag;
                break;

//            case i_position_control[int i].update_upstream_control_data(DownstreamControlData downstream_control_data_) -> UpstreamControlData upstream_control_data_:
//                    upstream_control_data = upstream_control_data;
//                    downstream_control_data_ = downstream_control_data;
//                break;

            case i_position_control[int i].update_control_data(DownstreamControlData downstream_control_data_) -> UpstreamControlData upstream_control_data_:
                    upstream_control_data_ = upstream_control_data;
                    downstream_control_data = downstream_control_data_;
                    int23_position_ref_k_in = downstream_control_data.position_cmd / 4;
                    int23_velocity_ref_k_in = downstream_control_data.velocity_cmd * 512;
                    int23_torque_ref_in = downstream_control_data.torque_cmd;
//                    downstream_control_data.torque_ofset = 0;
                break;


        }
    }
}
