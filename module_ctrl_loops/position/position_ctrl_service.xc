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
#include <refclk.h>
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
    int int23_position_k = 0;
    SecondOrderLPfilterParam position_SO_LP_filter_param;
    float flt23_position_k = 0;
    float flt23_position_in = 0;
    float flt23_position_k_1n = 0;
    float flt23_position_k_2n = 0;
    int int23_position_ref_k = 0;
    SecondOrderLPfilterParam position_ref_SO_LP_filter_param;
    float flt23_position_ref_in = 0;
    float flt23_position_ref_k = 0;
    float flt23_position_ref_k_1n = 0;
    float flt23_position_ref_k_2n = 0;
    int int23_position_ref_k_in = 0;
    int int23_position_cmd_k = 0;
    int int25_position_k_sens = 0;

    // velocity controller
    int int1_velocity_enable_flag = 0;
    PIDparam velocity_control_pid_param;
    velocity_control_pid_param.scale_factor = 10;
    SecondOrderLPfilterParam velocity_ref_SO_LP_filter_param;
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
    float flt23_velocity_ref_in = 0;
    float flt23_velocity_ref_k = 0;
    float flt23_velocity_ref_k_1n = 0;
    float flt23_velocity_ref_k_2n = 0;
    int int23_velocity_ref_k_in = 0;
    int int23_velocity_cmd_k = 0;
    int int14_velocity_k_sens = 0;
    int int23_feedforward_effort = 0;
    int int23_feedforward_effort_in = 0;

    // torque
    int int1_torque_enable_flag = 0;
    int int23_torque_ref_in = 0;
    int int13_torque_ref = 0;

    timer t;
    unsigned int ts;

    pos_velocity_ctrl_config.int21_max_speed *= 512;
    pos_velocity_ctrl_config.int21_min_position /= 4;
    pos_velocity_ctrl_config.int21_max_position /= 4;

    second_order_LP_filter_init(pos_velocity_ctrl_config.position_fc, pos_velocity_ctrl_config.control_loop_period, position_SO_LP_filter_param);
    second_order_LP_filter_init(pos_velocity_ctrl_config.position_ref_fc, pos_velocity_ctrl_config.control_loop_period, position_ref_SO_LP_filter_param);
    second_order_LP_filter_init(pos_velocity_ctrl_config.velocity_ref_fc, pos_velocity_ctrl_config.control_loop_period, velocity_ref_SO_LP_filter_param);
    second_order_LP_filter_init(pos_velocity_ctrl_config.velocity_fc, pos_velocity_ctrl_config.control_loop_period, velocity_SO_LP_filter_param);
    second_order_LP_filter_init(pos_velocity_ctrl_config.velocity_d_fc, pos_velocity_ctrl_config.control_loop_period, velocity_d_SO_LP_filter_param);

    pid_init(pos_velocity_ctrl_config.int10_P_velocity, pos_velocity_ctrl_config.int10_I_velocity, pos_velocity_ctrl_config.int10_D_velocity,
             pos_velocity_ctrl_config.int21_P_error_limit_velocity, pos_velocity_ctrl_config.int21_I_error_limit_velocity,
             pos_velocity_ctrl_config.int22_integral_limit_velocity, pos_velocity_ctrl_config.int21_max_torque,
             pos_velocity_ctrl_config.control_loop_period, velocity_control_pid_param);

    pid_init(pos_velocity_ctrl_config.int10_P_position, pos_velocity_ctrl_config.int10_I_position, pos_velocity_ctrl_config.int10_D_position,
             pos_velocity_ctrl_config.int21_P_error_limit_position, pos_velocity_ctrl_config.int21_I_error_limit_position,
             pos_velocity_ctrl_config.int22_integral_limit_position, pos_velocity_ctrl_config.int21_max_speed,
             pos_velocity_ctrl_config.control_loop_period, position_control_pid_param);


    downstream_control_data.position_cmd = 0;
    downstream_control_data.velocity_cmd = 0;
    downstream_control_data.torque_cmd = 0;
    downstream_control_data.offset_torque = 0;

    upstream_control_data = i_motorcontrol.update_upstream_control_data();
    int25_position_k_sens = upstream_control_data.position;
    int23_position_k_sens = int25_position_k_sens / 4;
    int23_position_ref_k_in = int23_position_k_sens;

    flt23_position_ref_k = int23_position_ref_k_in;
    flt23_position_ref_k_1n = int23_position_ref_k_in;
    flt23_position_ref_k_2n = int23_position_ref_k_in;
    flt23_position_k = int23_position_ref_k_in;
    flt23_position_k_1n = int23_position_ref_k_in;
    flt23_position_k_2n = int23_position_ref_k_in;

    printstr(">>   SOMANET POSITION CONTROL SERVICE STARTING...\n");

    t :> ts;
    while(1) {
#pragma ordered
        select {
            case t when timerafter(ts + USEC_STD * pos_velocity_ctrl_config.control_loop_period) :> ts:

                upstream_control_data = i_motorcontrol.update_upstream_control_data();

                int14_velocity_k_sens = upstream_control_data.velocity;
                int23_velocity_k_sens = int14_velocity_k_sens * 512;

                int25_position_k_sens = upstream_control_data.position;
                int23_position_k_sens = int25_position_k_sens / 4;
                int23_velocity_ref_k = int23_velocity_ref_k_in;
                int23_feedforward_effort = int23_feedforward_effort_in;

                int13_torque_ref = int23_torque_ref_in;

                if(int1_enable_flag) {
                    // position control
                    if (int1_position_enable_flag == 1) {
                        int23_position_k = int23_position_k_sens;
                        int23_position_ref_k = int23_position_ref_k_in;

                        flt23_position_in = int23_position_k;
                        second_order_LP_filter_update(&flt23_position_k,
                                                      &flt23_position_k_1n,
                                                      &flt23_position_k_2n,
                                                      &flt23_position_in, pos_velocity_ctrl_config.control_loop_period, position_SO_LP_filter_param);
                        int23_position_k = ((int) flt23_position_in);

                        flt23_position_ref_in = int23_position_ref_k;
                        second_order_LP_filter_update(&flt23_position_ref_k,
                                                      &flt23_position_ref_k_1n,
                                                      &flt23_position_ref_k_2n,
                                                      &flt23_position_ref_in, pos_velocity_ctrl_config.control_loop_period, position_ref_SO_LP_filter_param);
                        int23_position_ref_k = ((int) flt23_position_ref_k);

                        if(int23_position_ref_k > pos_velocity_ctrl_config.int21_max_position)
                            int23_position_ref_k = pos_velocity_ctrl_config.int21_max_position;
                        else if (int23_position_ref_k < pos_velocity_ctrl_config.int21_min_position)
                            int23_position_ref_k = pos_velocity_ctrl_config.int21_min_position;

                        // PID parameters should be int9 -> -255 to 255
                        int23_position_cmd_k = pid_update(int23_position_ref_k, int23_position_k, int23_position_k, 0, pos_velocity_ctrl_config.control_loop_period, position_control_pid_param);
                        int23_velocity_ref_k = int23_position_cmd_k;

                        second_order_LP_filter_shift_buffers(&flt23_position_k,
                                                             &flt23_position_k_1n,
                                                             &flt23_position_k_2n);
                        second_order_LP_filter_shift_buffers(&flt23_position_ref_k,
                                                             &flt23_position_ref_k_1n,
                                                             &flt23_position_ref_k_2n);
                    }

                    // velocity control
                    if (int1_velocity_enable_flag == 1 || int1_position_enable_flag == 1) {

                        if (int1_position_enable_flag == 0) {
                        flt23_velocity_ref_in = int23_velocity_ref_k;
                        second_order_LP_filter_update(&flt23_velocity_ref_k,
                                                      &flt23_velocity_ref_k_1n,
                                                      &flt23_velocity_ref_k_2n,
                                                      &flt23_velocity_ref_in, pos_velocity_ctrl_config.control_loop_period, velocity_ref_SO_LP_filter_param);
                        int23_velocity_ref_k = ((int) flt23_velocity_ref_k);
                        }

                        if (int23_velocity_ref_k > pos_velocity_ctrl_config.int21_max_speed)
                            int23_velocity_ref_k = pos_velocity_ctrl_config.int21_max_speed;
                        else if (int23_velocity_ref_k < -pos_velocity_ctrl_config.int21_max_speed)
                            int23_velocity_ref_k = -pos_velocity_ctrl_config.int21_max_speed;

                        flt23_velocity_measured_k = int23_velocity_k_sens;
                        flt23_velocity_d_measured_k = flt23_velocity_measured_k;

                        second_order_LP_filter_update(&flt23_velocity_k,
                                                      &flt23_velocity_k_1n,
                                                      &flt23_velocity_k_2n,
                                                      &flt23_velocity_measured_k, pos_velocity_ctrl_config.control_loop_period, velocity_SO_LP_filter_param);
                        int23_velocity_k = ((int) flt23_velocity_k);

                        second_order_LP_filter_update(&flt23_velocity_d_k,
                                                      &flt23_velocity_d_k_1n,
                                                      &flt23_velocity_d_k_2n,
                                                      &flt23_velocity_d_measured_k, pos_velocity_ctrl_config.control_loop_period, velocity_d_SO_LP_filter_param);
                        int23_velocity_d_k = ((int) flt23_velocity_d_k);

                        // PID parameters should be int9 -> -255 to 255
                        int23_velocity_cmd_k = pid_update(int23_velocity_ref_k, int23_velocity_k, int23_velocity_d_k, int23_feedforward_effort, pos_velocity_ctrl_config.control_loop_period, velocity_control_pid_param);

                        int13_torque_ref = int23_velocity_cmd_k;

                        second_order_LP_filter_shift_buffers(&flt23_velocity_ref_k,
                                                             &flt23_velocity_ref_k_1n,
                                                             &flt23_velocity_ref_k_2n);
                        second_order_LP_filter_shift_buffers(&flt23_velocity_k,
                                                             &flt23_velocity_k_1n,
                                                             &flt23_velocity_k_2n);
                        second_order_LP_filter_shift_buffers(&flt23_velocity_d_k,
                                                             &flt23_velocity_d_k_1n,
                                                             &flt23_velocity_d_k_2n);
                    }

                    if(int13_torque_ref > pos_velocity_ctrl_config.int21_max_torque)
                        int13_torque_ref = pos_velocity_ctrl_config.int21_max_torque;
                    else if (int13_torque_ref < (-pos_velocity_ctrl_config.int21_max_torque))
                        int13_torque_ref = (-pos_velocity_ctrl_config.int21_max_torque);

                    i_motorcontrol.set_torque(int13_torque_ref / 1024);
                }


#ifdef XSCOPE_POSITION_CTRL
                xscope_int(POSITION_REF, int23_position_ref_k);
                xscope_int(POSITION, int23_position_k);
//                xscope_int(POSITION_CMD, int23_velocity_ref_k);
//                xscope_int(POSITION_TEMP1, 0);
//                xscope_int(VELOCITY_REF, int23_velocity_ref_k);
                xscope_int(VELOCITY, int23_velocity_k);
//                xscope_int(VELOCITY_CMD, int23_velocity_cmd_k);
//                xscope_int(VELOCITY_TEMP1, int23_velocity_k_sens);
#endif
#ifdef XSCOPE_POSITION_CTRL_2
                xscope_int(VELOCITY, upstream_control_data.velocity);
                xscope_int(POSITION, upstream_control_data.position);
                xscope_int(TORQUE,   upstream_control_data.computed_torque);
                xscope_int(POSITION_REF, int23_position_ref_k_in * 4);
                xscope_int(TORQUE_CMD, int13_torque_ref / 1024);
#endif


                break;



            case i_position_control[int i].disable():
                int1_enable_flag = 0;
                i_motorcontrol.set_torque_control_disabled();
                i_motorcontrol.set_safe_torque_off_enabled();
                i_motorcontrol.set_brake_status(0);
                break;

            case i_position_control[int i].enable_position_ctrl():
                    int1_enable_flag = 1;
                    int1_position_enable_flag = 1;
                    int1_velocity_enable_flag = 0;
                    int1_torque_enable_flag = 0;
                    upstream_control_data = i_motorcontrol.update_upstream_control_data();
                    int25_position_k_sens = upstream_control_data.position;
                    int23_position_k_sens = int25_position_k_sens / 4;
                    int23_position_ref_k_in = int23_position_k_sens;
                    flt23_position_ref_k = int23_position_ref_k_in;
                    flt23_position_ref_k_1n = int23_position_ref_k_in;
                    flt23_position_ref_k_2n = int23_position_ref_k_in;
                    flt23_position_k = int23_position_ref_k_in;
                    flt23_position_k_1n = int23_position_ref_k_in;
                    flt23_position_k_2n = int23_position_ref_k_in;

                    int23_feedforward_effort_in = 0;
                    pid_reset(position_control_pid_param);
                    i_motorcontrol.set_torque_control_enabled();
                    i_motorcontrol.set_brake_status(1);
                break;
            case i_position_control[int i].set_position(int in_target_position):
                    int23_position_ref_k_in = in_target_position / 4;
                break;
            case i_position_control[int i].set_position_pid_coefficients(int int8_Kp, int int8_Ki, int int8_Kd):
                pid_set_coefficients(int8_Kp, int8_Ki, int8_Kd, position_control_pid_param);
                break;
            case i_position_control[int i].set_position_pid_limits(int int16_P_error_limit, int int16_I_error_limit, int int16_itegral_limit, int int21_target_max_velocity_):
                pos_velocity_ctrl_config.int21_max_speed = int21_target_max_velocity_ * 512;
                pid_set_limits(int16_P_error_limit, int16_I_error_limit, int16_itegral_limit, pos_velocity_ctrl_config.int21_max_speed, position_control_pid_param);
                break;
            case i_position_control[int i].set_position_limits(int position_min_limit, int position_max_limit):
                pos_velocity_ctrl_config.int21_min_position = position_min_limit / 4;
                pos_velocity_ctrl_config.int21_max_position = position_max_limit / 4;
                break;



            case i_position_control[int i].enable_velocity_ctrl():
                    int1_enable_flag = 1;
                    int1_position_enable_flag = 0;
                    int1_velocity_enable_flag = 1;
                    int1_torque_enable_flag = 0;
                    int23_velocity_ref_k_in = 0;
                    int23_feedforward_effort_in = 0;
                    pid_reset(velocity_control_pid_param);
                    i_motorcontrol.set_torque_control_enabled();
                    i_motorcontrol.set_brake_status(1);
                break;

            case i_position_control[int i].set_velocity(int in_target_velocity):
                    int23_velocity_ref_k_in = in_target_velocity * 512;
                break;
            case i_position_control[int i].set_offset_torque(int offset_torque_):
                    int23_feedforward_effort_in = offset_torque_;
                break;
            case i_position_control[int i].set_velocity_pid_coefficients(int int8_Kp, int int8_Ki, int int8_Kd):
                pid_set_coefficients(int8_Kp, int8_Ki, int8_Kd, velocity_control_pid_param);
                break;
            case i_position_control[int i].set_velocity_pid_limits(int int16_P_error_limit, int int16_I_error_limit, int int16_itegral_limit, int int21_target_max_torque_):
                pos_velocity_ctrl_config.int21_max_torque = int21_target_max_torque_;
                pid_set_limits(int16_P_error_limit, int16_I_error_limit, int16_itegral_limit, int21_target_max_torque_, velocity_control_pid_param);
                break;
            case i_position_control[int i].set_velocity_limits(int velocity_min_limit, int velocity_max_limit):
                pos_velocity_ctrl_config.int21_max_speed = velocity_max_limit * 512;
                break;

            case i_position_control[int i].enable_torque_ctrl():
                    int1_enable_flag = 1;
                    int1_position_enable_flag = 0;
                    int1_velocity_enable_flag = 0;
                    int1_torque_enable_flag = 1;
                    int23_torque_ref_in = 0;
                    i_motorcontrol.set_torque_control_enabled();
                    i_motorcontrol.set_brake_status(1);
                break;
            case i_position_control[int i].set_torque(int in_target_torque):
                int23_torque_ref_in = in_target_torque;
                break;
            case i_position_control[int i].set_torque_limits(int torque_min_limit, int torque_max_limit):
                pos_velocity_ctrl_config.int21_max_torque = torque_max_limit;
                break;

            case i_position_control[int i].set_position_velocity_control_config(PosVelocityControlConfig in_config):
                    pos_velocity_ctrl_config = in_config;
                    pos_velocity_ctrl_config.int21_max_speed *= 512;
                    pos_velocity_ctrl_config.int21_min_position /= 4;
                    pos_velocity_ctrl_config.int21_max_position /= 4;
                    pid_init(pos_velocity_ctrl_config.int10_P_velocity, pos_velocity_ctrl_config.int10_I_velocity, pos_velocity_ctrl_config.int10_D_velocity,
                             pos_velocity_ctrl_config.int21_P_error_limit_velocity, pos_velocity_ctrl_config.int21_I_error_limit_velocity,
                             pos_velocity_ctrl_config.int22_integral_limit_velocity, pos_velocity_ctrl_config.int21_max_torque,
                             pos_velocity_ctrl_config.control_loop_period, velocity_control_pid_param);
                    pid_init(pos_velocity_ctrl_config.int10_P_position, pos_velocity_ctrl_config.int10_I_position, pos_velocity_ctrl_config.int10_D_position,
                             pos_velocity_ctrl_config.int21_P_error_limit_position, pos_velocity_ctrl_config.int21_I_error_limit_position,
                             pos_velocity_ctrl_config.int22_integral_limit_position, pos_velocity_ctrl_config.int21_max_speed,
                             pos_velocity_ctrl_config.control_loop_period, position_control_pid_param);
                    second_order_LP_filter_init(/*f_c=*//*80*/pos_velocity_ctrl_config.position_fc, /*T_s=*/pos_velocity_ctrl_config.control_loop_period, position_SO_LP_filter_param);
                    second_order_LP_filter_init(/*f_c=*//*5*/ pos_velocity_ctrl_config.position_ref_fc, /*T_s=*/pos_velocity_ctrl_config.control_loop_period, position_ref_SO_LP_filter_param);
                    second_order_LP_filter_init(/*f_c=*//*25*/pos_velocity_ctrl_config.velocity_ref_fc, /*T_s=*/pos_velocity_ctrl_config.control_loop_period, velocity_ref_SO_LP_filter_param);
                    second_order_LP_filter_init(/*f_c=*//*80*/pos_velocity_ctrl_config.velocity_fc, /*T_s=*/pos_velocity_ctrl_config.control_loop_period, velocity_SO_LP_filter_param);
                    second_order_LP_filter_init(/*f_c=*//*75*/pos_velocity_ctrl_config.velocity_d_fc, /*T_s=*/pos_velocity_ctrl_config.control_loop_period, velocity_d_SO_LP_filter_param);
                break;

            case i_position_control[int i].get_position_velocity_control_config() ->  PosVelocityControlConfig out_config:
                    out_config = pos_velocity_ctrl_config;
                    out_config.int21_max_speed /= 512;
                    out_config.int21_min_position *= 4;
                    out_config.int21_max_position *= 4;
                break;

            case i_position_control[int i].get_position() -> int out_position:
                    out_position = upstream_control_data.position;
                break;


            case i_position_control[int i].get_velocity() -> int out_velocity:
                    out_velocity = upstream_control_data.velocity;
                break;

            case i_position_control[int i].check_busy() -> int out_activate:
//                out_activate = int1_position_enable_flag;
                break;


            case i_position_control[int i].update_control_data(DownstreamControlData downstream_control_data_) -> UpstreamControlData upstream_control_data_:
                    upstream_control_data_ = upstream_control_data;
                    downstream_control_data = downstream_control_data_;
                    int23_position_ref_k_in = downstream_control_data.position_cmd / 4;
                    int23_velocity_ref_k_in = downstream_control_data.velocity_cmd * 512;
                    int23_torque_ref_in = downstream_control_data.torque_cmd;
                    int23_feedforward_effort_in = downstream_control_data.offset_torque;
                break;


        }
    }
}
