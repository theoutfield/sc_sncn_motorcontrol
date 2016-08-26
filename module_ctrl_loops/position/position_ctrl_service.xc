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
#include <math.h>



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
    SecondOrderLPfilterParam position_SO_LP_filter_param;
    SecondOrderLPfilterParam position_ref_SO_LP_filter_param;
    float position_k = 0;
    float position_sens_k = 0;
    float position_k_1n = 0;
    float position_k_2n = 0;
    float position_ref_in_k = 0;
    float position_ref_k = 0;
    float position_ref_k_1n = 0;
    float position_ref_k_2n = 0;
    float position_cmd_k = 0;
    int position_ref_input_k = 0;

    // velocity controller
    int int1_velocity_enable_flag = 0;
    PIDparam velocity_control_pid_param;
    velocity_control_pid_param.scale_factor = 10;
    SecondOrderLPfilterParam velocity_ref_SO_LP_filter_param;
    SecondOrderLPfilterParam velocity_SO_LP_filter_param;
    SecondOrderLPfilterParam velocity_d_SO_LP_filter_param;
    int velocity_ref_input_k = 0;
    int int23_feedforward_effort = 0;
    int int23_feedforward_effort_in = 0;
    float velocity_ref_in_k = 0;
    float velocity_ref_k = 0;
    float velocity_ref_k_1n = 0;
    float velocity_ref_k_2n = 0;
    float velocity_sens_k = 0;
    float velocity_k = 0;
    float velocity_k_1n = 0;
    float velocity_k_2n = 0;
    float velocity_cmd_k = 0;

    // torque
    int int1_torque_enable_flag = 0;
    int int23_torque_ref_in = 0;
    float torque_ref_k = 0;

    // Pointman
    float POS_OFFSET = 0;
    float MAX_POS = 95000;
    float MAX_POS_TORQUE = 850 * (-512);
    float position_sens_k_original = 0;

    timer t;
    unsigned int ts;

    pos_velocity_ctrl_config.int21_max_speed *= 1;
    pos_velocity_ctrl_config.int21_min_position /= 1;
    pos_velocity_ctrl_config.int21_max_position /= 1;

    second_order_LP_filter_init(pos_velocity_ctrl_config.position_fc, pos_velocity_ctrl_config.control_loop_period, position_SO_LP_filter_param);
    second_order_LP_filter_init(pos_velocity_ctrl_config.position_ref_fc, pos_velocity_ctrl_config.control_loop_period, position_ref_SO_LP_filter_param);
    second_order_LP_filter_init(pos_velocity_ctrl_config.velocity_ref_fc, pos_velocity_ctrl_config.control_loop_period, velocity_ref_SO_LP_filter_param);
    second_order_LP_filter_init(pos_velocity_ctrl_config.velocity_fc, pos_velocity_ctrl_config.control_loop_period, velocity_SO_LP_filter_param);
    second_order_LP_filter_init(pos_velocity_ctrl_config.velocity_d_fc, pos_velocity_ctrl_config.control_loop_period, velocity_d_SO_LP_filter_param);

    pid_init(velocity_control_pid_param);
    pid_set_parameters((float)pos_velocity_ctrl_config.int10_P_velocity, (float)pos_velocity_ctrl_config.int10_I_velocity,
                       (float)pos_velocity_ctrl_config.int10_D_velocity, (float)pos_velocity_ctrl_config.int22_integral_limit_velocity,
                              pos_velocity_ctrl_config.control_loop_period, velocity_control_pid_param);
    pid_init(position_control_pid_param);
    pid_set_parameters((float)pos_velocity_ctrl_config.int10_P_position, (float)pos_velocity_ctrl_config.int10_I_position,
                       (float)pos_velocity_ctrl_config.int10_D_position, (float)pos_velocity_ctrl_config.int22_integral_limit_position,
                              pos_velocity_ctrl_config.control_loop_period, position_control_pid_param);


    downstream_control_data.position_cmd = 0;
    downstream_control_data.velocity_cmd = 0;
    downstream_control_data.torque_cmd = 0;
    downstream_control_data.offset_torque = 0;

    position_ref_input_k = upstream_control_data.position;
    position_sens_k = ((float) upstream_control_data.position);
    position_sens_k /= 512;// 2^(24-15);
    position_sens_k *= 1; // 2^(15-bits);
    position_ref_k = position_sens_k;
    position_ref_k_1n = position_sens_k;
    position_ref_k_2n = position_sens_k;
    position_k = position_sens_k;
    position_k_1n = position_sens_k;
    position_k_2n = position_sens_k;

    printstr(">>   SOMANET POSITION CONTROL SERVICE STARTING...\n");

    t :> ts;
    while(1) {
#pragma ordered
        select {
            case t when timerafter(ts + USEC_STD * pos_velocity_ctrl_config.control_loop_period) :> ts:

                upstream_control_data = i_motorcontrol.update_upstream_control_data();

                if ((upstream_control_data.position > pos_velocity_ctrl_config.int21_max_position) || (upstream_control_data.position < pos_velocity_ctrl_config.int21_min_position))
                {
                    int1_enable_flag = 0;
                    i_motorcontrol.set_torque_control_disabled();
                    i_motorcontrol.set_safe_torque_off_enabled();
                    i_motorcontrol.set_brake_status(0);
                }


                position_sens_k = ((float) upstream_control_data.position);
                position_sens_k_original = position_sens_k;
                position_sens_k /= 512;// 2^(24-15);
                position_sens_k *= 1; // 2^(15-bits);
                position_ref_in_k = ((float) position_ref_input_k);
                position_ref_in_k /= 512; // 2^(24-15);
                position_ref_in_k *= 1; // 2^(15-bits);

                int23_feedforward_effort = MAX_POS_TORQUE * sin(1.57*((position_sens_k_original-POS_OFFSET)/MAX_POS));//(float) int23_feedforward_effort_in;

                velocity_sens_k = (float) upstream_control_data.velocity;
                velocity_sens_k /= 1; // 2^(bits-16);
                velocity_sens_k *= 1; // 2^(16-bits);
                velocity_ref_in_k = ((float) velocity_ref_input_k);
                velocity_ref_in_k /= 1;
                velocity_ref_in_k *= 1;

                torque_ref_k = ((float) int23_torque_ref_in);

                if(int1_enable_flag) {
                    // position control
                    if (int1_position_enable_flag == 1) {

                        second_order_LP_filter_update(&position_k,
                                                      &position_k_1n,
                                                      &position_k_2n,
                                                      &position_sens_k, pos_velocity_ctrl_config.control_loop_period, position_SO_LP_filter_param);

                        second_order_LP_filter_update(&position_ref_k,
                                                      &position_ref_k_1n,
                                                      &position_ref_k_2n,
                                                      &position_ref_in_k, pos_velocity_ctrl_config.control_loop_period, position_ref_SO_LP_filter_param);

                        if(position_ref_k > pos_velocity_ctrl_config.int21_max_position)
                            position_ref_k = pos_velocity_ctrl_config.int21_max_position;
                        else if (position_ref_k < pos_velocity_ctrl_config.int21_min_position)
                            position_ref_k = pos_velocity_ctrl_config.int21_min_position;

//new pos controller
///*
                        position_cmd_k = new_pos_controller_updat(position_ref_k, position_k, int23_feedforward_effort, pos_velocity_ctrl_config.control_loop_period, position_control_pid_param);
                        torque_ref_k = (position_cmd_k / 512);
//*/
//new pos controller


//PID pos controller
/*
                        position_cmd_k = pid_update(position_ref_k, position_k, 0, pos_velocity_ctrl_config.control_loop_period, position_control_pid_param);
                        velocity_ref_k = (position_cmd_k / 512);
*/
//PID pos controller

                        second_order_LP_filter_shift_buffers(&position_k,
                                                             &position_k_1n,
                                                             &position_k_2n);
                        second_order_LP_filter_shift_buffers(&position_ref_k,
                                                             &position_ref_k_1n,
                                                             &position_ref_k_2n);
                    }

//PID pos controller
/*
                    // velocity control
                    if (int1_velocity_enable_flag == 1 || int1_position_enable_flag == 1) {

                        if (int1_position_enable_flag == 0) {
                        second_order_LP_filter_update(&velocity_ref_k,
                                                      &velocity_ref_k_1n,
                                                      &velocity_ref_k_2n,
                                                      &velocity_ref_in_k, pos_velocity_ctrl_config.control_loop_period, velocity_ref_SO_LP_filter_param);
                        }

                        if (velocity_ref_k > pos_velocity_ctrl_config.int21_max_speed)
                            velocity_ref_k = pos_velocity_ctrl_config.int21_max_speed;
                        else if (velocity_ref_k < -pos_velocity_ctrl_config.int21_max_speed)
                            velocity_ref_k = -pos_velocity_ctrl_config.int21_max_speed;

                        second_order_LP_filter_update(&velocity_k,
                                                      &velocity_k_1n,
                                                      &velocity_k_2n,
                                                      &velocity_sens_k, pos_velocity_ctrl_config.control_loop_period, velocity_SO_LP_filter_param);

                        // PID parameters should be int9 -> -255 to 255
                        velocity_cmd_k = pid_update(velocity_ref_k, velocity_k, int23_feedforward_effort, pos_velocity_ctrl_config.control_loop_period, velocity_control_pid_param);

                        torque_ref_k = (velocity_cmd_k / 32);

                        second_order_LP_filter_shift_buffers(&velocity_ref_k,
                                                             &velocity_ref_k_1n,
                                                             &velocity_ref_k_2n);
                        second_order_LP_filter_shift_buffers(&velocity_k,
                                                             &velocity_k_1n,
                                                             &velocity_k_2n);
                    }
//PID pos controller
*/

                    if(torque_ref_k > pos_velocity_ctrl_config.int21_max_torque)
                        torque_ref_k = pos_velocity_ctrl_config.int21_max_torque;
                    else if (torque_ref_k < (-pos_velocity_ctrl_config.int21_max_torque))
                        torque_ref_k = (-pos_velocity_ctrl_config.int21_max_torque);

                    i_motorcontrol.set_torque((int) torque_ref_k);
                }


#ifdef XSCOPE_POSITION_CTRL
                xscope_int(POSITION_REF, (int) (position_ref_k*512));
                xscope_int(POSITION, (int) (position_k*512));
                xscope_int(POSITION_CMD, (int) position_cmd_k);
                xscope_int(POSITION_TEMP1, 0);//(int) velocity_control_pid_param.Kp);
                xscope_int(VELOCITY_REF, 0);//(int) velocity_ref_k);
                xscope_int(VELOCITY, 0);//(int) velocity_k);
                xscope_int(VELOCITY_CMD, 0);//(int) velocity_cmd_k);
                xscope_int(VELOCITY_TEMP1, (int) (torque_ref_k / 1));
#endif
#ifdef XSCOPE_POSITION_CTRL_2
                xscope_int(VELOCITY, upstream_control_data.velocity);
                xscope_int(POSITION, upstream_control_data.position);
                xscope_int(TORQUE,   upstream_control_data.computed_torque);
                xscope_int(POSITION_REF, position_ref_input_k * 1);
                xscope_int(TORQUE_CMD, torque_ref_k / 1);
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
                    position_ref_input_k = upstream_control_data.position;
                    position_sens_k = ((float) upstream_control_data.position);
                    position_sens_k /= 512;// 2^(24-15);
                    position_sens_k *= 1; // 2^(15-bits);
                    position_ref_k = position_sens_k;
                    position_ref_k_1n = position_sens_k;
                    position_ref_k_2n = position_sens_k;
                    position_k = position_sens_k;
                    position_k_1n = position_sens_k;
                    position_k_2n = position_sens_k;

                    int23_feedforward_effort_in = 0;
                    pid_reset(position_control_pid_param);
                    i_motorcontrol.set_torque_control_enabled();
                    i_motorcontrol.set_brake_status(1);
                break;
            case i_position_control[int i].set_position(int in_target_position):
                    position_ref_input_k = in_target_position;
                break;
            case i_position_control[int i].set_position_pid_coefficients(int int8_Kp, int int8_Ki, int int8_Kd):
                    pid_set_parameters((float)int8_Kp, (float)int8_Ki, (float)int8_Kd, (float)pos_velocity_ctrl_config.int22_integral_limit_position, pos_velocity_ctrl_config.control_loop_period, position_control_pid_param);
                break;
            case i_position_control[int i].set_position_pid_limits(int int16_P_error_limit, int int16_I_error_limit, int int16_itegral_limit, int int21_target_max_velocity_):
                pos_velocity_ctrl_config.int21_max_speed = int21_target_max_velocity_ * 1;
//                pid_set_limits(int16_P_error_limit, int16_I_error_limit, int16_itegral_limit, pos_velocity_ctrl_config.int21_max_speed, position_control_pid_param);
                pid_set_parameters((float)position_control_pid_param.Kp, (float)position_control_pid_param.Ki, (float)position_control_pid_param.Kd, (float)int16_itegral_limit, pos_velocity_ctrl_config.control_loop_period, position_control_pid_param);
                break;
            case i_position_control[int i].set_position_limits(int position_min_limit, int position_max_limit):
                pos_velocity_ctrl_config.int21_min_position = position_min_limit / 1;
                pos_velocity_ctrl_config.int21_max_position = position_max_limit / 1;
                break;



            case i_position_control[int i].enable_velocity_ctrl():
                    int1_enable_flag = 1;
                    int1_position_enable_flag = 0;
                    int1_velocity_enable_flag = 1;
                    int1_torque_enable_flag = 0;
                    velocity_ref_input_k = 0;
                    int23_feedforward_effort_in = 0;
                    pid_reset(velocity_control_pid_param);
                    i_motorcontrol.set_torque_control_enabled();
                    i_motorcontrol.set_brake_status(1);
                break;

            case i_position_control[int i].set_velocity(int in_target_velocity):
                    velocity_ref_input_k = in_target_velocity * 1;
                break;
            case i_position_control[int i].set_offset_torque(int offset_torque_):
                    int23_feedforward_effort_in = offset_torque_;
                break;
            case i_position_control[int i].set_velocity_pid_coefficients(int int8_Kp, int int8_Ki, int int8_Kd):
                    pid_set_parameters((float)int8_Kp, (float)int8_Ki, (float)int8_Kd, (float)pos_velocity_ctrl_config.int22_integral_limit_velocity, pos_velocity_ctrl_config.control_loop_period, velocity_control_pid_param);
                break;
            case i_position_control[int i].set_velocity_pid_limits(int int16_P_error_limit, int int16_I_error_limit, int int16_itegral_limit, int int21_target_max_torque_):
                pos_velocity_ctrl_config.int21_max_torque = int21_target_max_torque_;
//                pid_set_limits(int16_P_error_limit, int16_I_error_limit, int16_itegral_limit, int21_target_max_torque_, velocity_control_pid_param);
                pid_set_parameters((float)velocity_control_pid_param.Kp, (float)velocity_control_pid_param.Ki, (float)velocity_control_pid_param.Kd, (float)int16_itegral_limit, pos_velocity_ctrl_config.control_loop_period, velocity_control_pid_param);
                break;
            case i_position_control[int i].set_velocity_limits(int velocity_min_limit, int velocity_max_limit):
                pos_velocity_ctrl_config.int21_max_speed = velocity_max_limit * 1;
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
                    pos_velocity_ctrl_config.int21_max_speed *= 1;
                    pos_velocity_ctrl_config.int21_min_position /= 1;
                    pos_velocity_ctrl_config.int21_max_position /= 1;
                    pid_init(velocity_control_pid_param);
                    pid_set_parameters((float)pos_velocity_ctrl_config.int10_P_velocity, (float)pos_velocity_ctrl_config.int10_I_velocity,
                                       (float)pos_velocity_ctrl_config.int10_D_velocity, (float)pos_velocity_ctrl_config.int22_integral_limit_velocity,
                                              pos_velocity_ctrl_config.control_loop_period, velocity_control_pid_param);
                    pid_init(position_control_pid_param);
                    pid_set_parameters((float)pos_velocity_ctrl_config.int10_P_position, (float)pos_velocity_ctrl_config.int10_I_position,
                                       (float)pos_velocity_ctrl_config.int10_D_position, (float)pos_velocity_ctrl_config.int22_integral_limit_position,
                                              pos_velocity_ctrl_config.control_loop_period, position_control_pid_param);
                    second_order_LP_filter_init(/*f_c=*//*80*/pos_velocity_ctrl_config.position_fc, /*T_s=*/pos_velocity_ctrl_config.control_loop_period, position_SO_LP_filter_param);
                    second_order_LP_filter_init(/*f_c=*//*5*/ pos_velocity_ctrl_config.position_ref_fc, /*T_s=*/pos_velocity_ctrl_config.control_loop_period, position_ref_SO_LP_filter_param);
                    second_order_LP_filter_init(/*f_c=*//*25*/pos_velocity_ctrl_config.velocity_ref_fc, /*T_s=*/pos_velocity_ctrl_config.control_loop_period, velocity_ref_SO_LP_filter_param);
                    second_order_LP_filter_init(/*f_c=*//*80*/pos_velocity_ctrl_config.velocity_fc, /*T_s=*/pos_velocity_ctrl_config.control_loop_period, velocity_SO_LP_filter_param);
                    second_order_LP_filter_init(/*f_c=*//*75*/pos_velocity_ctrl_config.velocity_d_fc, /*T_s=*/pos_velocity_ctrl_config.control_loop_period, velocity_d_SO_LP_filter_param);
                break;

            case i_position_control[int i].get_position_velocity_control_config() ->  PosVelocityControlConfig out_config:
                    out_config = pos_velocity_ctrl_config;
                    out_config.int21_max_speed /= 1;
                    out_config.int21_min_position *= 1;
                    out_config.int21_max_position *= 1;
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
                    position_ref_input_k = downstream_control_data.position_cmd / 1;
                    velocity_ref_input_k = downstream_control_data.velocity_cmd * 1;
                    int23_torque_ref_in = downstream_control_data.torque_cmd;
                    int23_feedforward_effort_in = downstream_control_data.offset_torque;
                break;


        }
    }
}
