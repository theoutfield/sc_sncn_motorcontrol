/**
 * @file  position_ctrl_server.xc
 * @brief Position Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/
#include <xs1.h>
#include <xscope.h>
#include <print.h>
#include <stdlib.h>

#include <math.h>

#include <controllers_lib.h>
#include <filters_lib.h>

#include <position_ctrl_service.h>
#include <refclk.h>
#include <mc_internal_constants.h>
#include <filters_lib.h>
#include <stdio.h>


int special_brake_release(int &counter, int start_position, int actual_position, int range, int duration, int max_torque)
{
    const int steps = 8;
    int target;
    if ( (actual_position-start_position) > range || (actual_position-start_position) < (-range)) //we moved more than half the range so the brake should be released
    {
        target = 0;
        counter = duration; //stop counter
    }
    else if (counter < duration)
    {
        int step = counter/(duration/steps);
        int sign = 1;
        if (step%2) {
            sign = -1;
        }
        target = ((counter-(duration/steps)*step)*max_torque*(step+1)*sign)/(duration); //ramp to max torque of step
    }
    else if (counter == duration) //end:
    {
        target = 0; //stop
    }
    counter++;

    return target;
}


void position_velocity_control_service(PosVelocityControlConfig &pos_velocity_ctrl_config,
                              interface MotorcontrolInterface client i_motorcontrol,
                              interface PositionVelocityCtrlInterface server i_position_control[3])
{

    UpstreamControlData upstream_control_data;
    DownstreamControlData downstream_control_data;
    int pos_control_mode = 0;
    int velocity_control_mode = 0;

    NonlinearPositionControl nl_pos_ctrl;
    nl_position_control_reset(nl_pos_ctrl);
    nl_position_control_set_parameters(nl_pos_ctrl, pos_velocity_ctrl_config);

    int position_ref_input_k_ = 0;
    double position_ref_k_ = 0.00;
    double position_sens_k_ = 0.00, position_sens_k_1_=0.00;

    unsigned int /*ts_=0, */t_old_=0, t_new_=0, t_end_=0, idle_time_=0, loop_time_=0;
    ///////////////////////////////////////////////

    int position_enable_flag = 0;
    int motorctrl_enable_flag = 0;
    int motorctrl_enable_timeout = 0;
    int position_limit_reached = 0;
    int max_position, min_position;
    PIDparam position_control_pid_param;
    SecondOrderLPfilterParam position_SO_LP_filter_param;
    double position_k = 0;
    double position_sens_k = 0;
    double position_k_1n = 0;
    double position_k_2n = 0;
    double position_ref_k = 0;
    double position_cmd_k = 0;
    int position_ref_input_k = 0;
    double position_ref_in_k = 0;
    double position_ref_in_k_1n = 0;
    double position_ref_in_k_2n = 0;

    // velocity controller
    int velocity_enable_flag = 0;
    PIDparam velocity_control_pid_param;
    SecondOrderLPfilterParam velocity_SO_LP_filter_param;
    int velocity_ref_input_k = 0;
    double additive_torque_k = 0;
    int additive_torque_input_k = 0;
    double velocity_ref_k = 0;
    double velocity_sens_k = 0;
    double velocity_k = 0;
    double velocity_k_1n = 0;
    double velocity_k_2n = 0;
    double velocity_cmd_k = 0;

    // torque
    int torque_enable_flag = 0;
    int torque_ref_input_k = 0;
    double torque_ref_k = 0;

    //pos profiler
    posProfilerParam pos_profiler_param;
    pos_profiler_param.delta_T = ((float)pos_velocity_ctrl_config.control_loop_period)/1000000;
    pos_profiler_param.a_max = ((float) pos_velocity_ctrl_config.max_acceleration_profiler);
    pos_profiler_param.v_max = ((float) pos_velocity_ctrl_config.max_speed_profiler);
    float acceleration_monitor = 0;
    int enable_profiler = 1;

    //special_brake_release
    const int special_brake_release_range = 1000;
    const int special_brake_release_duration = 2000;
    int special_brake_release_counter = special_brake_release_duration+1;
    int special_brake_release_initial_position = 0;
    int special_brake_release_torque = 0;

    timer t;
    unsigned int ts;

    second_order_LP_filter_init(pos_velocity_ctrl_config.position_fc, pos_velocity_ctrl_config.control_loop_period, position_SO_LP_filter_param);
    second_order_LP_filter_init(pos_velocity_ctrl_config.velocity_fc, pos_velocity_ctrl_config.control_loop_period, velocity_SO_LP_filter_param);

    pid_init(velocity_control_pid_param);
    pid_set_parameters((double)pos_velocity_ctrl_config.P_velocity, (double)pos_velocity_ctrl_config.I_velocity,
                       (double)pos_velocity_ctrl_config.D_velocity, (double)pos_velocity_ctrl_config.integral_limit_velocity,
                              pos_velocity_ctrl_config.control_loop_period, velocity_control_pid_param);
    pid_init(position_control_pid_param);
    pid_set_parameters((double)pos_velocity_ctrl_config.P_pos, (double)pos_velocity_ctrl_config.I_pos,
                       (double)pos_velocity_ctrl_config.D_pos, (double)pos_velocity_ctrl_config.integral_limit_pos,
                              pos_velocity_ctrl_config.control_loop_period, position_control_pid_param);

    //reverse position limits when polarity is inverted
    if (pos_velocity_ctrl_config.polarity == -1) {
        min_position = -pos_velocity_ctrl_config.max_pos;
        max_position = -pos_velocity_ctrl_config.min_pos;
    } else {
        min_position = pos_velocity_ctrl_config.min_pos;
        max_position = pos_velocity_ctrl_config.max_pos;
    }

    downstream_control_data.position_cmd = 0;
    downstream_control_data.velocity_cmd = 0;
    downstream_control_data.torque_cmd = 0;
    downstream_control_data.offset_torque = 0;

    delay_milliseconds(500);
    upstream_control_data = i_motorcontrol.update_upstream_control_data();
    position_ref_input_k = upstream_control_data.position;
    position_ref_in_k = (float) position_ref_input_k;
    position_ref_in_k_1n = (float) position_ref_input_k;
    position_ref_in_k_2n = (float) position_ref_input_k;
    position_sens_k = ((float) upstream_control_data.position);
    position_sens_k /= 512;// 2^(24-15);
    position_k = position_sens_k;
    position_k_1n = position_sens_k;
    position_k_2n = position_sens_k;

    printstr(">>   SOMANET POSITION CONTROL SERVICE STARTING...\n");

    t :> ts;
    while(1)
    {
#pragma ordered
        select
        {
            case t when timerafter(ts + USEC_STD * pos_velocity_ctrl_config.control_loop_period) :> ts:

                t_old_=t_new_;
                t :> t_new_;
                loop_time_=t_new_-t_old_;
                idle_time_=t_new_-t_end_;
                motorctrl_enable_timeout--;

                upstream_control_data = i_motorcontrol.update_upstream_control_data();

                /*
                 * CONTELEC:    16bits single turn + 12bits multi turn
                 * BISS-MABI:   18bits signle turn + 10bits multi turn
                 * AMS:         14bits single turn + 18bits multi turn
                 * Other sens:  13bits single turn + 12bits multi turn
                 */



                if (enable_profiler)
                {
                    position_ref_in_k = pos_profiler(((float) position_ref_input_k), position_ref_in_k_1n, position_ref_in_k_2n, pos_profiler_param);
                    acceleration_monitor = (position_ref_in_k - (2 * position_ref_in_k_1n) + position_ref_in_k_2n)/(pos_velocity_ctrl_config.control_loop_period * pos_velocity_ctrl_config.control_loop_period);
                    position_ref_in_k_2n = position_ref_in_k_1n;
                    position_ref_in_k_1n = position_ref_in_k;
                }
                else
                {
                    position_ref_in_k = (float) position_ref_input_k;
                }

//                position_ref_k = position_ref_in_k / 512;

//                position_sens_k = ((float) upstream_control_data.position);
//                position_sens_k /= 512;

                additive_torque_k = ((double) additive_torque_input_k);

                velocity_ref_k = ((double) velocity_ref_input_k);

                velocity_sens_k = (double) upstream_control_data.velocity;

                torque_ref_k = ((double) torque_ref_input_k);

                if(torque_enable_flag)
                {
                    // position control
                    if (position_enable_flag == 1)
                    {

//                        second_order_LP_filter_update(&position_k,
//                                                      &position_k_1n,
//                                                      &position_k_2n,
//                                                      &position_sens_k, pos_velocity_ctrl_config.control_loop_period, position_SO_LP_filter_param);

                        //************************************************

                        position_ref_input_k_ =  position_ref_in_k;//downstream_control_data.position_cmd;

                        position_ref_k_ = (double) (position_ref_input_k_);

                        position_sens_k_1_ = position_sens_k_;
                        position_sens_k_   = (double) (upstream_control_data.position);


                        if (pos_control_mode == POS_PID_CONTROLLER)
                        {
                            position_cmd_k = pid_update(position_ref_k_, position_sens_k_, pos_velocity_ctrl_config.control_loop_period, position_control_pid_param);
                            torque_ref_k = position_cmd_k ;
                        }
                        else if (pos_control_mode == POS_PID_VELOCITY_CASCADED_CONTROLLER)
                        {
                            position_cmd_k = pos_cascade_controller(
                                    position_ref_k_, position_sens_k_,
                                    pos_velocity_ctrl_config.control_loop_period, position_control_pid_param,
                                    velocity_sens_k);
                            torque_ref_k = position_cmd_k ;
                        }
                        else if (pos_control_mode == NL_POSITION_CONTROLLER)
                        {



                            // apply position control algorithm
                            torque_ref_k = update_nl_position_control(nl_pos_ctrl, position_ref_k_, position_sens_k_1_, position_sens_k_);
                        }

//                        second_order_LP_filter_shift_buffers(&position_k,
//                                                             &position_k_1n,
//                                                             &position_k_2n);
                    }

                    // velocity control
                    if (velocity_enable_flag == 1)
                    {
                        if (velocity_ref_k > pos_velocity_ctrl_config.max_speed)
                            velocity_ref_k = pos_velocity_ctrl_config.max_speed;
                        else if (velocity_ref_k < -pos_velocity_ctrl_config.max_speed)
                            velocity_ref_k = -pos_velocity_ctrl_config.max_speed;

//                        second_order_LP_filter_update(&velocity_k,
//                                                      &velocity_k_1n,
//                                                      &velocity_k_2n,
//                                                      &velocity_sens_k, pos_velocity_ctrl_config.control_loop_period, velocity_SO_LP_filter_param);

                        if (velocity_control_mode == VELOCITY_PID_CONTROLLER)
                        {
                            velocity_cmd_k = velocity_controller(velocity_ref_k, velocity_sens_k, velocity_control_pid_param);
                            torque_ref_k = velocity_cmd_k;
                        }

//                        second_order_LP_filter_shift_buffers(&velocity_k,
//                                                             &velocity_k_1n,
//                                                             &velocity_k_2n);
                    }

                    torque_ref_k += additive_torque_k;

                    //brake release
                    if (special_brake_release_counter <= special_brake_release_duration) //change target torque if we are in special brake release
                    {
                        torque_ref_k = special_brake_release(special_brake_release_counter, special_brake_release_initial_position, upstream_control_data.position,\
                                special_brake_release_range, special_brake_release_duration, special_brake_release_torque);
                    }

                    //torque limit check
                    if(torque_ref_k > pos_velocity_ctrl_config.max_torque)
                        torque_ref_k = pos_velocity_ctrl_config.max_torque;
                    else if (torque_ref_k < (-pos_velocity_ctrl_config.max_torque))
                        torque_ref_k = (-pos_velocity_ctrl_config.max_torque);

                    //position limit check
                    if (upstream_control_data.position > max_position)
                    {
                        if (((int)torque_ref_k) >= 0 && motorctrl_enable_flag == 1)
                        {
                            motorctrl_enable_flag = 0;
                            motorctrl_enable_timeout = 3000000/pos_velocity_ctrl_config.control_loop_period; //wait 3 seconds before enabling the motor again
                            position_limit_reached = 1;
                            i_motorcontrol.set_brake_status(0);
                            i_motorcontrol.set_torque_control_disabled();
                            i_motorcontrol.set_safe_torque_off_enabled();
                            printstr("*** Maximum Position Limit Reached ***\n");
                        }
                        else if (((int)torque_ref_k) < 0 && motorctrl_enable_flag == 0 && motorctrl_enable_timeout < 0 && position_limit_reached == 0)
                        {
                            motorctrl_enable_flag = 1;
                            i_motorcontrol.set_torque_control_enabled();
                            i_motorcontrol.set_brake_status(1);
                        }
                    }
                    else if (upstream_control_data.position < min_position)
                    {
                        if (((int)torque_ref_k) <= 0 && motorctrl_enable_flag == 1)
                        {
                            motorctrl_enable_flag = 0;
                            motorctrl_enable_timeout = 3000000/pos_velocity_ctrl_config.control_loop_period; //wait 3 seconds before enabling the motor again
                            position_limit_reached = 1;
                            i_motorcontrol.set_brake_status(0);
                            i_motorcontrol.set_torque_control_disabled();
                            i_motorcontrol.set_safe_torque_off_enabled();
                            printstr("*** Minimum Position Limit Reached ***\n");
                        }
                        else if (((int)torque_ref_k) > 0 && motorctrl_enable_flag == 0 && motorctrl_enable_timeout < 0 && position_limit_reached == 0)
                        {
                            motorctrl_enable_flag = 1;
                            i_motorcontrol.set_torque_control_enabled();
                            i_motorcontrol.set_brake_status(1);
                        }
                    }
                    else if (motorctrl_enable_flag == 0 && motorctrl_enable_timeout < 0 && position_limit_reached == 0)
                    {
                        motorctrl_enable_flag = 1;
                        i_motorcontrol.set_torque_control_enabled();
                        i_motorcontrol.set_brake_status(1);
                    }

                    i_motorcontrol.set_torque((int) torque_ref_k);
                }

#ifdef XSCOPE_POSITION_CTRL
//                xscope_int(VELOCITY, velocity_sens_k);
//                xscope_int(POSITION, position_sens_k_);
//                xscope_int(TORQUE,   upstream_control_data.computed_torque);
//                xscope_int(ANGEL, upstream_control_data.angle);
//                xscope_int(POSITION_CMD, position_ref_k_);
//                xscope_int(VELOCITY_CMD, velocity_ref_k);
//                xscope_int(TORQUE_CMD, (int)torque_ref_k);
#endif

                t :> t_end_;
                break;

            case i_position_control[int i].disable():
                motorctrl_enable_flag = 0;
                torque_enable_flag = 0;
                position_enable_flag = 0;
                velocity_enable_flag = 0;
                i_motorcontrol.set_torque_control_disabled();
                i_motorcontrol.set_safe_torque_off_enabled();
                i_motorcontrol.set_brake_status(0);
                break;

            case i_position_control[int i].enable_position_ctrl(int pos_control_mode_):
                pos_control_mode = pos_control_mode_;
                torque_enable_flag = 1;
                motorctrl_enable_flag = 1;
                position_enable_flag = 1;
//                if (pos_control_mode == POS_PID_VELOCITY_CASCADED_CONTROLLER)
//                {
//                    velocity_enable_flag = 1;
//                    velocity_control_mode = VELOCITY_PID_CONTROLLER;
//                }
//                else
//                {
//                    velocity_enable_flag = 0;
//                    velocity_control_mode = VELOCITY_PID_CONTROLLER;
//                }

                //nonlinear position control
                position_ref_input_k_ = 0;
                position_ref_k_ = 0;
                position_sens_k_ = (double) (upstream_control_data.position);
                nl_pos_ctrl.torque_ref_k = 0.00;
                nl_pos_ctrl.t_additive = 0.00;
                nl_pos_ctrl.feedback_p_loop =0.00;
                nl_pos_ctrl.feedback_d_loop=0.00;
                nl_pos_ctrl.t_additive = 0.00;
                //////////////////////////////

                position_ref_input_k = upstream_control_data.position;
                downstream_control_data.position_cmd = upstream_control_data.position;
                position_ref_in_k = (float) position_ref_input_k;
                position_ref_in_k_1n = (float) position_ref_input_k;
                position_ref_in_k_2n = (float) position_ref_input_k;
                position_sens_k = ((float) upstream_control_data.position);
                position_sens_k /= 512;// 2^(24-15);
                position_k = position_sens_k;
                position_k_1n = position_sens_k;
                position_k_2n = position_sens_k;
                additive_torque_input_k = 0;
                pid_reset(position_control_pid_param);
                pid_reset(velocity_control_pid_param);
                i_motorcontrol.set_torque_control_enabled();
                i_motorcontrol.set_brake_status(1);
                //special brake release
                if (pos_velocity_ctrl_config.special_brake_release != 0)
                {
                    special_brake_release_counter = 0;
                    special_brake_release_initial_position = upstream_control_data.position;
                    special_brake_release_torque = (pos_velocity_ctrl_config.special_brake_release*pos_velocity_ctrl_config.max_torque)/100;
                }
                enable_profiler = pos_velocity_ctrl_config.enable_profiler;
                break;

            case i_position_control[int i].enable_velocity_ctrl(int velocity_control_mode_):
                velocity_control_mode = velocity_control_mode_;
                torque_enable_flag = 1;
                motorctrl_enable_flag = 1;
                position_enable_flag = 0;
                velocity_enable_flag = 1;
                velocity_ref_input_k = 0;
                additive_torque_input_k = 0;
                pid_reset(velocity_control_pid_param);
                i_motorcontrol.set_torque_control_enabled();
                i_motorcontrol.set_brake_status(1);
                //special brake release
                if (pos_velocity_ctrl_config.special_brake_release != 0)
                {
                    special_brake_release_counter = 0;
                    special_brake_release_initial_position = upstream_control_data.position;
                    special_brake_release_torque = (pos_velocity_ctrl_config.special_brake_release*pos_velocity_ctrl_config.max_torque)/100;
                }
                break;

            case i_position_control[int i].enable_torque_ctrl():
                torque_enable_flag = 1;
                motorctrl_enable_flag = 1;
                position_enable_flag = 0;
                velocity_enable_flag = 0;
                torque_ref_input_k = 0;
                i_motorcontrol.set_torque_control_enabled();
                i_motorcontrol.set_brake_status(1);
                //special brake release
                if (pos_velocity_ctrl_config.special_brake_release != 0)
                {
                    special_brake_release_counter = 0;
                    special_brake_release_initial_position = upstream_control_data.position;
                    special_brake_release_torque = (pos_velocity_ctrl_config.special_brake_release*pos_velocity_ctrl_config.max_torque)/100;
                }
                break;

            case i_position_control[int i].set_position_velocity_control_config(PosVelocityControlConfig in_config):
                pos_velocity_ctrl_config = in_config;
                //reverse position limits when polarity is inverted
                if (pos_velocity_ctrl_config.polarity == -1) {
                    min_position = -pos_velocity_ctrl_config.max_pos;
                    max_position = -pos_velocity_ctrl_config.min_pos;
                } else {
                    min_position = pos_velocity_ctrl_config.min_pos;
                    max_position = pos_velocity_ctrl_config.max_pos;
                }
                //pid_init(velocity_control_pid_param);
                pid_set_parameters((double)pos_velocity_ctrl_config.P_velocity, (double)pos_velocity_ctrl_config.I_velocity,
                                   (double)pos_velocity_ctrl_config.D_velocity, (double)pos_velocity_ctrl_config.integral_limit_velocity,
                                          pos_velocity_ctrl_config.control_loop_period, velocity_control_pid_param);
                //pid_init(position_control_pid_param);
                if(pos_velocity_ctrl_config.P_pos<0)            pos_velocity_ctrl_config.P_pos=0;
                if(pos_velocity_ctrl_config.P_pos>100000000)    pos_velocity_ctrl_config.P_pos=100000000;
                if(pos_velocity_ctrl_config.I_pos<0)            pos_velocity_ctrl_config.I_pos=0;
                if(pos_velocity_ctrl_config.I_pos>100000000)    pos_velocity_ctrl_config.I_pos=100000000;
                if(pos_velocity_ctrl_config.D_pos<0)            pos_velocity_ctrl_config.D_pos=0;
                if(pos_velocity_ctrl_config.D_pos>100000000)    pos_velocity_ctrl_config.D_pos=100000000;

                pid_set_parameters((double)pos_velocity_ctrl_config.P_pos, (double)pos_velocity_ctrl_config.I_pos,
                                   (double)pos_velocity_ctrl_config.D_pos, (double)pos_velocity_ctrl_config.integral_limit_pos,
                                          pos_velocity_ctrl_config.control_loop_period, position_control_pid_param);

                second_order_LP_filter_init(pos_velocity_ctrl_config.position_fc, pos_velocity_ctrl_config.control_loop_period, position_SO_LP_filter_param);
                second_order_LP_filter_init(pos_velocity_ctrl_config.velocity_fc, pos_velocity_ctrl_config.control_loop_period, velocity_SO_LP_filter_param);
                pos_profiler_param.a_max = ((float) pos_velocity_ctrl_config.max_acceleration_profiler);
                pos_profiler_param.v_max = ((float) pos_velocity_ctrl_config.max_speed_profiler);
                enable_profiler = pos_velocity_ctrl_config.enable_profiler;

                nl_position_control_reset(nl_pos_ctrl);
                nl_position_control_set_parameters(nl_pos_ctrl, pos_velocity_ctrl_config);
                break;

            case i_position_control[int i].get_position_velocity_control_config() ->  PosVelocityControlConfig out_config:
                out_config = pos_velocity_ctrl_config;
                break;


            case i_position_control[int i].update_control_data(DownstreamControlData downstream_control_data_in) -> UpstreamControlData upstream_control_data_out:
                //reset position limiter if a new position target is received
                if (position_limit_reached == 1) {
                    if (downstream_control_data_in.position_cmd != downstream_control_data.position_cmd ||
                        downstream_control_data_in.velocity_cmd != downstream_control_data.velocity_cmd ||
                        downstream_control_data_in.torque_cmd != downstream_control_data.torque_cmd  ) {
                        position_limit_reached = 0;
                    }
                }
                //send the actual position/velocity/torque upstream
                upstream_control_data_out = upstream_control_data;
                //receive position/velocity/torque commands
                downstream_control_data = downstream_control_data_in;
                //reverse position/velocity feedback/commands when polarity is inverted
                if (pos_velocity_ctrl_config.polarity == -1) {
                    upstream_control_data_out.position = -upstream_control_data_out.position;
                    upstream_control_data_out.velocity = -upstream_control_data_out.velocity;
                    downstream_control_data.position_cmd = -downstream_control_data.position_cmd;
                    downstream_control_data.velocity_cmd = -downstream_control_data.velocity_cmd;
                }
                //apply limits
                if (downstream_control_data.position_cmd > max_position) {
                    downstream_control_data.position_cmd = max_position;
                } else if (downstream_control_data.position_cmd < min_position) {
                    downstream_control_data.position_cmd = min_position;
                }
                position_ref_input_k = downstream_control_data.position_cmd;
                velocity_ref_input_k = downstream_control_data.velocity_cmd;
                torque_ref_input_k = downstream_control_data.torque_cmd;
                additive_torque_input_k = downstream_control_data.offset_torque;
                break;


            case i_position_control[int i].set_j(int j):
                    pos_velocity_ctrl_config.j = j;
                    nl_position_control_set_parameters(nl_pos_ctrl, pos_velocity_ctrl_config);
                    break;

            case i_position_control[int i].set_torque(int in_target_torque):
                torque_ref_input_k = in_target_torque;
                break;


            case i_position_control[int i].get_position() -> int out_position:
                    if (pos_velocity_ctrl_config.polarity == -1)
                        out_position = -upstream_control_data.position;
                    else
                        out_position = upstream_control_data.position;
                break;


            case i_position_control[int i].get_velocity() -> int out_velocity:
                    if (pos_velocity_ctrl_config.polarity == -1)
                        out_velocity = -upstream_control_data.velocity;
                    else
                        out_velocity = upstream_control_data.velocity;
                break;


            case i_position_control[int i].get_motorcontrol_config() -> MotorcontrolConfig out_motorcontrol_config:
                out_motorcontrol_config = i_motorcontrol.get_config();
                break;

            case i_position_control[int i].set_motorcontrol_config(MotorcontrolConfig in_motorcontrol_config):
                motorctrl_enable_flag = 0;
                torque_enable_flag = 0;
                position_enable_flag = 0;
                velocity_enable_flag = 0;
                i_motorcontrol.set_config(in_motorcontrol_config);
                break;

            case i_position_control[int i].set_brake_status(int in_brake_status):
                i_motorcontrol.set_brake_status(in_brake_status);
                break;

            case i_position_control[int i].set_offset_detection_enabled() -> MotorcontrolConfig out_motorcontrol_config:
                //offset detection
                out_motorcontrol_config = i_motorcontrol.get_config();
                out_motorcontrol_config.commutation_angle_offset = -1;
                i_motorcontrol.set_offset_detection_enabled();
                while(out_motorcontrol_config.commutation_angle_offset == -1)
                {
                    out_motorcontrol_config = i_motorcontrol.get_config();
                    out_motorcontrol_config.commutation_angle_offset = i_motorcontrol.set_calib(0);
                    delay_milliseconds(50);//wait until offset is detected
                }

                //check polarity state
                if(i_motorcontrol.get_sensor_polarity_state() != 1)
                {
                    out_motorcontrol_config.commutation_angle_offset = -1;
                }
                //write offset in config
                i_motorcontrol.set_config(out_motorcontrol_config);
                motorctrl_enable_flag = 0;
                torque_enable_flag = 0;
                position_enable_flag = 0;
                velocity_enable_flag = 0;
                break;
        }
    }
}
