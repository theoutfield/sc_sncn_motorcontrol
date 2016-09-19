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



//void init_position_velocity_control(interface PositionVelocityCtrlInterface client i_position_control)
//{
//    int ctrl_state;
//
//    while (1) {
//        ctrl_state = i_position_control.check_busy();
//        if (ctrl_state == INIT_BUSY) {
//            i_position_control.enable_position_ctrl();
//        }
//
//        if (ctrl_state == INIT) {
//#ifdef debug_print
//            printstrln("position_ctrl_service: position control initialized");
//#endif
//            break;
//        }
//    }
//}


void position_velocity_control_service(PosVelocityControlConfig &pos_velocity_ctrl_config,
                              interface MotorcontrolInterface client i_motorcontrol,
                              interface PositionVelocityCtrlInterface server i_position_control[3])
{

    UpstreamControlData upstream_control_data;
    DownstreamControlData downstream_control_data;
    int pos_control_mode = 0;
    int velocity_control_mode = 0;

    ///////////////////////////////////////////////
    // position controller

    PositionControlWithSaturation pos_ctrl_with_saturation;

    position_control_with_saturation_reset(pos_ctrl_with_saturation);




    //************************************************
    // set parameters of position controller structure
    pos_ctrl_with_saturation.w_max = (((double)(pos_velocity_ctrl_config.max_speed))*2.00*3.1415)/60;
    pos_ctrl_with_saturation.k_fb = pos_velocity_ctrl_config.k_fb;
    pos_ctrl_with_saturation.k_m  = 0.001;

    //1ms
    pos_ctrl_with_saturation.kp =  9895.00;
    pos_ctrl_with_saturation.ki =  1001.00;
    pos_ctrl_with_saturation.kd =  41421.00;

    ////500us
    //pos_ctrl_with_saturation.kp = 39580.00 ;
    //pos_ctrl_with_saturation.ki = 4003.00  ;
    //pos_ctrl_with_saturation.kd = 165685.00;

    pos_ctrl_with_saturation.ts_position = ((double)(pos_velocity_ctrl_config.control_loop_period))/1000000.00; //s


    pos_ctrl_with_saturation.j   = 100.00; // in micro-kgm2
    pos_ctrl_with_saturation.kp *= pos_ctrl_with_saturation.j;
    pos_ctrl_with_saturation.ki *= pos_ctrl_with_saturation.j;
    pos_ctrl_with_saturation.kd *= pos_ctrl_with_saturation.j;
    pos_ctrl_with_saturation.kp /=1000000.00;
    pos_ctrl_with_saturation.ki /=1000000.00;
    pos_ctrl_with_saturation.kd /=1000000.00;

    pos_ctrl_with_saturation.kp *= (pos_ctrl_with_saturation.gain_p);
    pos_ctrl_with_saturation.kp /= 1000.00;
    pos_ctrl_with_saturation.ki *= (pos_ctrl_with_saturation.gain_i);
    pos_ctrl_with_saturation.ki /= 1000.00;
    pos_ctrl_with_saturation.kd *= (pos_ctrl_with_saturation.gain_d);
    pos_ctrl_with_saturation.kd /= 1000.00;

    pos_ctrl_with_saturation.t_max=((double)(pos_velocity_ctrl_config.max_torque));









    int position_enable_flag_ = 0;
    int torque_enable_flag_ = 0;

    int position_ref_input_k_ = 0;
    int initial_position_=0;
    double position_ref_k_ = 0.00;
    double position_sens_k_ = 0.00, position_sens_k_1_=0.00;

    int min_s_index_=0;

    unsigned int ts_=0, t_old_=0, t_new_=0, t_end_=0, idle_time_=0, loop_time_=0;
    ///////////////////////////////////////////////

    int position_enable_flag = 0;
    PIDparam position_control_pid_param;
    integralOptimumPosControllerParam integral_optimum_pos_ctrl_pid_param;
    SecondOrderLPfilterParam position_SO_LP_filter_param;
    float position_k = 0;
    float position_sens_k = 0;
    float position_k_1n = 0;
    float position_k_2n = 0;
    float position_ref_k = 0;
    float position_cmd_k = 0;
    float min_position = 0;
    float max_position = 0;
    int position_ref_input_k = 0;
    float position_ref_in_k = 0;
    float position_ref_in_k_1n = 0;
    float position_ref_in_k_2n = 0;

    // velocity controller
    int velocity_enable_flag = 0;
    PIDparam velocity_control_pid_param;
    SecondOrderLPfilterParam velocity_SO_LP_filter_param;
    int velocity_ref_input_k = 0;
    float additive_torque_k = 0;
    int additive_torque_input_k = 0;
    float velocity_ref_in_k = 0;
    float velocity_ref_k = 0;
    float velocity_sens_k = 0;
    float velocity_k = 0;
    float velocity_k_1n = 0;
    float velocity_k_2n = 0;
    float velocity_cmd_k = 0;

    // torque
    int torque_enable_flag = 0;
    int torque_ref_input_k = 0;
    float torque_ref_k = 0;

    //pos profiler
    posProfilerParam pos_profiler_param;
    pos_profiler_param.delta_T = ((float)pos_velocity_ctrl_config.control_loop_period)/1000000;
    pos_profiler_param.a_max = ((float) pos_velocity_ctrl_config.max_acceleration_profiler);
    pos_profiler_param.v_max = ((float) pos_velocity_ctrl_config.max_speed_profiler);
    float acceleration_monitor = 0;
    int enable_profiler = 1;

    timer t;
    unsigned int ts;

    second_order_LP_filter_init(pos_velocity_ctrl_config.position_fc, pos_velocity_ctrl_config.control_loop_period, position_SO_LP_filter_param);
    second_order_LP_filter_init(pos_velocity_ctrl_config.velocity_fc, pos_velocity_ctrl_config.control_loop_period, velocity_SO_LP_filter_param);

    pid_init(velocity_control_pid_param);
    pid_set_parameters((float)pos_velocity_ctrl_config.P_velocity, (float)pos_velocity_ctrl_config.I_velocity,
                       (float)pos_velocity_ctrl_config.D_velocity, (float)pos_velocity_ctrl_config.integral_limit_velocity,
                              pos_velocity_ctrl_config.control_loop_period, velocity_control_pid_param);
    pid_init(position_control_pid_param);
    pid_set_parameters((float)pos_velocity_ctrl_config.P_pos, (float)pos_velocity_ctrl_config.I_pos,
                       (float)pos_velocity_ctrl_config.D_pos, (float)pos_velocity_ctrl_config.integral_limit_pos,
                              pos_velocity_ctrl_config.control_loop_period, position_control_pid_param);
    integral_optimum_pos_controller_init(integral_optimum_pos_ctrl_pid_param);
    integral_optimum_pos_controller_set_parameters((float)pos_velocity_ctrl_config.P_pos_Integral_optimum, (float)pos_velocity_ctrl_config.I_pos_Integral_optimum,
                                                   (float)pos_velocity_ctrl_config.D_pos_Integral_optimum, (float)pos_velocity_ctrl_config.integral_limit_pos_Integral_optimum,
                                                    pos_velocity_ctrl_config.control_loop_period, integral_optimum_pos_ctrl_pid_param);


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

    max_position = ((float) pos_velocity_ctrl_config.max_pos);
    max_position /= 512;
    min_position = ((float) pos_velocity_ctrl_config.min_pos);
    min_position /= 512;

    printstr(">>   SOMANET POSITION CONTROL SERVICE STARTING...\n");

    t :> ts;
    while(1) {
#pragma ordered
        select {
            case t when timerafter(ts + USEC_STD * pos_velocity_ctrl_config.control_loop_period) :> ts:

                t_old_=t_new_;
                t :> t_new_;
                loop_time_=t_new_-t_old_;
                idle_time_=t_new_-t_end_;

                upstream_control_data = i_motorcontrol.update_upstream_control_data();

                /*
                 * CONTELEC:    16bits single turn + 12bits multi turn
                 * BISS-MABI:   18bits signle turn + 10bits multi turn
                 * AMS:         14bits single turn + 18bits multi turn
                 * Other sens:  13bits single turn + 12bits multi turn
                 */


                if (enable_profiler) {
                    position_ref_in_k = pos_profiler(((float) position_ref_input_k), position_ref_in_k_1n, position_ref_in_k_2n, pos_profiler_param);
                    acceleration_monitor = (position_ref_in_k - (2 * position_ref_in_k_1n) + position_ref_in_k_2n)/(pos_velocity_ctrl_config.control_loop_period * pos_velocity_ctrl_config.control_loop_period);
                    position_ref_in_k_2n = position_ref_in_k_1n;
                    position_ref_in_k_1n = position_ref_in_k;
                }
                else
                    position_ref_in_k = (float) position_ref_input_k;

                position_ref_k = position_ref_in_k / 512;

                position_sens_k = ((float) upstream_control_data.position);
                position_sens_k /= 512;

                additive_torque_k = ((float) additive_torque_input_k);

                velocity_ref_k = ((float) velocity_ref_input_k);

                velocity_sens_k = (float) upstream_control_data.velocity;

                torque_ref_k = ((float) torque_ref_input_k);

                if(torque_enable_flag) {

                    if ((position_sens_k > max_position) || (position_sens_k < min_position))
                    {
                        torque_enable_flag = 0;
                        position_enable_flag = 0;
                        velocity_enable_flag = 0;
                        i_motorcontrol.set_torque_control_disabled();
                        i_motorcontrol.set_safe_torque_off_enabled();
                        i_motorcontrol.set_brake_status(0);
                        printstr("*** Position Limit Reached ***\n");
                    }


                    // position control
                    if (position_enable_flag == 1)
                    {

                        if(position_ref_k > max_position)
                            position_ref_k = max_position;
                        else if (position_ref_k < min_position)
                            position_ref_k = min_position;

                        second_order_LP_filter_update(&position_k,
                                                      &position_k_1n,
                                                      &position_k_2n,
                                                      &position_sens_k, pos_velocity_ctrl_config.control_loop_period, position_SO_LP_filter_param);

                        if (pos_control_mode == POS_PID_CONTROLLER)
                        {
                            position_cmd_k = pid_update(position_ref_k, position_k, pos_velocity_ctrl_config.control_loop_period, position_control_pid_param);
                            torque_ref_k = (position_cmd_k / 512);
                        }
                        else if (pos_control_mode == POS_PID_VELOCITY_CASCADED_CONTROLLER)
                        {
                            position_cmd_k = pid_update(position_ref_k, position_k, pos_velocity_ctrl_config.control_loop_period, position_control_pid_param);
                            velocity_ref_k = (position_cmd_k / 512);
                        }
                        else if (pos_control_mode == POS_WITH_SATURATION_CONTROLLER)
                        {

                            //************************************************
                            // update feedback data
                            upstream_control_data = i_motorcontrol.update_upstream_control_data();

                            position_ref_input_k_ = initial_position_ + downstream_control_data.position_cmd;

                            position_ref_k_ = (double) (position_ref_input_k_);

                            position_sens_k_1_ = position_sens_k_;
                            position_sens_k_   = (double) (upstream_control_data.position);


                            // apply position control algorithm

                            pos_ctrl_with_saturation.gained_error = position_ref_k_ - position_sens_k_;

                            pos_ctrl_with_saturation.feedback_p_loop  = pos_ctrl_with_saturation.kp * (position_sens_k_ - position_sens_k_1_);

                            pos_ctrl_with_saturation.delta_y_k = pos_ctrl_with_saturation.gained_error*pos_ctrl_with_saturation.ki - pos_ctrl_with_saturation.feedback_p_loop;

                            pos_ctrl_with_saturation.y_k = pos_ctrl_with_saturation.delta_y_k + pos_ctrl_with_saturation.y_k_1;

                            if(pos_ctrl_with_saturation.y_k>0)
                            {
                                pos_ctrl_with_saturation.abs_y_k = pos_ctrl_with_saturation.y_k;
                                pos_ctrl_with_saturation.y_k_sign= 1;
                            }
                            else if (pos_ctrl_with_saturation.y_k<0)
                            {
                                pos_ctrl_with_saturation.abs_y_k =-pos_ctrl_with_saturation.y_k;
                                pos_ctrl_with_saturation.y_k_sign=-1;
                            }
                            else if (pos_ctrl_with_saturation.y_k == 0)
                            {
                                pos_ctrl_with_saturation.abs_y_k  = 0;
                                pos_ctrl_with_saturation.y_k_sign = 0;
                            }

                            pos_ctrl_with_saturation.state_1 = pos_ctrl_with_saturation.abs_y_k;

                            pos_ctrl_with_saturation.dynamic_max_speed  = (2.00*pos_ctrl_with_saturation.t_max)/1000;

                            if(pos_ctrl_with_saturation.gained_error>0)
                                pos_ctrl_with_saturation.dynamic_max_speed *=   pos_ctrl_with_saturation.gained_error;
                            else if(pos_ctrl_with_saturation.gained_error<0)
                                pos_ctrl_with_saturation.dynamic_max_speed *= (-pos_ctrl_with_saturation.gained_error);
                            else if(pos_ctrl_with_saturation.gained_error==0)
                                pos_ctrl_with_saturation.dynamic_max_speed  = 0;

                            pos_ctrl_with_saturation.dynamic_max_speed /= pos_ctrl_with_saturation.k_fb;
                            pos_ctrl_with_saturation.dynamic_max_speed *= 1000.00;
                            pos_ctrl_with_saturation.dynamic_max_speed /= (pos_ctrl_with_saturation.j/1000.00);
                            pos_ctrl_with_saturation.dynamic_max_speed  = sqrt(pos_ctrl_with_saturation.dynamic_max_speed);

                            pos_ctrl_with_saturation.state_2 = pos_ctrl_with_saturation.dynamic_max_speed;

                            pos_ctrl_with_saturation.state_2*= pos_ctrl_with_saturation.kd;
                            pos_ctrl_with_saturation.state_2*= pos_ctrl_with_saturation.ts_position;
                            pos_ctrl_with_saturation.state_2*= pos_ctrl_with_saturation.k_fb;

                            pos_ctrl_with_saturation.state_2*=0.9;

                            pos_ctrl_with_saturation.state_3 = pos_ctrl_with_saturation.w_max;
                            pos_ctrl_with_saturation.state_3*= pos_ctrl_with_saturation.kd;
                            pos_ctrl_with_saturation.state_3*= pos_ctrl_with_saturation.ts_position;
                            pos_ctrl_with_saturation.state_3*= pos_ctrl_with_saturation.k_fb;


                            if(pos_ctrl_with_saturation.state_1<pos_ctrl_with_saturation.state_2)
                            {
                                pos_ctrl_with_saturation.state_min = pos_ctrl_with_saturation.state_1;
                                pos_ctrl_with_saturation.state_index=1000;
                            }
                            else
                            {
                                pos_ctrl_with_saturation.state_min = pos_ctrl_with_saturation.state_2;
                                pos_ctrl_with_saturation.state_index=2000;
                            }

                            if(pos_ctrl_with_saturation.state_3<pos_ctrl_with_saturation.state_min)
                            {
                                pos_ctrl_with_saturation.state_min = pos_ctrl_with_saturation.state_3;
                                pos_ctrl_with_saturation.state_index=3000;
                            }

                            if(   pos_ctrl_with_saturation.state_1<0 || pos_ctrl_with_saturation.state_2<0
                               || pos_ctrl_with_saturation.state_3<0 || pos_ctrl_with_saturation.state_min<0)
                            {
                                printstr(">>   ERROR, NEGATIVE VALUE OF States !!!! \n");
                                while(1);
                            }

                            pos_ctrl_with_saturation.y_k = pos_ctrl_with_saturation.state_min * pos_ctrl_with_saturation.y_k_sign;
                            pos_ctrl_with_saturation.y_k_1 = pos_ctrl_with_saturation.y_k;

                            pos_ctrl_with_saturation.feedback_d_loop = pos_ctrl_with_saturation.kd * (position_sens_k_ - position_sens_k_1_);

                            pos_ctrl_with_saturation.torque_ref_k = pos_ctrl_with_saturation.y_k - pos_ctrl_with_saturation.feedback_d_loop;


                            if(pos_ctrl_with_saturation.torque_ref_k >  pos_ctrl_with_saturation.t_max)
                                pos_ctrl_with_saturation.torque_ref_k = pos_ctrl_with_saturation.t_max;

                            if(pos_ctrl_with_saturation.torque_ref_k < -pos_ctrl_with_saturation.t_max)
                                pos_ctrl_with_saturation.torque_ref_k =-pos_ctrl_with_saturation.t_max;

                            torque_ref_k = ((int) (pos_ctrl_with_saturation.torque_ref_k));

                            xscope_int(POSITION_REF_SP3 , ((int)(downstream_control_data.position_cmd)));
                            xscope_int(POSITION_REAL_SP3, ((int)(position_sens_k_ - initial_position_)));

                        }

                        second_order_LP_filter_shift_buffers(&position_k,
                                                             &position_k_1n,
                                                             &position_k_2n);
                    }

                    // velocity control
                    if (velocity_enable_flag == 1)
                    {
                        if (velocity_ref_k > pos_velocity_ctrl_config.max_speed)
                            velocity_ref_k = pos_velocity_ctrl_config.max_speed;
                        else if (velocity_ref_k < -pos_velocity_ctrl_config.max_speed)
                            velocity_ref_k = -pos_velocity_ctrl_config.max_speed;

                        second_order_LP_filter_update(&velocity_k,
                                                      &velocity_k_1n,
                                                      &velocity_k_2n,
                                                      &velocity_sens_k, pos_velocity_ctrl_config.control_loop_period, velocity_SO_LP_filter_param);

                        if (velocity_control_mode == VELOCITY_PID_CONTROLLER)
                        {
                            velocity_cmd_k = pid_update(velocity_ref_k, velocity_k, pos_velocity_ctrl_config.control_loop_period, velocity_control_pid_param);
                            torque_ref_k = (velocity_cmd_k / 32);
                        }

                        second_order_LP_filter_shift_buffers(&velocity_k,
                                                             &velocity_k_1n,
                                                             &velocity_k_2n);
                    }

                    torque_ref_k += additive_torque_k;

                    if(torque_ref_k > pos_velocity_ctrl_config.max_torque)
                        torque_ref_k = pos_velocity_ctrl_config.max_torque;
                    else if (torque_ref_k < (-pos_velocity_ctrl_config.max_torque))
                        torque_ref_k = (-pos_velocity_ctrl_config.max_torque);

                    i_motorcontrol.set_torque((int) torque_ref_k);
                }

#ifdef XSCOPE_POSITION_CTRL
                xscope_int(POSITION_REF, (int) (position_ref_k*512));
                xscope_int(POSITION, (int) (position_sens_k*512));
                xscope_int(POSITION_CMD, (int) position_cmd_k);
                xscope_int(POSITION_TEMP1, ((int) (acceleration_monitor*1000000000)));//velocity_control_pid_param.Kp);
                xscope_int(VELOCITY_REF, (int) velocity_ref_k);
                xscope_int(VELOCITY, (int) velocity_sens_k);
                xscope_int(VELOCITY_CMD, (int) velocity_cmd_k);
                xscope_int(VELOCITY_TEMP1, (int) torque_ref_k);
#endif
#ifdef XSCOPE_POSITION_CTRL_2
                xscope_int(VELOCITY, upstream_control_data.velocity);
                xscope_int(POSITION, upstream_control_data.position);
                xscope_int(TORQUE,   upstream_control_data.computed_torque);
                xscope_int(POSITION_REF, position_ref_input_k * 1);
                xscope_int(TORQUE_CMD, torque_ref_k / 1);
#endif


                t :> t_end_;
                break;



            case i_position_control[int i].disable():
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
                position_enable_flag = 1;
                if (pos_control_mode == POS_PID_VELOCITY_CASCADED_CONTROLLER) {
                    velocity_enable_flag = 1;
                    velocity_control_mode = VELOCITY_PID_CONTROLLER;
                }
                else {
                    velocity_enable_flag = 0;
                    velocity_control_mode = VELOCITY_PID_CONTROLLER;
                }

                //////nonlinear position control
                position_ref_input_k_ = 0;
                position_ref_k_ = 0;
                position_sens_k_ = 0;
                position_sens_k_1_=0;
                pos_ctrl_with_saturation.torque_ref_k = 0.00;
                pos_ctrl_with_saturation.t_additive = 0.00;
                pos_ctrl_with_saturation.feedback_p_loop =0.00;
                pos_ctrl_with_saturation.feedback_d_loop=0.00;
                //////////////////////////////

                upstream_control_data = i_motorcontrol.update_upstream_control_data();

                //////nonlinear position control
                downstream_control_data.position_cmd = 0;
                pos_ctrl_with_saturation.t_additive = 0.00;
                initial_position_ = upstream_control_data.position;
                ////////////////////////////////


                position_ref_input_k = upstream_control_data.position;
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
                integral_optimum_pos_controller_reset(integral_optimum_pos_ctrl_pid_param);
                pid_reset(velocity_control_pid_param);
                i_motorcontrol.set_torque_control_enabled();
                i_motorcontrol.set_brake_status(1);
                enable_profiler = pos_velocity_ctrl_config.enable_profiler;
                break;

            case i_position_control[int i].enable_velocity_ctrl(int velocity_control_mode_):
                velocity_control_mode = velocity_control_mode_;
                torque_enable_flag = 1;
                position_enable_flag = 0;
                velocity_enable_flag = 1;
                velocity_ref_input_k = 0;
                additive_torque_input_k = 0;
                pid_reset(velocity_control_pid_param);
                i_motorcontrol.set_torque_control_enabled();
                i_motorcontrol.set_brake_status(1);
                break;

            case i_position_control[int i].enable_torque_ctrl():
                torque_enable_flag = 1;
                position_enable_flag = 0;
                velocity_enable_flag = 0;
                torque_ref_input_k = 0;
                i_motorcontrol.set_torque_control_enabled();
                i_motorcontrol.set_brake_status(1);
                break;

            case i_position_control[int i].set_position_velocity_control_config(PosVelocityControlConfig in_config):
                pos_velocity_ctrl_config = in_config;
                max_position = ((float) pos_velocity_ctrl_config.max_pos);
                max_position /= 512;
                min_position = ((float) pos_velocity_ctrl_config.min_pos);
                min_position /= 512;
//                pid_init(velocity_control_pid_param);
                pid_set_parameters((float)pos_velocity_ctrl_config.P_velocity, (float)pos_velocity_ctrl_config.I_velocity,
                                   (float)pos_velocity_ctrl_config.D_velocity, (float)pos_velocity_ctrl_config.integral_limit_velocity,
                                          pos_velocity_ctrl_config.control_loop_period, velocity_control_pid_param);
//                pid_init(position_control_pid_param);
                pid_set_parameters((float)pos_velocity_ctrl_config.P_pos, (float)pos_velocity_ctrl_config.I_pos,
                                   (float)pos_velocity_ctrl_config.D_pos, (float)pos_velocity_ctrl_config.integral_limit_pos,
                                          pos_velocity_ctrl_config.control_loop_period, position_control_pid_param);
//                integral_optimum_pos_controller_init(integral_optimum_pos_ctrl_pid_param);
                integral_optimum_pos_controller_set_parameters((float)pos_velocity_ctrl_config.P_pos_Integral_optimum, (float)pos_velocity_ctrl_config.I_pos_Integral_optimum,
                                                               (float)pos_velocity_ctrl_config.D_pos_Integral_optimum, (float)pos_velocity_ctrl_config.integral_limit_pos_Integral_optimum,
                                                                pos_velocity_ctrl_config.control_loop_period, integral_optimum_pos_ctrl_pid_param);
                second_order_LP_filter_init(pos_velocity_ctrl_config.position_fc, pos_velocity_ctrl_config.control_loop_period, position_SO_LP_filter_param);
                second_order_LP_filter_init(pos_velocity_ctrl_config.velocity_fc, pos_velocity_ctrl_config.control_loop_period, velocity_SO_LP_filter_param);
                pos_profiler_param.a_max = ((float) pos_velocity_ctrl_config.max_acceleration_profiler);
                pos_profiler_param.v_max = ((float) pos_velocity_ctrl_config.max_speed_profiler);
                enable_profiler = pos_velocity_ctrl_config.enable_profiler;
                break;

            case i_position_control[int i].get_position_velocity_control_config() ->  PosVelocityControlConfig out_config:
                out_config = pos_velocity_ctrl_config;
                break;


            case i_position_control[int i].update_control_data(DownstreamControlData downstream_control_data_) -> UpstreamControlData upstream_control_data_:
                upstream_control_data_ = upstream_control_data;
                downstream_control_data = downstream_control_data_;
                position_ref_input_k = downstream_control_data.position_cmd;
                velocity_ref_input_k = downstream_control_data.velocity_cmd;
                torque_ref_input_k = downstream_control_data.torque_cmd;
                additive_torque_input_k = downstream_control_data.offset_torque;
                break;


            case i_position_control[int i].set_j(int j):
                    pos_ctrl_with_saturation.j = ((double)(j));
                    break;











//            case i_position_control[int i].set_position(int in_target_position):
//                    position_ref_input_k = in_target_position;
//                break;
//            case i_position_control[int i].set_position_pid_coefficients(int int8_Kp, int int8_Ki, int int8_Kd):
//                    pid_set_parameters((float)int8_Kp, (float)int8_Ki, (float)int8_Kd, (float)pos_velocity_ctrl_config.integral_limit_pos, pos_velocity_ctrl_config.control_loop_period, position_control_pid_param);
//                break;
//            case i_position_control[int i].set_position_pid_limits(int int16_P_error_limit, int int16_I_error_limit, int int16_itegral_limit, int int21_target_max_velocity_):
//                pos_velocity_ctrl_config.max_speed = int21_target_max_velocity_;
//                pid_set_parameters((float)position_control_pid_param.Kp, (float)position_control_pid_param.Ki, (float)position_control_pid_param.Kd, (float)int16_itegral_limit, pos_velocity_ctrl_config.control_loop_period, position_control_pid_param);
//                break;
//            case i_position_control[int i].set_position_limits(int position_min_limit, int position_max_limit):
//                pos_velocity_ctrl_config.min_pos = position_min_limit;
//                pos_velocity_ctrl_config.max_pos = position_max_limit;
//                max_position = ((float) pos_velocity_ctrl_config.max_pos);
//                max_position /= 512;
//                min_position = ((float) pos_velocity_ctrl_config.min_pos);
//                min_position /= 512;
//                break;



//            case i_position_control[int i].set_velocity(int in_target_velocity):
//                    velocity_ref_input_k = in_target_velocity * 1;
//                break;
//            case i_position_control[int i].set_offset_torque(int offset_torque_):
//                    additive_torque_input_k = offset_torque_;
//                break;
//            case i_position_control[int i].set_velocity_pid_coefficients(int int8_Kp, int int8_Ki, int int8_Kd):
//                    pid_set_parameters((float)int8_Kp, (float)int8_Ki, (float)int8_Kd, (float)pos_velocity_ctrl_config.integral_limit_velocity, pos_velocity_ctrl_config.control_loop_period, velocity_control_pid_param);
//                break;
//            case i_position_control[int i].set_velocity_pid_limits(int int16_P_error_limit, int int16_I_error_limit, int int16_itegral_limit, int int21_target_max_torque_):
//                pos_velocity_ctrl_config.max_torque = int21_target_max_torque_;
//                pid_set_parameters((float)velocity_control_pid_param.Kp, (float)velocity_control_pid_param.Ki, (float)velocity_control_pid_param.Kd, (float)int16_itegral_limit, pos_velocity_ctrl_config.control_loop_period, velocity_control_pid_param);
//                break;
//            case i_position_control[int i].set_velocity_limits(int velocity_min_limit, int velocity_max_limit):
//                pos_velocity_ctrl_config.max_speed = velocity_max_limit;
//                break;



//            case i_position_control[int i].set_torque(int in_target_torque):
//                torque_ref_input_k = in_target_torque;
//                break;
//            case i_position_control[int i].set_torque_limits(int torque_min_limit, int torque_max_limit):
//                pos_velocity_ctrl_config.max_torque = torque_max_limit;
//                break;




//            case i_position_control[int i].get_position() -> int out_position:
//                    out_position = upstream_control_data.position;
//                break;
//
//
//            case i_position_control[int i].get_velocity() -> int out_velocity:
//                    out_velocity = upstream_control_data.velocity;
//                break;
//
//            case i_position_control[int i].check_busy() -> int out_activate:
////                out_activate = position_enable_flag;
//                break;


        }
    }
}
