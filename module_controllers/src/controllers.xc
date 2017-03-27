/**
 * @file controllers.xc
 * @brief Controllers Libraries
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <controllers.h>
#include <motion_control_service.h>
#include <math.h>
#include <xscope.h>


/**
 * @brief sign function.
 * @param output, sign of the number
 * @param input, number
 */
int sign_function(float a)
{
    if (a < 0)
        return -1;
    else
        return 1;
}

/**
 * @brief intializing the parameters of the PID controller.
 *
 * @param the parameters of the controller
 *
 * @return void
 */
void pid_init(PIDparam &param)
{
    param.Kp = 0;
    param.Ki = 0;
    param.Kd = 0;
    param.integral_limit = 0;
    param.integral = 0;
    param.actual_value_1n = 0;
    param.T_s = 0;
}

/**
 * @brief setting the parameters of the PID controller.
 * @param input, P parameter
 * @param input, I parameter
 * @param input, D parameter
 * @param input, Integral limit
 * @param input, sample-time in us (microseconds).
 * @param structure including the parameters of the PID controller
 *
 * @return void
 */
void pid_set_parameters(double Kp, double Ki, double Kd, double integral_limit, int T_s, PIDparam &param)
{
    param.Kp = Kp;
    param.Ki = Ki;
    param.Kd = Kd;
    param.integral_limit = integral_limit;
    param.T_s = T_s;
}

/**
 * @brief updating the PID controller.
 * @param desired_value, the reference set point
 * @param actual_value, the actual value (measurement)
 * @param T_s, sampling time
 * @param param, the structure containing the pid controller parameters
 *
 *
 * @return the output of pid controller
 */
double pid_update(double desired_value, double actual_value, int T_s, PIDparam &param)
{
    double error=0.00, cmd=0.00;

    error = desired_value - actual_value;

    param.integral += (param.Ki/1000000.00) * error;
    if ((param.integral >= param.integral_limit) || (param.integral <= -param.integral_limit))
        param.integral -= ((param.Ki/1000000.00) * error);

    cmd = ((param.Kp/1000000.00) * (desired_value- actual_value)) + param.integral - ((param.Kd/1000000.00) * (actual_value - param.actual_value_1n));

    param.actual_value_1n = actual_value;

    return cmd;
}

/**
 * @brief resetting the parameters of the PID controller.
 * @param the parameters of the controller
 *
 * @return void
 */
void pid_reset(PIDparam &param)
{
    param.actual_value_1n = 0;
    param.integral = 0;
}

/**
 * @brief resetting the parameters of the nonlinear position controller
 * @param the parameters of the controller
 *
 * @return void
 */
void nl_position_control_reset(NonlinearPositionControl &nl_pos_ctrl)
{
    //************************************
    // reset position controller structure
    nl_pos_ctrl.k_fb = 0.00;
    nl_pos_ctrl.k_m = 0.00;
    nl_pos_ctrl.moment_of_inertia = 0.00;
    nl_pos_ctrl.ts_position = 0.00;

    nl_pos_ctrl.kp =  0.00;
    nl_pos_ctrl.ki =  0.00;
    nl_pos_ctrl.kd =  0.00;

    nl_pos_ctrl.feedback_p_loop=0.00;
    nl_pos_ctrl.feedback_d_loop=0.00;
    nl_pos_ctrl.gained_error=0.00;
    nl_pos_ctrl.t_max=0.00;

    nl_pos_ctrl.y_k=0.00;
    nl_pos_ctrl.y_k_sign=0.00;
    nl_pos_ctrl.y_k_1=0.00;
    nl_pos_ctrl.delta_y_k=0.00;

    nl_pos_ctrl.dynamic_max_speed=0.00;
    nl_pos_ctrl.w_max = 0.00;
    nl_pos_ctrl.state_1=0.00;
    nl_pos_ctrl.state_2=0.00;
    nl_pos_ctrl.state_3=0.00;
    nl_pos_ctrl.state_min=0.00;

    nl_pos_ctrl.torque_ref_k=0.00;
    nl_pos_ctrl.t_additive = 0.00;
}


/**
 * @brief resetting the parameters of nonlinear position controller.
 *
 * @param nl_pos_ctrl, structure containing the parameters of the controller
 * @param pos_velocity_ctrl_config, structure containing the parameters of non-linear position controller
 * @param control_loop_period in us
 *
 * @return void
 */
void nl_position_control_set_parameters(NonlinearPositionControl &nl_pos_ctrl, MotionControlConfig &motion_ctrl_config, int control_loop_period)
{
    //************************************************
    // set parameters of position controller structure
    nl_pos_ctrl.w_max= (((double)(motion_ctrl_config.max_motor_speed))*2.00*3.1415)/60;
    nl_pos_ctrl.resolution = ((double)(motion_ctrl_config.resolution));
    nl_pos_ctrl.k_fb = (nl_pos_ctrl.resolution)/(2.00*3.1416);
    nl_pos_ctrl.k_m  = ( (double)(1) )/1000.00;
    nl_pos_ctrl.moment_of_inertia   = ((double)(motion_ctrl_config.moment_of_inertia));
    nl_pos_ctrl.ts_position = ((double)(control_loop_period))/1000000.00; //s

    //check if the PID parameters are in range
    if(0<=motion_ctrl_config.position_kp && motion_ctrl_config.position_kp<=100000000)
        nl_pos_ctrl.kp =  ((double)(motion_ctrl_config.position_kp));
    if(0<=motion_ctrl_config.position_ki && motion_ctrl_config.position_ki<=100000000)
        nl_pos_ctrl.ki =  ((double)(motion_ctrl_config.position_ki));
    if(0<=motion_ctrl_config.position_kd && motion_ctrl_config.position_kd<=100000000)
        nl_pos_ctrl.kd =  ((double)(motion_ctrl_config.position_kd));

    nl_pos_ctrl.constant_gain = 2/nl_pos_ctrl.ts_position;
    nl_pos_ctrl.constant_gain/= nl_pos_ctrl.ts_position;
    nl_pos_ctrl.constant_gain/=(nl_pos_ctrl.k_fb * nl_pos_ctrl.k_m);

    nl_pos_ctrl.integral_limit_pos = ((double)(motion_ctrl_config.position_integral_limit))/1000.00;

    nl_pos_ctrl.kd *= nl_pos_ctrl.integral_limit_pos;
    nl_pos_ctrl.kd /=100000.00;

    nl_pos_ctrl.calculated_j = (nl_pos_ctrl.kd*10000000)/(nl_pos_ctrl.constant_gain*0.216);

    nl_pos_ctrl.t_max=((double)(motion_ctrl_config.max_torque));
}

/**
 * @brief updating the output of position controller.
 *
 * @param nl_pos_ctrl, structure containing the parameters of position controller
 * @param position_ref_k_, the reference value of position
 * @param position_sens_k_1_, actual position value in previous sampling
 * @param position_sens_k_, actual position value in current sampling
 *
 * @return the reference value of required torque (in milli-Nm)
 */
int update_nl_position_control(
        NonlinearPositionControl &nl_pos_ctrl,
        double position_ref_k_,
        double position_sens_k_1_,
        double position_sens_k_)
{
    nl_pos_ctrl.gained_error = position_ref_k_ - position_sens_k_;

    nl_pos_ctrl.feedback_p_loop  = position_sens_k_ - position_sens_k_1_;
    nl_pos_ctrl.feedback_p_loop *= nl_pos_ctrl.kp;

    nl_pos_ctrl.delta_y_k = position_ref_k_ - position_sens_k_;
    nl_pos_ctrl.delta_y_k*= nl_pos_ctrl.ki;
    nl_pos_ctrl.delta_y_k-= nl_pos_ctrl.feedback_p_loop;
    nl_pos_ctrl.delta_y_k*= nl_pos_ctrl.integral_limit_pos;

    nl_pos_ctrl.y_k = (nl_pos_ctrl.delta_y_k/100000.00) + nl_pos_ctrl.y_k_1;

    if(nl_pos_ctrl.y_k>0)
    {
        nl_pos_ctrl.abs_y_k = nl_pos_ctrl.y_k;
        nl_pos_ctrl.y_k_sign= 1;
    }
    else if (nl_pos_ctrl.y_k<0)
    {
        nl_pos_ctrl.abs_y_k =-nl_pos_ctrl.y_k;
        nl_pos_ctrl.y_k_sign=-1;
    }
    else if (nl_pos_ctrl.y_k == 0)
    {
        nl_pos_ctrl.abs_y_k  = 0;
        nl_pos_ctrl.y_k_sign = 0;
    }

    nl_pos_ctrl.state_1 = nl_pos_ctrl.abs_y_k;

    nl_pos_ctrl.dynamic_max_speed  = (2.00*nl_pos_ctrl.t_max)/1000;//t_max is considered as milli-Nm

    if(nl_pos_ctrl.gained_error>0)
        nl_pos_ctrl.dynamic_max_speed *=   nl_pos_ctrl.gained_error;
    else if(nl_pos_ctrl.gained_error<0)
        nl_pos_ctrl.dynamic_max_speed *= (-nl_pos_ctrl.gained_error);
    else if(nl_pos_ctrl.gained_error==0)
        nl_pos_ctrl.dynamic_max_speed  = 0;

    nl_pos_ctrl.dynamic_max_speed /= nl_pos_ctrl.k_fb;

    if(nl_pos_ctrl.moment_of_inertia>=1)
    {
        //moment of inertia in gram square centimeter
        nl_pos_ctrl.dynamic_max_speed *= 10000.00;
        nl_pos_ctrl.dynamic_max_speed /= (nl_pos_ctrl.moment_of_inertia/1000.00);
    }

    nl_pos_ctrl.dynamic_max_speed  = sqrt(nl_pos_ctrl.dynamic_max_speed);

    nl_pos_ctrl.state_2 = nl_pos_ctrl.dynamic_max_speed;

    nl_pos_ctrl.state_2*= nl_pos_ctrl.kd;
    nl_pos_ctrl.state_2*= nl_pos_ctrl.ts_position;
    nl_pos_ctrl.state_2*= nl_pos_ctrl.k_fb;
    //FIXME: the effect of transient ref_torque is not considered in state_2
    nl_pos_ctrl.state_2*=0.9;

    nl_pos_ctrl.state_3 = nl_pos_ctrl.w_max;
    nl_pos_ctrl.state_3*= nl_pos_ctrl.kd;
    nl_pos_ctrl.state_3*= nl_pos_ctrl.ts_position;
    nl_pos_ctrl.state_3*= nl_pos_ctrl.k_fb;


    //consider state_2 only if it is possible to measure moment of inertia.
    //otherwise, only consider state_1 and state_3
    if(nl_pos_ctrl.state_1<nl_pos_ctrl.state_3)
    {
        nl_pos_ctrl.state_min = nl_pos_ctrl.state_1;
        nl_pos_ctrl.state_index=1000;
    }
    else
    {
        nl_pos_ctrl.state_min = nl_pos_ctrl.state_3;
        nl_pos_ctrl.state_index=3000;
    }

    if((nl_pos_ctrl.state_2<nl_pos_ctrl.state_min)&&(nl_pos_ctrl.moment_of_inertia>=1))
    {
        nl_pos_ctrl.state_min = nl_pos_ctrl.state_2;
        nl_pos_ctrl.state_index=2000;
    }

    nl_pos_ctrl.y_k = nl_pos_ctrl.state_min * nl_pos_ctrl.y_k_sign;
    nl_pos_ctrl.y_k_1 = nl_pos_ctrl.y_k;

    nl_pos_ctrl.feedback_d_loop = nl_pos_ctrl.kd * (position_sens_k_ - position_sens_k_1_);

    nl_pos_ctrl.torque_ref_k = nl_pos_ctrl.y_k - nl_pos_ctrl.feedback_d_loop;


    if(nl_pos_ctrl.torque_ref_k >  nl_pos_ctrl.t_max)
        nl_pos_ctrl.torque_ref_k = nl_pos_ctrl.t_max;

    if(nl_pos_ctrl.torque_ref_k < -nl_pos_ctrl.t_max)
        nl_pos_ctrl.torque_ref_k =-nl_pos_ctrl.t_max;

    return ((int) (nl_pos_ctrl.torque_ref_k));

}

/**
 * @brief updating the position reference profiler
 * @param   pos_target, target position
 * @param   pos_k_1n, profiled position calculated in one step ago
 * @param   pos_k_2n, profiled position calculated in two steps ago
 * @param   pos_profiler_param parameters of the position reference profiler
 *
 * @return  profiled position calculated for the next step
 */
float pos_profiler(double pos_target, double pos_k_1n, double pos_k_2n, posProfilerParam pos_profiler_param)
{
    double velocity_k_1n, temp, deceleration_distance, pos_deceleration, pos_k, pos_temp1, pos_temp2, v_max = 0.00, a_max = 0.00;
    int deceleration_flag = 0;

    v_max = (((double)(pos_profiler_param.v_max)) * pos_profiler_param.resolution )/60.00;
    a_max = (((double)(pos_profiler_param.a_max)) * pos_profiler_param.resolution )/60.00;

    if (pos_target == pos_k_1n)
        pos_k = pos_target;
    else if (pos_target > pos_k_1n)
    {
        if (((pos_k_1n-pos_k_2n)==0) && (pos_target < (pos_k_1n+10)))
            pos_k = pos_k_1n; //ignore the command
        else
        {
            velocity_k_1n = ((pos_k_1n - pos_k_2n) / pos_profiler_param.delta_T);
            deceleration_distance = (velocity_k_1n * velocity_k_1n) / (2 * a_max);
            pos_deceleration = pos_target - deceleration_distance;
            if ((pos_k_1n >= pos_deceleration) && (pos_k_1n > pos_k_2n))
                deceleration_flag = 1;
            temp = pos_profiler_param.delta_T * pos_profiler_param.delta_T * a_max;
            if (deceleration_flag == 0)
            {
                pos_temp1 = temp + (2 * pos_k_1n) - pos_k_2n;
                pos_temp2 = (pos_profiler_param.delta_T * v_max) + pos_k_1n;
                if (pos_temp1 < pos_temp2)
                    pos_k = pos_temp1;
                else
                    pos_k = pos_temp2;
            }
            else
            {
                pos_k = -temp + (2 * pos_k_1n) - pos_k_2n;
            }
            if (pos_k > pos_target)
                pos_k = pos_target;
            if ((pos_k < pos_target) && (sign_function(pos_k_1n-pos_k_2n) > sign_function(pos_k-pos_k_1n)))
                pos_k = pos_target;
        }
    }
    else
    {
        if (((pos_k_1n-pos_k_2n)==0) && (pos_target > (pos_k_1n-10)))
            pos_k = pos_k_1n; //ignore the command
        else
        {
            velocity_k_1n = ((pos_k_1n - pos_k_2n) / pos_profiler_param.delta_T);
            deceleration_distance = (velocity_k_1n * velocity_k_1n) / (2 * a_max);
            pos_deceleration = pos_target + deceleration_distance;
            if ((pos_k_1n <= pos_deceleration) && (pos_k_1n < pos_k_2n))
                deceleration_flag = 1;
            temp = pos_profiler_param.delta_T * pos_profiler_param.delta_T * a_max;
            if (deceleration_flag == 0)
            {
                pos_temp1 = -temp + (2 * pos_k_1n) - pos_k_2n;
                pos_temp2 = -(pos_profiler_param.delta_T * v_max) + pos_k_1n;
                if (pos_temp1 > pos_temp2)
                    pos_k = pos_temp1;
                else
                    pos_k = pos_temp2;
            }
            else
            {
                pos_k = temp + (2 * pos_k_1n) - pos_k_2n;
            }
            if (pos_k < pos_target)
                pos_k = pos_target;
            if ((pos_k > pos_target) && (sign_function(pos_k_1n-pos_k_2n) < sign_function(pos_k-pos_k_1n)))
                pos_k = pos_target;
        }
    }

    return pos_k;
}

/**
 * @brief updating the velocity reference profiler
 *
 * @param   velocity_ref, target velocity
 * @param   velocity_ref_in_k_1n, profiled velocity calculated in one step
 * @param   profiler_param, structure containing the profiler parameters
 * @param   velocity_control_loop, the execution cycle of velocity controller (us)
 *
 * @return  profiled velocity calculated for the next step
 */
double velocity_profiler(double velocity_ref, double velocity_ref_in_k_1n, posProfilerParam profiler_param, int position_control_loop)
{

    double velocity_step=0.00, velocity_ref_in_k=0.00;
    double velocity_error=0.00;

    velocity_step = (((double)(position_control_loop)) * profiler_param.a_max )/1000000.00; //rpm/s
    if(velocity_step<0) velocity_step=-velocity_step;

    if(velocity_ref_in_k_1n<velocity_ref)
    {
        velocity_ref_in_k = velocity_ref_in_k_1n + velocity_step;
    }
    else if (velocity_ref_in_k_1n>velocity_ref)
    {
        velocity_ref_in_k = velocity_ref_in_k_1n - velocity_step;
    }

    velocity_error = velocity_ref - velocity_ref_in_k_1n;
    if( (-velocity_step)<velocity_error  && velocity_error<velocity_step)
        velocity_ref_in_k = velocity_ref;

    return velocity_ref_in_k;
}

/**
 * @brief updating the torque reference profiler
 *
 * @param   torque_ref, target torque
 * @param   torque_ref_in_k_1n, profiled torque calculated in one step
 * @param   profiler_param, structure containing the profiler parameters
 * @param   torque_control_loop, the execution cycle of torque controller (us)
 *
 * @return  profiled torque calculated for the next step
 */
double torque_profiler(double torque_ref, double torque_ref_in_k_1n, posProfilerParam profiler_param, int position_control_loop)
{

    double torque_step=0.00, torque_ref_in_k=0.00;
    double torque_error=0.00;

    torque_step = (((double)(position_control_loop)) * profiler_param.torque_rate_max )/1000000.00;
    //r_max [mNm/s] / 1000 [mNm/ms] * (position_control_loop/1000) mNm
    if(torque_step<0) torque_step=-torque_step;

    if(torque_ref_in_k_1n<torque_ref)
    {
        torque_ref_in_k = torque_ref_in_k_1n + torque_step;
    }
    else if (torque_ref_in_k_1n>torque_ref)
    {
        torque_ref_in_k = torque_ref_in_k_1n - torque_step;
    }

    torque_error = torque_ref - torque_ref_in_k_1n;
    if( (-torque_step)<torque_error  && torque_error<torque_step)
        torque_ref_in_k = torque_ref;

    return torque_ref_in_k;
}
