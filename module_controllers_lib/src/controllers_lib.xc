/**
 * @file controllers_lib.xc
 * @brief Controllers Libraries
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <controllers_lib.h>
#include <control_loops_common.h>
#include <math.h>



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
 * @param the parameters of the controller
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
 * @param the parameters of the PID controller
 */
void pid_set_parameters(float Kp, float Ki, float Kd, float integral_limit, int T_s, PIDparam &param)
{
    param.Kp = Kp;
    param.Ki = Ki;
    param.Kd = Kd;
    param.integral_limit = integral_limit;
    param.T_s = T_s;
}

/**
 * @brief updating the PID controller.
 * @param output, control command
 * @param input, setpoint
 * @param input, feedback
 * @param input, sample-time in us (microseconds).
 * @param the parameters of the PID controller
 */
float pid_update(float desired_value, float actual_value, int T_s, PIDparam &param)
{
    float error, cmd, integral_term;
    error = desired_value - actual_value;
    param.integral += error;
    integral_term = param.Ki * param.integral;
    if ((integral_term > param.integral_limit) || (integral_term < -param.integral_limit))
        param.integral -= error;
    cmd = ((param.Kp * error) + integral_term - (param.Kd * (actual_value - param.actual_value_1n)));
    param.actual_value_1n = actual_value;
    return cmd;
}


/**
 * @brief resetting the parameters of the PID controller.
 * @param the parameters of the controller
 */
void pid_reset(PIDparam &param)
{
    param.actual_value_1n = 0;
    param.integral = 0;
}

void nl_position_control_reset(NonlinearPositionControl &nl_pos_ctrl)
{
    //************************************
    // reset position controller structure
    nl_pos_ctrl.os_suppression = 0.00; // in micro-kgm2
    nl_pos_ctrl.k_fb = 0.00;
    nl_pos_ctrl.k_m = 0.00;
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
}


void nl_position_control_set_parameters(NonlinearPositionControl &nl_pos_ctrl, PosVelocityControlConfig &pos_velocity_ctrl_config)
{
    //************************************************
    // set parameters of position controller structure
    nl_pos_ctrl.w_max= (((double)(pos_velocity_ctrl_config.max_speed))*2.00*3.1415)/60;
    nl_pos_ctrl.resolution = ((double)(pos_velocity_ctrl_config.resolution));
    nl_pos_ctrl.k_fb = (nl_pos_ctrl.resolution)/(2.00*3.1416);
    nl_pos_ctrl.k_m  = ( (double)(1) )/1000.00;
    nl_pos_ctrl.ts_position = ((double)(pos_velocity_ctrl_config.control_loop_period))/1000000.00; //s

    //PID parameters are pre-multiplied by 100 (by user)
    nl_pos_ctrl.kp =  ((double)(pos_velocity_ctrl_config.P_pos))/100.00;
    nl_pos_ctrl.ki =  ((double)(pos_velocity_ctrl_config.I_pos))/100.00;
    nl_pos_ctrl.kd =  ((double)(pos_velocity_ctrl_config.D_pos))/100.00;

    nl_pos_ctrl.constant_gain = 2/nl_pos_ctrl.ts_position;
    nl_pos_ctrl.constant_gain/= nl_pos_ctrl.ts_position;
    nl_pos_ctrl.constant_gain/=(nl_pos_ctrl.k_fb * nl_pos_ctrl.k_m);

    nl_pos_ctrl.os_suppression   = ((double)(pos_velocity_ctrl_config.os_suppression)); //s

    nl_pos_ctrl.kp *= nl_pos_ctrl.os_suppression;
    nl_pos_ctrl.ki *= nl_pos_ctrl.os_suppression;
    nl_pos_ctrl.kd *= nl_pos_ctrl.os_suppression;
    nl_pos_ctrl.kp /=10000000.00;
    nl_pos_ctrl.ki /=10000000.00;
    nl_pos_ctrl.kd /=10000000.00;

    nl_pos_ctrl.t_max=((double)(pos_velocity_ctrl_config.max_torque));
}


/**
 * @brief updating the output of position controller with update.
 * @param output, torque reference in milli-Nm
 * @param input, setpoint
 * @param input, feedback
 */
int update_nl_position_control(
        NonlinearPositionControl &nl_pos_ctrl,
        double position_ref_k_,
        double position_sens_k_1_,
        double position_sens_k_)
{

    nl_pos_ctrl.gained_error = position_ref_k_ - position_sens_k_;

    nl_pos_ctrl.feedback_p_loop  = nl_pos_ctrl.kp * (position_sens_k_ - position_sens_k_1_);

    nl_pos_ctrl.delta_y_k = nl_pos_ctrl.gained_error*nl_pos_ctrl.ki - nl_pos_ctrl.feedback_p_loop;

    nl_pos_ctrl.y_k = nl_pos_ctrl.delta_y_k + nl_pos_ctrl.y_k_1;

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

    //overshoot suppresion factor is multiplied by 1e7
    nl_pos_ctrl.dynamic_max_speed *= 10000.00;
    nl_pos_ctrl.dynamic_max_speed /= (nl_pos_ctrl.os_suppression/1000.00);

    nl_pos_ctrl.dynamic_max_speed  = sqrt(nl_pos_ctrl.dynamic_max_speed);

    nl_pos_ctrl.state_2 = nl_pos_ctrl.dynamic_max_speed;

    nl_pos_ctrl.state_2*= nl_pos_ctrl.kd;
    nl_pos_ctrl.state_2*= nl_pos_ctrl.ts_position;
    nl_pos_ctrl.state_2*= nl_pos_ctrl.k_fb;
    //the effect of transient ref_torque is not considered in state_2
    nl_pos_ctrl.state_2*=0.9;

    nl_pos_ctrl.state_3 = nl_pos_ctrl.w_max;
    nl_pos_ctrl.state_3*= nl_pos_ctrl.kd;
    nl_pos_ctrl.state_3*= nl_pos_ctrl.ts_position;
    nl_pos_ctrl.state_3*= nl_pos_ctrl.k_fb;


    if(nl_pos_ctrl.state_1<nl_pos_ctrl.state_2)
    {
        nl_pos_ctrl.state_min = nl_pos_ctrl.state_1;
        nl_pos_ctrl.state_index=1000;
    }
    else
    {
        nl_pos_ctrl.state_min = nl_pos_ctrl.state_2;
        nl_pos_ctrl.state_index=2000;
    }

    if(nl_pos_ctrl.state_3<nl_pos_ctrl.state_min)
    {
        nl_pos_ctrl.state_min = nl_pos_ctrl.state_3;
        nl_pos_ctrl.state_index=3000;
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
 * @param output, profiled position calculated for the next step
 * @param input, target position
 * @param input, profiled position calculated in one step ago
 * @param input, profiled position calculated in two steps ago
 * @param the parameters of the position reference profiler
 */
float pos_profiler(float pos_target, float pos_k_1n, float pos_k_2n, posProfilerParam pos_profiler_param)
{
    float velocity_k_1n, temp, deceleration_distance, pos_deceleration, pos_k, pos_temp1, pos_temp2;
    int deceleration_flag = 0;


    if (pos_target == pos_k_1n)
        pos_k = pos_target;
    else if (pos_target > pos_k_1n) {
        if (((pos_k_1n-pos_k_2n)==0) && (pos_target < (pos_k_1n+10)))
            pos_k = pos_k_1n; //ignore the command
        else {
            velocity_k_1n = ((pos_k_1n - pos_k_2n) / pos_profiler_param.delta_T);
            deceleration_distance = (velocity_k_1n * velocity_k_1n) / (2 * pos_profiler_param.a_max);
            pos_deceleration = pos_target - deceleration_distance;
            if ((pos_k_1n >= pos_deceleration) && (pos_k_1n > pos_k_2n))
                deceleration_flag = 1;
            temp = pos_profiler_param.delta_T * pos_profiler_param.delta_T * pos_profiler_param.a_max;
            if (deceleration_flag == 0) {
                pos_temp1 = temp + (2 * pos_k_1n) - pos_k_2n;
                pos_temp2 = (pos_profiler_param.delta_T * pos_profiler_param.v_max) + pos_k_1n;
                if (pos_temp1 < pos_temp2)
                    pos_k = pos_temp1;
                else
                    pos_k = pos_temp2;
            }
            else {
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
        else {
            velocity_k_1n = ((pos_k_1n - pos_k_2n) / pos_profiler_param.delta_T);
            deceleration_distance = (velocity_k_1n * velocity_k_1n) / (2 * pos_profiler_param.a_max);
            pos_deceleration = pos_target + deceleration_distance;
            if ((pos_k_1n <= pos_deceleration) && (pos_k_1n < pos_k_2n))
                deceleration_flag = 1;
            temp = pos_profiler_param.delta_T * pos_profiler_param.delta_T * pos_profiler_param.a_max;
            if (deceleration_flag == 0) {
                pos_temp1 = -temp + (2 * pos_k_1n) - pos_k_2n;
                pos_temp2 = -(pos_profiler_param.delta_T * pos_profiler_param.v_max) + pos_k_1n;
                if (pos_temp1 > pos_temp2)
                    pos_k = pos_temp1;
                else
                    pos_k = pos_temp2;
            }
            else {
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

