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

#include <controllers.h>
#include <limited_torque_position_control.h>

#include <profile.h>
#include <auto_tune.h>
#include <filters.h>

#include <motion_control_service.h>
#include <refclk.h>
#include <mc_internal_constants.h>
#include <stdio.h>
#include <cogging_torque_compensation.h>

#define HALL_MASK_A 0x4
#define HALL_MASK_B 0x2
#define HALL_MASK_C 0x1

enum
{
    NO_ERROR,
    A,
    B,
    C,
    NR_PHASES
};

typedef enum
{
    POS_ERR =   1,
    SPEED_ERR = 2,
    ANGLE_ERR = 3,
    PORTS_ERR = 4
} sensor_fault;

/*
 * checking the position, velocity and angle reading from the sensor, when user triggers 'gse' command in tuning console or prior to commutation offset detection
 */
sensor_fault sensor_functionality_evaluation(client interface TorqueControlInterface i_torque_control,
        MotorcontrolConfig motorcontrol_config, DownstreamControlData &downstream_control_data, int app_tile_usec)
{
    UpstreamControlData upstream_control_data;
    int angle = 0, position = 0, hall_state = 0, velocity = 0;

    unsigned counter = 0;
    int max_pos = 0, mean_pos = 0, real_mean_pos = 0, tq_pos = 0, real_tq_pos = 0, fq_pos = 0, real_fq_pos = 0;
    int max_angle = 0, mean_angle = 0, real_mean_angle = 0, tq_angle = 0, real_tq_angle = 0, fq_angle = 0, real_fq_angle = 0;
    int hall_state_old = 0, max_pos_found = 0, max_angle_found = 0;
    int filter_diff = 0, filter_vel = 0, hall_state_ch[NR_PHASES] = { 0 };
    int index_v = 0, velocity_arr[500], index_d = 0, difference_arr[500],  old_position = 0;
    sensor_fault error_sens = NO_ERROR;
    timer t;
    unsigned ts;

    // starts rotating the motor
    i_torque_control.enable_index_detection();

    while(counter < 15000)
    {
        upstream_control_data = i_torque_control.update_upstream_control_data(downstream_control_data.gpio_output);
        position = upstream_control_data.singleturn;
        angle = upstream_control_data.angle;
        hall_state = upstream_control_data.hall_state;

        // remembering the first element
        if (counter == 0)
        {
            if (motorcontrol_config.commutation_sensor == HALL_SENSOR)
                hall_state_old = hall_state;
        }

        // checking if hall ports are toggling
        // if they are not, hall_state variables will be zero and will signal an error
        if (motorcontrol_config.commutation_sensor == HALL_SENSOR)
        {
            if (hall_state & HALL_MASK_A && !(hall_state_old & HALL_MASK_A))
                ++hall_state_ch[A];
            if (hall_state & HALL_MASK_B && !(hall_state_old & HALL_MASK_B))
                ++hall_state_ch[B];
            if (hall_state & HALL_MASK_C && !(hall_state_old & HALL_MASK_C))
                ++hall_state_ch[C];

            hall_state_old = hall_state;
        }

        // finding the max angle which should be 4095 (12 bits resolution)
        // calculating the expected mean angle, point between mean and max, point between mean and 0
        if (angle > 4000 && !max_angle_found)
        {
            max_angle_found = 1;
            max_angle = angle;
            mean_angle = max_angle / 2;
            tq_angle = (max_angle + mean_angle) / 2;
            fq_angle = mean_angle / 2;
        }

        // calculating the real mean angle, point between mean and max, point between mean and 0
        // given threshold is 50
        if (max_angle_found)
        {
            if (angle > mean_angle - 50 && angle < mean_angle + 50)
                real_mean_angle = angle;
            if (angle > tq_angle - 50 && angle < tq_angle + 50)
                real_tq_angle = angle;
            if (angle > fq_angle - 50 && angle < fq_angle + 50)
                real_fq_angle = angle;
        }


        // finding the max pos which should be 65535 (16 bits resolution)
        // calculating the expected mean position, point between mean and max, point between mean and 0
        if (!max_pos_found && position > 60000)
        {
            max_pos_found = 1;
            max_pos = position;
            mean_pos = max_pos / 2;
            tq_pos = (max_pos + mean_pos) / 2;
            fq_pos = mean_pos / 2;
        }

        // calculating the real mean position, point between mean and max, point between mean and 0
        // given threshold is 5000
        if (max_pos_found)
        {
            if (position > mean_pos -5000 && position < mean_pos + 5000)
                real_mean_pos = position;
            if (position > tq_pos - 5000 && position < tq_pos + 5000)
                real_tq_pos = position;
            if (position > fq_pos - 5000 && position < fq_pos + 5000)
                real_fq_pos = position;
        }

        ++counter;


        t :> ts;
        if (motorcontrol_config.commutation_sensor == BISS_SENSOR)
            t when timerafter (ts + 1500*app_tile_usec) :> void;
        else
            t when timerafter (ts + 500*app_tile_usec) :> void;
    }

    // evaluating velocity readings by comparing velocity data from the sensor and computed velocity from position data from the sensor
    while (counter > 5000)
    {
        upstream_control_data = i_torque_control.update_upstream_control_data(downstream_control_data.gpio_output);
        position = upstream_control_data.singleturn;
        velocity = upstream_control_data.velocity;

        if (counter == 15000)
            old_position = position;

        // calculating velocity from position readings as simple derivative
        if (position != old_position)
        {
            if(position - old_position > -60000 && position - old_position < 60000)
            {
                difference_arr[index_d] = ((position - old_position) * (60000000/1000)) / 65535;
                index_d++;
                old_position = position;
            }
        }

        // mean average filter of velocity data from the sensor
        velocity_arr[index_v] = velocity;
        index_v++;
        if (index_v == 500)
        {
            index_v = 0;
            filter_vel = 0;
            for (int i = 0; i < 500; i++)
                filter_vel += velocity_arr[i];
            filter_vel /= 500;
            filter_vel = abs(filter_vel);
        }

        // mean average filter of computed velocity from the sensor
        if (index_d == 500)
        {
            index_d = 0;
            filter_diff = 0;
            for (int i = 0; i < 500; i++)
                filter_diff += difference_arr[i];
            filter_diff /= 500;
            filter_diff = abs(filter_diff);
        }

        --counter;
        t :> ts;
        t when timerafter (ts + 1000*app_tile_usec) :> void;
    }

    // comparison of position data
    if ((mean_pos-real_mean_pos < -5000 || mean_pos-real_mean_pos > 5000)
            || (tq_pos - real_tq_pos < -5000 || tq_pos - real_tq_pos > 5000)
            || (fq_pos - real_fq_pos < -5000 || fq_pos - real_fq_pos > 5000)
            || max_pos < 60000)
        error_sens = POS_ERR;

    // comparison of velocity data for low and high velocities
    if (filter_vel < 60 && filter_vel > -60)
    {
        if (filter_vel - filter_diff < -30 || filter_vel - filter_diff > 30)
            error_sens = SPEED_ERR;
    }
    else
    {
        if (filter_vel - filter_diff < -50 || filter_vel - filter_diff > 50)
            error_sens = SPEED_ERR;
    }

    // comparison of angle data
    if ((mean_angle-real_mean_angle < -50 || mean_angle-real_mean_angle > 50)
            || (tq_angle - real_tq_angle < -50 || tq_angle - real_tq_angle > 50)
            || (fq_angle - real_fq_angle < -50 || fq_angle - real_fq_angle > 50)
            || max_angle < 4000)
        error_sens = ANGLE_ERR;

    if (motorcontrol_config.commutation_sensor == HALL_SENSOR)
    {
        if (!hall_state_ch[A] || !hall_state_ch[B] || !hall_state_ch[C])
            error_sens = PORTS_ERR;
    }

    //    printf("%d\n", error_sens);
    //    printf("%d %d %d %d\n", max_pos, real_mean_pos, real_tq_pos, real_fq_pos);
    i_torque_control.disable_index_detection();
    i_torque_control.set_sensor_status(error_sens);
    return error_sens;
}

/*
 * detection of open phase prior to starting the motorcontrol
 * voltage difference between terminal a and terminals b,c (on the same potential) should establish electric circuit consisting of phase a in series with a branch consisting of parallel branches b and c
 */
int open_phase_detection_offline(client interface TorqueControlInterface i_torque_control,
        MotorcontrolConfig motorcontrol_config, DownstreamControlData &downstream_control_data,
        int app_tile_usec, int current_ratio, float * ret)
{
    UpstreamControlData upstream_control_data;
    float I[NR_PHASES] = { 0 };
    int refer[NR_PHASES] = {0, 20, 20, 20};
    unsigned counter = 1;
    float voltage = 0;
    float rated_curr = (float)motorcontrol_config.rated_current/1000;
    int error_phase = NO_ERROR;

    timer tmr;
    unsigned ts;

    i_torque_control.set_torque_control_disabled();
    i_torque_control.start_system_eval();
    i_torque_control.set_evaluation_references(refer[A], refer[B], refer[C]);

    // algorithm is applied until all currents become high enough, i.e. have enough information to make a conclusion that phases are not open
    while ((I[A] < 1|| I[B] < 1 || I[C] < 1) || (I[A] != (2*I[B]) || I[A] != (2*I[C])))
    {
        upstream_control_data = i_torque_control.update_upstream_control_data(downstream_control_data.gpio_output);

        I[B] = (float)upstream_control_data.I_b/(float)(current_ratio);
        I[C] = (float)upstream_control_data.I_c/(float)(current_ratio);
        I[A] = -(I[B]+I[C]);

        for(int i = 0; i < NR_PHASES; i++)
            if (I[i] < 0)
                I[i] = -I[i];

        // change terminal b and c voltages
        if (counter % 100 == 0)
        {
            refer[B] += 1;
            refer[C] += 1;
            i_torque_control.set_evaluation_references(refer[A], refer[B], refer[C]);
        }

        // break the algortihm if voltage on terminals goes over the max limit or currents went over 80 % raed current
        // at this moment we can conclude if the phase is opened
        // at such a high voltage at the terminals, if the current is below 10 % of rated current it can be said that the phase is opened
        // if non of the conditions here are met, it can be said that phases are not opened
        if (refer[B] > 90 || I[A] > 0.8*rated_curr || I[B] > 0.8*rated_curr || I[C] > 0.8*rated_curr)
        {
            if (I[A] < rated_curr/10)
                error_phase = A;
            else if (I[B] < rated_curr/10)
                error_phase = B;
            else if (I[C] < rated_curr/10)
                error_phase = C;
            break;
        }

        tmr :> ts;
        tmr when timerafter (ts + 1000*app_tile_usec) :> ts;

        ++counter;
    }

    //    printf("%.2f %.2f %.2f\n", I[A], I[B], I[C]);

    // calculation of phase resistance
    voltage =  (((float)refer[B] - refer[A])/100) * (float)upstream_control_data.V_dc / 2;
    if (error_phase == 0)
        *ret = voltage / I[B];

    i_torque_control.set_evaluation_references(0, 0, 0);

    return error_phase;
}

int special_brake_release(int &counter, int start_position, int actual_position, int range, int duration, int max_torque, MotionControlError &motion_control_error)
{
    int steps = 8;
    int phase_1 = (duration/3); //1000
    int phase_2 = duration;

    int target;
    if ( (actual_position-start_position) > range || (actual_position-start_position) < (-range)) //we moved more than half the range so the brake should be released
    {
        target = 0;
        counter= duration; //stop counter
    }
    else if (counter < phase_1)
    {
        int step = counter/(phase_1/steps);
        int sign = 1;
        if (step%2) {
            sign = -1;
        }
        target = ((counter-(phase_1/steps)*step)*max_torque*(step+1+(step+1)%2)*sign)/phase_1; //ramp to max torque of step
    }
    else if (counter < duration) //end:
    {
        steps = 4;
        int step = (counter-phase_1)/((phase_2-phase_1)/steps);
        int max_torque_step = max_torque*(step+1+(step+1)%2);
        int sign = 1;
        if (step%2) {
            sign = -1;
        }
        // ramp to max torque of step, we ramp faster and then limit the torque,
        // so we have some time when the maximum torque is applied
        target = (((counter-phase_1)-((phase_2-phase_1)/steps)*step)*max_torque_step*sign)/(((phase_2-phase_1)*6)/10);
        if (target > max_torque_step/steps)
            target = max_torque_step/steps;
        else if (target < -max_torque_step/steps)
            target = -max_torque_step/steps;
    }
    else if (counter == duration) //end:
    {
        target = 0; //stop
        if ((actual_position-start_position) < range && (actual_position-start_position) > (-range)) {
            // we didn't move enough, brake is probably not released
            motion_control_error = MOTION_CONTROL_BRAKE_NOT_RELEASED;
        }
    }

    counter++;

    return target;
}


/**
 * @brief Service to perform torque, velocity or position control.
 *        You will need a Motor Control Stack running parallel to this Service,
 *        have a look at Motor Control Service for more information.
 *
 *  Note: It is important to allocate this service in a different tile from the remaining Motor Control stack.
 *
 * @param motion_ctrl_config            Structure containing all parameters of motion control service
 * @param pos_velocity_control_config   Configuration for ttorque/velocity/position controllers.
 * @param i_torque_control Communication    interface to the Motor Control Service.
 * @param i_motion_control[3]               array of MotionControlInterfaces to communicate with upto 3 clients
 * @param i_update_brake                    Interface to update brake configuration in PWM service
 *
 * @return void
 *  */
void motion_control_service(MotionControlConfig &motion_ctrl_config,
        interface TorqueControlInterface client i_torque_control,
        interface MotionControlInterface server i_motion_control[3],client interface UpdateBrake i_update_brake)
{
    timer t;
    unsigned int ts;

    //proper task startup
    t :> ts;
    t when timerafter (ts + (1000*100*50)) :> void;

    unsigned time_start=0, time_start_old=0, time_loop=0, time_end=0, time_free=0, time_used=0;

    SecondOrderLPfilterParam torque_filter_param;
    second_order_LP_filter_init(motion_ctrl_config.filter, POSITION_CONTROL_LOOP_PERIOD, torque_filter_param);
    double filter_input=0.00, filter_output=0.00;

    int torque_measurement;
    int torque_buffer [8] = {0};
    int index = 0;

    // structure definition
    UpstreamControlData upstream_control_data;
    DownstreamControlData downstream_control_data = {0};

    PIDT1param velocity_control_pid_param;

    PIDT1param position_control_pid_param;

    LimitedTorquePosCtrl lt_pos_ctrl;


    // variable definition
    int torque_enable_flag = 0;
    int velocity_enable_flag = 0;
    int position_enable_flag = 0;

    int pos_control_mode = 0;

    int torque_ref_k = 0;
    double torque_ref_in_k=0, torque_ref_in_k_1n=0;

    double velocity_ref_k = 0;
    double velocity_ref_in_k=0, velocity_ref_in_k_1n=0;
    double velocity_k = 0.00;

    //cogging compensation
    CoggingTorqueParam ct_parameters;

    init_cogging_torque_parameters(ct_parameters, 10);

    VelCtrlAutoTuneParam velocity_auto_tune;

    downstream_control_data.velocity_cmd = 0;
    motion_ctrl_config.enable_velocity_auto_tuner = 0;

    double position_ref_in_k = 0.00;
    double position_ref_in_k_1n = 0.00;
    double position_ref_in_k_2n = 0.00;
    double position_ref_in_k_3n = 0.00;
    double position_k   = 0.00, position_k_1=0.00;

    PosCtrlAutoTuneParam pos_ctrl_auto_tune;

    motion_ctrl_config.step_amplitude_autotune  = AUTO_TUNE_STEP_AMPLITUDE;
    motion_ctrl_config.counter_max_autotune     = AUTO_TUNE_COUNTER_MAX   ;
    motion_ctrl_config.per_thousand_overshoot_autotune   = PER_THOUSAND_OVERSHOOT;

    // initialization of position control automatic tuning:
    motion_ctrl_config.position_control_autotune =0;

    init_pos_ctrl_autotune(pos_ctrl_auto_tune, motion_ctrl_config, CASCADED);
    //***********************************************************************************************

    MotionControlError motion_control_error = MOTION_CONTROL_NO_ERROR;

    //pos profiler
    ProfilerParam profiler_param;
    profiler_param.delta_T = ((double)POSITION_CONTROL_LOOP_PERIOD)/1000000.00;
    profiler_param.v_max = (double)(motion_ctrl_config.max_speed_profiler);
    profiler_param.acceleration_max = (double)(motion_ctrl_config.max_acceleration_profiler);
    profiler_param.deceleration_max = (double)(motion_ctrl_config.max_deceleration_profiler);
    profiler_param.torque_rate_max = (double)(motion_ctrl_config.max_torque_rate_profiler);
    profiler_param.resolution = (double)(motion_ctrl_config.resolution);

    float acceleration_monitor = 0;

    //position limiter
    int position_limit_reached = 0;
    int max_position_orig, min_position_orig;
    int max_position, min_position;
    //reverse position limits when polarity is inverted
    if (motion_ctrl_config.polarity == MOTION_POLARITY_INVERTED)
    {
        min_position = -motion_ctrl_config.max_pos_range_limit;
        max_position = -motion_ctrl_config.min_pos_range_limit;
    }
    else
    {
        min_position = motion_ctrl_config.min_pos_range_limit;
        max_position = motion_ctrl_config.max_pos_range_limit;
    }

    // initialization
    MotorcontrolConfig motorcontrol_config = i_torque_control.get_config();
    motion_ctrl_config.max_torque =motorcontrol_config.max_torque;
    int current_ratio = motorcontrol_config.current_ratio;

    //init brake config
    update_brake_configuration(motion_ctrl_config, i_torque_control);

    lt_position_control_reset(lt_pos_ctrl);
    lt_position_control_set_parameters(lt_pos_ctrl, motion_ctrl_config.max_motor_speed, motion_ctrl_config.resolution, motion_ctrl_config.moment_of_inertia,
            motion_ctrl_config.position_kp, motion_ctrl_config.position_ki, motion_ctrl_config.position_kd, motion_ctrl_config.position_integral_limit,
            motion_ctrl_config.max_torque, POSITION_CONTROL_LOOP_PERIOD);

    pid_init(velocity_control_pid_param);
    if(motion_ctrl_config.velocity_kp<0)            motion_ctrl_config.velocity_kp=0;
    if(motion_ctrl_config.velocity_kp>100000000)    motion_ctrl_config.velocity_kp=100000000;
    if(motion_ctrl_config.velocity_ki<0)            motion_ctrl_config.velocity_ki=0;
    if(motion_ctrl_config.velocity_ki>100000000)    motion_ctrl_config.velocity_ki=100000000;
    if(motion_ctrl_config.velocity_kd<0)            motion_ctrl_config.velocity_kd=0;
    if(motion_ctrl_config.velocity_kd>100000000)    motion_ctrl_config.velocity_kd=100000000;
    pid_set_parameters(
            (double)motion_ctrl_config.velocity_kp, (double)motion_ctrl_config.velocity_ki,
            (double)motion_ctrl_config.velocity_kd, (double)motion_ctrl_config.velocity_integral_limit,
            POSITION_CONTROL_LOOP_PERIOD, velocity_control_pid_param);

    pid_init(position_control_pid_param);
    if(motion_ctrl_config.position_kp<0)            motion_ctrl_config.position_kp=0;
    if(motion_ctrl_config.position_kp>100000000)    motion_ctrl_config.position_kp=100000000;
    if(motion_ctrl_config.position_ki<0)            motion_ctrl_config.position_ki=0;
    if(motion_ctrl_config.position_ki>100000000)    motion_ctrl_config.position_ki=100000000;
    if(motion_ctrl_config.position_kd<0)            motion_ctrl_config.position_kd=0;
    if(motion_ctrl_config.position_kd>100000000)    motion_ctrl_config.position_kd=100000000;
    pid_set_parameters((double)motion_ctrl_config.position_kp, (double)motion_ctrl_config.position_ki,
            (double)motion_ctrl_config.position_kd, (double)motion_ctrl_config.position_integral_limit,
            POSITION_CONTROL_LOOP_PERIOD, position_control_pid_param);

    // initialize GS controller params to zero
    gain_scheduling_init(gain_sch_param);
    // set GS controller params to default ones from configuration file
    if(motion_ctrl_config.position_kp_l<0)            motion_ctrl_config.position_kp_l=0;
    if(motion_ctrl_config.position_kp_l>100000000)    motion_ctrl_config.position_kp_l=100000000;
    if(motion_ctrl_config.position_ki_l<0)            motion_ctrl_config.position_ki_l=0;
    if(motion_ctrl_config.position_ki_l>100000000)    motion_ctrl_config.position_ki_l=100000000;
    if(motion_ctrl_config.position_kd_l<0)            motion_ctrl_config.position_kd_l=0;
    if(motion_ctrl_config.position_kd_l>100000000)    motion_ctrl_config.position_kd_l=100000000;
    if(motion_ctrl_config.position_kp_h<0)            motion_ctrl_config.position_kp_h=0;
    if(motion_ctrl_config.position_kp_h>100000000)    motion_ctrl_config.position_kp_h=100000000;
    if(motion_ctrl_config.position_ki_h<0)            motion_ctrl_config.position_ki_h=0;
    if(motion_ctrl_config.position_ki_h>100000000)    motion_ctrl_config.position_ki_h=100000000;
    if(motion_ctrl_config.position_kd_h<0)            motion_ctrl_config.position_kd_h=0;
    if(motion_ctrl_config.position_kd_h>100000000)    motion_ctrl_config.position_kd_h=100000000;
    if(motion_ctrl_config.velocity_kp_l<0)            motion_ctrl_config.velocity_kp_l=0;
    if(motion_ctrl_config.velocity_kp_l>100000000)    motion_ctrl_config.velocity_kp_l=100000000;
    if(motion_ctrl_config.velocity_ki_l<0)            motion_ctrl_config.velocity_ki_l=0;
    if(motion_ctrl_config.velocity_ki_l>100000000)    motion_ctrl_config.velocity_ki_l=100000000;
    if(motion_ctrl_config.velocity_kd_l<0)            motion_ctrl_config.velocity_kd_l=0;
    if(motion_ctrl_config.velocity_kd_l>100000000)    motion_ctrl_config.velocity_kd_l=100000000;
    if(motion_ctrl_config.velocity_kp_h<0)            motion_ctrl_config.velocity_kp_h=0;
    if(motion_ctrl_config.velocity_kp_h>100000000)    motion_ctrl_config.velocity_kp_h=100000000;
    if(motion_ctrl_config.velocity_ki_h<0)            motion_ctrl_config.velocity_ki_h=0;
    if(motion_ctrl_config.velocity_ki_h>100000000)    motion_ctrl_config.velocity_ki_h=100000000;
    if(motion_ctrl_config.velocity_kd_h<0)            motion_ctrl_config.velocity_kd_h=0;
    if(motion_ctrl_config.velocity_kd_h>100000000)    motion_ctrl_config.velocity_kd_h=100000000;
    if(motion_ctrl_config.velocity_lo_l<0)            motion_ctrl_config.velocity_lo_l=0;
    if(motion_ctrl_config.velocity_lo_l>motion_ctrl_config.max_motor_speed)  motion_ctrl_config.velocity_lo_l=motion_ctrl_config.max_motor_speed;
    if(motion_ctrl_config.velocity_hi_l<0)            motion_ctrl_config.velocity_hi_l=0;
    if(motion_ctrl_config.velocity_hi_l>motion_ctrl_config.max_motor_speed)  motion_ctrl_config.velocity_hi_l=motion_ctrl_config.max_motor_speed;

    gain_scheduling_set_param((double)motion_ctrl_config.position_kp_l, (double)motion_ctrl_config.position_ki_l, (double)motion_ctrl_config.position_kd_l,
            (double)motion_ctrl_config.position_kp_h, (double)motion_ctrl_config.position_ki_h, (double)motion_ctrl_config.position_kd_h,
            (double)motion_ctrl_config.velocity_kp_l, (double)motion_ctrl_config.velocity_ki_l, (double)motion_ctrl_config.velocity_kd_l,
            (double)motion_ctrl_config.velocity_kp_h, (double)motion_ctrl_config.velocity_ki_h, (double)motion_ctrl_config.velocity_kd_h,
            motion_ctrl_config.velocity_lo_l, motion_ctrl_config.velocity_hi_l, gain_sch_param);

    init_velocity_auto_tuner(velocity_auto_tune, motion_ctrl_config, TUNING_VELOCITY, SETTLING_TIME);

    downstream_control_data.position_cmd = 0;
    downstream_control_data.velocity_cmd = 0;
    downstream_control_data.torque_cmd   = 0;
    downstream_control_data.offset_torque = 0;

    upstream_control_data = i_torque_control.update_upstream_control_data(downstream_control_data.gpio_output);

    position_k  = ((double) upstream_control_data.position);
    position_k_1= position_k;

    /* read tile frequency
     * this needs to be after the first call to i_torque_control
     * so when we read it the frequency has already been changed by the torque controller
     */
    unsigned int app_tile_usec = USEC_STD;
    unsigned ctrlReadData;
    read_sswitch_reg(get_local_tile_id(), 8, ctrlReadData);
    if(ctrlReadData == 1) {
        app_tile_usec = USEC_FAST;
    }

    int error_phase = NO_ERROR;
    float resist = 0;
    sensor_fault error_sens = NO_ERROR;
    unsigned qei_ctr = 100000;
    float curr_threshold = (float)motorcontrol_config.rated_current /1000 / 10;
    unsigned measurement = 0;
    int ftr = 0;
    float rms[NR_PHASES] = { 0 };
    int sum_sq[NR_PHASES] = { 0 }, sum[NR_PHASES] = { 0 }, detect[NR_PHASES] = { 0 };
    int detect_low[NR_PHASES] = { 0 }, phase_cur[NR_PHASES] = { 0 };

    //QEI index calibration
    if (motorcontrol_config.commutation_sensor == QEI_SENSOR)
    {
        printf("Find encoder index \n");
        int index_found = 0;
        i_torque_control.enable_index_detection();
        while (!index_found)
        {
            upstream_control_data = i_torque_control.update_upstream_control_data(downstream_control_data.gpio_output);
            index_found = upstream_control_data.qei_index_found;
            if (qei_ctr == 0)
            {
                error_sens = POS_ERR;
                index_found = 1;
            }

            --qei_ctr;
        }

        if (error_sens == NO_ERROR)
            printf("Incremental encoder index found\n");
        else
            printf("Incremental encoder index not found \n Check if the sensor is connected\n\n");

        i_torque_control.disable_index_detection();
    }

    //brake
    int special_brake_release_counter = BRAKE_RELEASE_DURATION+1;
    int special_brake_release_initial_position = 0;
    int special_brake_release_torque = 0;
    int brake_shutdown_counter = 0;
    i_torque_control.set_safe_torque_off_enabled();
    i_torque_control.set_brake_status(DISABLE_BRAKE);

    printstr(">>   SOMANET POSITION CONTROL SERVICE STARTING...\n");

    t :> ts;

    while(1)
    {
#pragma ordered
        select
        {
        case t when timerafter(ts + app_tile_usec * POSITION_CONTROL_LOOP_PERIOD) :> ts:

                time_start_old = time_start;
                t :> time_start;
                time_loop = time_start - time_start_old;
                time_free = time_start - time_end;

                upstream_control_data = i_torque_control.update_upstream_control_data(downstream_control_data.gpio_output);


                if (motion_ctrl_config.enable_compensation_recording)
                {
                    downstream_control_data.velocity_cmd = ct_parameters.velocity_reference;
                }

                velocity_ref_k    = ((double) downstream_control_data.velocity_cmd);
                velocity_k        = ((double) upstream_control_data.velocity);

                // torque control
                if(torque_enable_flag == 1)
                {
                    if(motion_ctrl_config.enable_profiler==1)
                    {
                        torque_ref_in_k = torque_profiler(((double)(downstream_control_data.torque_cmd)), torque_ref_in_k_1n, profiler_param, POSITION_CONTROL_LOOP_PERIOD);
                        torque_ref_in_k_1n = torque_ref_in_k;
                        torque_ref_k = ((int)(torque_ref_in_k));
                    }
                    else
                    {
                        torque_ref_k = downstream_control_data.torque_cmd;
                    }
                }
                else if (velocity_enable_flag == 1)// velocity control
                {
                    if(motion_ctrl_config.enable_velocity_auto_tuner == 1)
                    {
                        if(velocity_auto_tune.enable == 0)
                        {
                            init_velocity_auto_tuner(velocity_auto_tune, motion_ctrl_config, TUNING_VELOCITY, SETTLING_TIME);
                            motion_ctrl_config.velocity_kp = KP_VELOCITY_TUNING;
                            motion_ctrl_config.velocity_ki = 0;
                            motion_ctrl_config.velocity_kd = 0;
                            pid_init(velocity_control_pid_param);
                            pid_set_parameters((double)motion_ctrl_config.velocity_kp, (double)motion_ctrl_config.velocity_ki, (double)motion_ctrl_config.velocity_kd, (double)motion_ctrl_config.velocity_integral_limit, POSITION_CONTROL_LOOP_PERIOD, velocity_control_pid_param);

                            velocity_auto_tune.enable = 1;
                        }

                        velocity_controller_auto_tune(velocity_auto_tune, motion_ctrl_config, velocity_ref_in_k, velocity_k, POSITION_CONTROL_LOOP_PERIOD);
                        torque_ref_k = pid_update(velocity_ref_in_k, velocity_k, POSITION_CONTROL_LOOP_PERIOD, velocity_control_pid_param);
                        if(velocity_auto_tune.enable == 0)
                        {
                            torque_ref_k=0;
                            torque_enable_flag   =0;
                            velocity_enable_flag =0;
                            position_enable_flag =0;
                            i_torque_control.set_torque_control_disabled();

                            printf("kp:%i ki:%i kd:%i \n",  ((int)(velocity_auto_tune.kp)), ((int)(velocity_auto_tune.ki)), ((int)(velocity_auto_tune.kd)));
                        }
                    }
                    else if(motion_ctrl_config.enable_profiler==1)
                    {
                        velocity_ref_in_k = velocity_profiler(velocity_ref_k, velocity_ref_in_k_1n, velocity_k, profiler_param, POSITION_CONTROL_LOOP_PERIOD);
                        velocity_ref_in_k_1n = velocity_ref_in_k;
                        torque_ref_k = pid_update(velocity_ref_in_k, velocity_k, POSITION_CONTROL_LOOP_PERIOD, velocity_control_pid_param);
                    }
                    else if(motion_ctrl_config.enable_profiler==0)
                    {
                        velocity_ref_in_k = velocity_ref_k;
                        torque_ref_k = pid_update(velocity_ref_in_k, velocity_k, POSITION_CONTROL_LOOP_PERIOD, velocity_control_pid_param);
                    }
                    //Recording function for the cogging torque
                    if (motion_ctrl_config.enable_compensation_recording)
                    {
                        downstream_control_data.velocity_cmd = ct_parameters.velocity_reference;
                        if (ct_parameters.delay_counter < (1000 * 1000 * POSITION_CONTROL_LOOP_PERIOD))
                        {
                            if (!ct_parameters.torque_recording_started)
                            {
                                ct_parameters.torque_recording_started = 1;
                                ct_parameters.count_start = upstream_control_data.position;

                                if (ct_parameters.velocity_reference < 0)
                                {
                                    if (ct_parameters.rotation_sign != -1)
                                    {
                                        ct_parameters.back_and_forth++;
                                    }
                                    ct_parameters.rotation_sign = -1;
                                }
                                else
                                {
                                    if (ct_parameters.rotation_sign != 1)
                                    {
                                        ct_parameters.back_and_forth++;
                                    }
                                    ct_parameters.rotation_sign = 1;
                                }
                            }
                            if (((upstream_control_data.position - ct_parameters.count_start) * ct_parameters.rotation_sign < ct_parameters.number_turns * motion_ctrl_config.resolution) || ct_parameters.remaining_cells != 0)
                            {
                                if (upstream_control_data.singleturn % ct_parameters.position_step)
                                {
                                    int index = upstream_control_data.singleturn / ct_parameters.position_step;
                                    ct_parameters.torque_recording[index] += torque_measurement;
                                    if (ct_parameters.counter_average[index] == 0)
                                    {
                                        ct_parameters.remaining_cells--;
                                    }
                                    ct_parameters.counter_average[index] ++;
                                }
                            }
                            else {
                                printf("Measurement done\n");
                                for (int i = 0; i < COGGING_TORQUE_ARRAY_SIZE ; i++)
                                {
                                    ct_parameters.torque_recording[i] /= ct_parameters.counter_average[i];
                                    ct_parameters.counter_average[i] = 0;
                                }


                                motorcontrol_config = i_torque_control.get_config();
                                if (ct_parameters.back_and_forth == 1)
                                {
                                    printf("\nFirst turn done\n");
                                    for (int i = 0; i < COGGING_TORQUE_ARRAY_SIZE; i++)
                                    {
                                        motorcontrol_config.torque_offset[i] = ct_parameters.torque_recording[i];
                                        ct_parameters.torque_recording[i]= 0;
                                    }
                                    ct_parameters.velocity_reference = -ct_parameters.velocity_reference;
                                }
                                else if(ct_parameters.back_and_forth == 2)
                                {
                                    printf("\nSecond turn done\n");
                                    for (int i = 0; i < COGGING_TORQUE_ARRAY_SIZE; i++)
                                    {
                                        motorcontrol_config.torque_offset[i] += ct_parameters.torque_recording[i];
                                        motorcontrol_config.torque_offset[i] /= 2;
                                        ct_parameters.torque_recording[i]= 0;
                                    }
                                    ct_parameters.rotation_sign = 0;
                                    motion_ctrl_config.enable_compensation_recording = 0;
                                    velocity_ref_k = 0;
                                }
                                ct_parameters.remaining_cells = COGGING_TORQUE_ARRAY_SIZE;
                                i_torque_control.set_config(motorcontrol_config);
                                ct_parameters.torque_recording_started = 0;
                                ct_parameters.delay_counter = 0;
                                i_torque_control.set_torque_control_enabled();
                                i_torque_control.set_brake_status(DISABLE_BRAKE);

                                if(ct_parameters.back_and_forth == 2)
                                {
                                    ct_parameters.back_and_forth = 0;
                                    torque_enable_flag   =0;
                                    velocity_enable_flag =0;
                                    position_enable_flag =0;
                                    i_torque_control.set_torque_control_disabled();
                                }
                            }
                        }
                        else
                        {
                            ct_parameters.delay_counter ++;
                        }

                    }
                }
                else if (position_enable_flag == 1)// position control
                {
                    //brake shutdown delay, don't update target position
                    if (brake_shutdown_counter > 0)
                    {
                        brake_shutdown_counter--;
                        if (brake_shutdown_counter == 0)
                        {
                            torque_enable_flag   =0;
                            velocity_enable_flag =0;
                            position_enable_flag =0;
                            i_torque_control.set_torque_control_disabled();
                        }
                    }
                    //profiler enabled, set target position
                    else if (motion_ctrl_config.enable_profiler)
                    {
                        position_ref_in_k = pos_profiler((
                                (double) downstream_control_data.position_cmd),
                                position_ref_in_k_1n,
                                position_ref_in_k_2n,
                                position_k,
                                profiler_param);

                        acceleration_monitor = (position_ref_in_k - (2 * position_ref_in_k_1n) + position_ref_in_k_2n)/(POSITION_CONTROL_LOOP_PERIOD * POSITION_CONTROL_LOOP_PERIOD);
                        position_ref_in_k_3n = position_ref_in_k_2n;
                        position_ref_in_k_2n = position_ref_in_k_1n;
                        position_ref_in_k_1n = position_ref_in_k;
                    }
                    //use direct target position
                    else
                    {
                        position_ref_in_k = (double) downstream_control_data.position_cmd;
                    }
                    position_k_1= position_k;
                    position_k  = ((double) upstream_control_data.position);

                    if (pos_control_mode == POS_PID_CONTROLLER)
                    {
                        torque_ref_k = pid_update(position_ref_in_k, position_k, POSITION_CONTROL_LOOP_PERIOD, position_control_pid_param);
                    }
                    else if (pos_control_mode == POS_PID_VELOCITY_CASCADED_CONTROLLER)
                    {
                        if(motion_ctrl_config.position_control_autotune == 1)
                        {
                            if(pos_ctrl_auto_tune.activate==0)
                                init_pos_ctrl_autotune(pos_ctrl_auto_tune, motion_ctrl_config, CASCADED);

                            pos_ctrl_autotune(pos_ctrl_auto_tune, motion_ctrl_config, position_k);

                            if(motion_ctrl_config.position_control_autotune == 0)
                            {
                                printf("TUNING ENDED: \n");
                                printf("kp:%i ki:%i kd:%i kl:%d \n",  motion_ctrl_config.position_kp, motion_ctrl_config.position_ki, motion_ctrl_config.position_kd, motion_ctrl_config.position_integral_limit);
                            }

                            if(pos_ctrl_auto_tune.counter==0)
                            {
                                pid_init(velocity_control_pid_param);
                                pid_init(position_control_pid_param);

                                pid_set_parameters((double)motion_ctrl_config.velocity_kp, (double)motion_ctrl_config.velocity_ki, (double)motion_ctrl_config.velocity_kd, (double)motion_ctrl_config.velocity_integral_limit, POSITION_CONTROL_LOOP_PERIOD, velocity_control_pid_param);
                                pid_set_parameters((double)motion_ctrl_config.position_kp, (double)motion_ctrl_config.position_ki, (double)motion_ctrl_config.position_kd, (double)motion_ctrl_config.position_integral_limit, POSITION_CONTROL_LOOP_PERIOD, position_control_pid_param);
                            }


                            position_ref_in_k    = pos_profiler(pos_ctrl_auto_tune.position_ref, position_ref_in_k_1n, position_ref_in_k_2n, position_k, profiler_param);
                            position_ref_in_k_2n = position_ref_in_k_1n;
                            position_ref_in_k_1n = position_ref_in_k;

                            velocity_ref_k =pid_update(position_ref_in_k, position_k, POSITION_CONTROL_LOOP_PERIOD, position_control_pid_param);
                            if(velocity_ref_k> motion_ctrl_config.max_motor_speed) velocity_ref_k = motion_ctrl_config.max_motor_speed;
                            if(velocity_ref_k<-motion_ctrl_config.max_motor_speed) velocity_ref_k =-motion_ctrl_config.max_motor_speed;
                            torque_ref_k   =pid_update(velocity_ref_k   , velocity_k, POSITION_CONTROL_LOOP_PERIOD, velocity_control_pid_param);
                        }
                        else
                        {
                            velocity_ref_k =pid_update(position_ref_in_k, position_k, POSITION_CONTROL_LOOP_PERIOD, position_control_pid_param);
                            if(velocity_ref_k> motion_ctrl_config.max_motor_speed) velocity_ref_k = motion_ctrl_config.max_motor_speed;
                            if(velocity_ref_k<-motion_ctrl_config.max_motor_speed) velocity_ref_k =-motion_ctrl_config.max_motor_speed;
                            torque_ref_k   =pid_update(velocity_ref_k   , velocity_k, POSITION_CONTROL_LOOP_PERIOD, velocity_control_pid_param);
                        }
                    }
                    else if (pos_control_mode == LT_POSITION_CONTROLLER)
                    {

                        if(motion_ctrl_config.position_control_autotune == 1)
                        {
                            if(pos_ctrl_auto_tune.activate==0)
                                init_pos_ctrl_autotune(pos_ctrl_auto_tune, motion_ctrl_config, LIMITED_TORQUE);

                            pos_ctrl_autotune(pos_ctrl_auto_tune, motion_ctrl_config, position_k);

                            if(motion_ctrl_config.position_control_autotune == 0)
                            {
                                if(pos_ctrl_auto_tune.active_step==UNSUCCESSFUL)
                                    printf("TUNING UNSECCESSFUL \n");
                                else
                                {
                                    printf("TUNING ENDED \n");
                                    printf("kp:%i ki:%i kd:%i kl:%d J:%d\n",  motion_ctrl_config.position_kp, motion_ctrl_config.position_ki, motion_ctrl_config.position_kd, motion_ctrl_config.position_integral_limit, motion_ctrl_config.moment_of_inertia);
                                }
                            }

                            if(pos_ctrl_auto_tune.counter==0)
                            {
                                lt_position_control_reset(lt_pos_ctrl);
                                lt_position_control_set_parameters(lt_pos_ctrl, motion_ctrl_config.max_motor_speed, motion_ctrl_config.resolution, motion_ctrl_config.moment_of_inertia,
                                        motion_ctrl_config.position_kp, motion_ctrl_config.position_ki, motion_ctrl_config.position_kd, motion_ctrl_config.position_integral_limit,
                                        motion_ctrl_config.max_torque, POSITION_CONTROL_LOOP_PERIOD);
                            }

                            position_ref_in_k    = pos_profiler(pos_ctrl_auto_tune.position_ref, position_ref_in_k_1n, position_ref_in_k_2n, position_k, profiler_param);
                            position_ref_in_k_2n = position_ref_in_k_1n;
                            position_ref_in_k_1n = position_ref_in_k;

                            torque_ref_k = update_lt_position_control(lt_pos_ctrl, position_ref_in_k, position_k_1, position_k);
                        }
                        else
                        {
                            torque_ref_k = update_lt_position_control(lt_pos_ctrl, position_ref_in_k, position_k_1, position_k);
                        }
                    }
                }

                //brake release, override target torque if we are in brake release
                if (special_brake_release_counter <= BRAKE_RELEASE_DURATION)
                {
                    torque_ref_k = special_brake_release(special_brake_release_counter, special_brake_release_initial_position, upstream_control_data.position,\
                            BRAKE_RELEASE_THRESHOLD, BRAKE_RELEASE_DURATION, special_brake_release_torque, motion_control_error);
                }

                // check error code
                if (motion_control_error != MOTION_CONTROL_NO_ERROR) {
                    i_torque_control.set_brake_status(DISABLE_BRAKE);
                    torque_enable_flag   =0;
                    velocity_enable_flag =0;
                    position_enable_flag =0;
                    i_torque_control.set_torque_control_disabled();
                }

                //position limit check
                if (upstream_control_data.position > max_position || upstream_control_data.position < min_position)
                {
                    //disable everything
                    torque_enable_flag = 0;
                    position_enable_flag = 0;
                    velocity_enable_flag = 0;
                    i_torque_control.set_brake_status(DISABLE_BRAKE);
                    i_torque_control.set_torque_control_disabled();
                    printstr("*** Position Limit Reached ***\n");
                    //store original limits, with threshold if possible
                    if (position_limit_reached == 0)
                    {
                        position_limit_reached = 1;
                        if (max_position-min_position > 2*POSITION_LIMIT_THRESHOLD)
                        {
                            max_position_orig = max_position-POSITION_LIMIT_THRESHOLD;
                            min_position_orig = min_position+POSITION_LIMIT_THRESHOLD;
                        }
                        else
                        {
                            max_position_orig = max_position;
                            min_position_orig = min_position;
                        }
                    }
                    //increase limit by threshold
                    max_position += POSITION_LIMIT_THRESHOLD;
                    min_position -= POSITION_LIMIT_THRESHOLD;
                }
                //test if we moved back inside the original limits, then restore the position limits
                else if (position_limit_reached == 1 && upstream_control_data.position < max_position_orig && upstream_control_data.position > min_position_orig)
                {
                    if (motion_ctrl_config.polarity == MOTION_POLARITY_INVERTED) {
                        min_position = -motion_ctrl_config.max_pos_range_limit;
                        max_position = -motion_ctrl_config.min_pos_range_limit;
                    } else {
                        min_position = motion_ctrl_config.min_pos_range_limit;
                        max_position = motion_ctrl_config.max_pos_range_limit;
                    }
                    position_limit_reached = 0;
                }

                torque_ref_k += (double)(downstream_control_data.offset_torque);

                //torque limit check
                if(torque_ref_k > motion_ctrl_config.max_torque)
                    torque_ref_k = motion_ctrl_config.max_torque;
                else if (torque_ref_k < (-motion_ctrl_config.max_torque))
                    torque_ref_k = (-motion_ctrl_config.max_torque);

                filter_input  =  ((double)(torque_ref_k));
                filter_output = second_order_LP_filter_update(&filter_input, torque_filter_param);

                if(motion_ctrl_config.filter>0)
                    i_torque_control.set_torque(((int)(filter_output))); // use the filter only if the filter cut-off frequency is bigger/equal to zero, otherwise, do not use filter
                else
                    i_torque_control.set_torque(((int)(torque_ref_k)));

                torque_measurement = filter(torque_buffer, index, 8, torque_ref_k);

                switch (error_sens)
                {
                    case POS_ERR:
                        upstream_control_data.sensor_error = SENSOR_POSITION_FAULT;
                        break;
                    case SPEED_ERR:
                        upstream_control_data.sensor_error = SENSOR_SPEED_FAULT;
                        break;
                    case ANGLE_ERR:
                        if (motorcontrol_config.commutation_sensor == HALL_SENSOR)
                            upstream_control_data.sensor_error = SENSOR_HALL_FAULT;
                        else
                            upstream_control_data.sensor_error = SENSOR_INCREMENTAL_FAULT;
                        break;
                    case PORTS_ERR:
                        upstream_control_data.sensor_error = SENSOR_HALL_FAULT;
                        break;
                }

                /*
                 * open phase online detection
                 */
                if(motion_ctrl_config.enable_open_phase_detection)
                {
                    if(error_phase == NO_ERROR)
                    {
                        for (int i = A; i < NR_PHASES; i++)
                        {
                            switch (i)
                            {
                            case A:
                                phase_cur[A] = -(upstream_control_data.I_b+upstream_control_data.I_c);
                                break;
                            case B:
                                phase_cur[B] = upstream_control_data.I_b;
                                break;
                            case C:
                                phase_cur[C] = upstream_control_data.I_c;
                                break;
                            }

                            // observing abs value
                            if (phase_cur[i] < 0)
                                phase_cur[i] = -phase_cur[i];

                            // calculating sum and sum of squares for currents
                            sum[i] += phase_cur[i];
                            sum_sq[i] += phase_cur[i]*phase_cur[i];
                        }

                        ftr++;
                        // every 33 ms rms value is calculated
                        if (ftr > 1 && ftr % 100 == 0)
                        {
                            ++measurement;
                            float mean[NR_PHASES], sd[NR_PHASES];
                            // calculating mean, standard deviation and rms value
                            for (int i = A; i < NR_PHASES; i++)
                            {
                                mean[i] = (float)sum[i] / ftr;
                                sd[i] = ((float)sum_sq[i] - sum[i]*sum[i]/ftr)/(ftr-1);
                                rms[i] += mean[i] + sqrt(sd[i]);
                            }

                            for (int i = A; i < NR_PHASES; i++)
                                rms[i] = rms[i] / current_ratio;

                            // if velocity is high
                            // if three times it's detected that there is high difference between phase currents, phase is opened
                            if (abs(upstream_control_data.velocity) > 50)
                            {
                                if (rms[A] < curr_threshold && rms[B] > 8 * rms[A] && rms[C] > 8 * rms[A])
                                {
                                    detect[A]++;

                                    if (detect[A] == 3)
                                    {
                                        error_phase = A;
                                        i_torque_control.set_brake_status(0);
                                        if (motion_ctrl_config.brake_release_delay != 0 && position_enable_flag == 1)
                                        {
                                            brake_shutdown_counter = motion_ctrl_config.brake_release_delay;
                                        }
                                        else
                                        {
                                            torque_enable_flag   =0;
                                            velocity_enable_flag =0;
                                            position_enable_flag =0;
                                            i_torque_control.set_torque_control_disabled();
                                        }
                                    }
                                }

                                if (rms[B] < curr_threshold && rms[A] > 8 * rms[B] && rms[C] > 8 * rms[B])
                                {
                                    detect[B]++;

                                    if (detect[B] == 3)
                                    {
                                        error_phase = B;
                                        i_torque_control.set_brake_status(0);
                                        if (motion_ctrl_config.brake_release_delay != 0 && position_enable_flag == 1)
                                        {
                                            brake_shutdown_counter = motion_ctrl_config.brake_release_delay;
                                        }
                                        else
                                        {
                                            torque_enable_flag   =0;
                                            velocity_enable_flag =0;
                                            position_enable_flag =0;
                                            i_torque_control.set_torque_control_disabled();
                                        }
                                    }
                                }

                                if (rms[C] < curr_threshold && rms[A] > 8 * rms[C] && rms[B] > 8 * rms[C])
                                {
                                    detect[C]++;

                                    if (detect[C] == 3)
                                    {
                                        error_phase = C;
                                        i_torque_control.set_brake_status(0);
                                        if (motion_ctrl_config.brake_release_delay != 0 && position_enable_flag == 1)
                                        {
                                            brake_shutdown_counter = motion_ctrl_config.brake_release_delay;
                                        }
                                        else
                                        {
                                            torque_enable_flag   =0;
                                            velocity_enable_flag =0;
                                            position_enable_flag =0;
                                            i_torque_control.set_torque_control_disabled();
                                        }
                                    }
                                }
                            }
                            else if (abs(upstream_control_data.velocity) > 5)// if velocity is low
                            {
                                if (abs(upstream_control_data.computed_torque) > 10)
                                {
                                    if (rms[A] < curr_threshold && rms[B] > 4 * rms[A] && rms[C] > 4 * rms[A])
                                    {
                                        detect_low[A]++;
                                        if (detect_low[A] == 2)
                                        {
                                            error_phase = A;
                                            i_torque_control.set_brake_status(0);
                                            if (motion_ctrl_config.brake_release_delay != 0 && position_enable_flag == 1)
                                            {
                                                brake_shutdown_counter = motion_ctrl_config.brake_release_delay;
                                            }
                                            else
                                            {
                                                torque_enable_flag   =0;
                                                velocity_enable_flag =0;
                                                position_enable_flag =0;
                                                i_torque_control.set_torque_control_disabled();
                                            }
                                        }
                                    }
                                    else
                                        detect_low[A] = 0;

                                    if (rms[B] < curr_threshold && rms[A] > 4 * rms[B] && rms[C] > 4 * rms[B])
                                    {
                                        detect_low[B]++;
                                        if (detect_low[B] == 2)
                                        {
                                            error_phase = B;
                                            i_torque_control.set_brake_status(0);
                                            if (motion_ctrl_config.brake_release_delay != 0 && position_enable_flag == 1)
                                            {
                                                brake_shutdown_counter = motion_ctrl_config.brake_release_delay;
                                            }
                                            else
                                            {
                                                torque_enable_flag   =0;
                                                velocity_enable_flag =0;
                                                position_enable_flag =0;
                                                i_torque_control.set_torque_control_disabled();
                                            }
                                        }
                                    }
                                    else
                                        detect_low[B] = 0;

                                    if (rms[C] < curr_threshold && rms[A] > 4 * rms[C] && rms[B] > 4 * rms[C])
                                    {
                                        detect_low[C]++;
                                        if (detect_low[C] == 2)
                                        {
                                            error_phase = C;
                                            i_torque_control.set_brake_status(0);
                                            if (motion_ctrl_config.brake_release_delay != 0 && position_enable_flag == 1)
                                            {
                                                brake_shutdown_counter = motion_ctrl_config.brake_release_delay;
                                            }
                                            else
                                            {
                                                torque_enable_flag   =0;
                                                velocity_enable_flag =0;
                                                position_enable_flag =0;
                                                i_torque_control.set_torque_control_disabled();
                                            }
                                        }
                                    }
                                    else
                                        detect_low[C] = 0;
                                }
                            }

                            // every 20 measurements reset detection flag
                            if(measurement == 20)
                            {
                                measurement = 0;
                                for (int i = A; i < NR_PHASES; i++)
                                    detect[i] = 0;
                            }

                            ftr = 0;
                            for (int i = A; i < NR_PHASES; i++)
                            {
                                sum[i] = 0;
                                sum_sq[i] = 0;
                                rms[i] = 0;
                            }
                            //                            printf("%.2f %.2f %.2f\n", rms[A], rms[B], rms[C]);
                        }
                    }
                }

                switch (error_phase)
                {
                    case A:
                        upstream_control_data.error_status = PHASE_FAILURE_L1;
                        break;
                    case B:
                        upstream_control_data.error_status = PHASE_FAILURE_L2;
                        break;
                    case C:
                        upstream_control_data.error_status = PHASE_FAILURE_L3;
                        break;
                }

#ifdef XSCOPE_POSITION_CTRL
                xscope_int(VELOCITY, upstream_control_data.velocity);
                xscope_int(POSITION, upstream_control_data.position);
                xscope_int(VELOCITY_SECONDARY, upstream_control_data.secondary_velocity);
                xscope_int(POSITION_SECONDARY, upstream_control_data.secondary_position);
                xscope_int(TORQUE,   upstream_control_data.computed_torque);
                xscope_int(POSITION_CMD, downstream_control_data.position_cmd);
                xscope_int(VELOCITY_CMD, downstream_control_data.velocity_cmd);
                xscope_int(TORQUE_CMD, torque_ref_k);
                xscope_int(FAULT_CODE, upstream_control_data.error_status*1000);
                xscope_int(SENSOR_ERROR_X100, upstream_control_data.sensor_error*100);
#endif

#ifdef XSCOPE_ANALOGUE_MEASUREMENT
                xscope_int(V_DC, upstream_control_data.V_dc);
                xscope_int(I_DC, upstream_control_data.analogue_input_b_2);
                xscope_int(TEMPERATURE, (upstream_control_data.temperature/motorcontrol_config.temperature_ratio));
                xscope_int(I_A, -(upstream_control_data.I_b+upstream_control_data.I_c));
                xscope_int(I_B, upstream_control_data.I_b);
                xscope_int(I_C, upstream_control_data.I_c);
                xscope_int(AI_A1, upstream_control_data.analogue_input_a_1);
                xscope_int(AI_A2, upstream_control_data.analogue_input_a_2);
                xscope_int(AI_B1, upstream_control_data.analogue_input_b_1);
                xscope_int(AI_B2, upstream_control_data.analogue_input_b_2);
#endif

                if((time_used/app_tile_usec)>(POSITION_CONTROL_LOOP_PERIOD-5))
                {
                    printf("TIMING ERR \n");
                }

                t :> time_end;
                time_used = time_end - time_start;

                break;

        case i_motion_control[int i].disable():

                i_torque_control.set_brake_status(DISABLE_BRAKE);
                if (motion_ctrl_config.brake_release_delay != 0 && position_enable_flag == 1)
                {
                    brake_shutdown_counter = motion_ctrl_config.brake_release_delay;
                }
                else
                {
                    torque_enable_flag   =0;
                    velocity_enable_flag =0;
                    position_enable_flag =0;
                    motion_ctrl_config.enable_compensation_recording = 0;
                    init_cogging_torque_parameters(ct_parameters, 10);
                    i_torque_control.set_torque_control_disabled();

                }

                motion_ctrl_config.position_control_autotune =0;
                pos_ctrl_auto_tune.activate=0;

                break;

        case i_motion_control[int i].enable_position_ctrl(int in_pos_control_mode):

                torque_enable_flag   =0;
                velocity_enable_flag =0;
                position_enable_flag =1;

                pos_control_mode = in_pos_control_mode;

                //set current position as target
                downstream_control_data.position_cmd = upstream_control_data.position;
                position_k        = ((double) upstream_control_data.position);
                position_ref_in_k_1n = ((double) upstream_control_data.position);
                position_ref_in_k_2n = ((double) upstream_control_data.position);
                downstream_control_data.offset_torque = 0;

                //reset pid
                lt_position_control_reset(lt_pos_ctrl);
                lt_position_control_set_parameters(lt_pos_ctrl, motion_ctrl_config.max_motor_speed, motion_ctrl_config.resolution, motion_ctrl_config.moment_of_inertia,
                        motion_ctrl_config.position_kp, motion_ctrl_config.position_ki, motion_ctrl_config.position_kd, motion_ctrl_config.position_integral_limit,
                        motion_ctrl_config.max_torque, POSITION_CONTROL_LOOP_PERIOD);
                pid_reset(position_control_pid_param);

                enable_motorcontrol(motion_ctrl_config, i_torque_control, upstream_control_data.position, special_brake_release_counter, special_brake_release_initial_position, special_brake_release_torque, motion_control_error);

                //start control loop just after
                t :> ts;
                ts = ts - USEC_STD * POSITION_CONTROL_LOOP_PERIOD;

                break;

        case i_motion_control[int i].enable_velocity_ctrl(void):

                torque_enable_flag   =0;
                velocity_enable_flag =1;
                position_enable_flag =0;

                downstream_control_data.velocity_cmd = 0;
                downstream_control_data.offset_torque = 0;

                pid_reset(velocity_control_pid_param);

                pid_init(velocity_control_pid_param);
                if(motion_ctrl_config.velocity_kp<0)            motion_ctrl_config.velocity_kp=0;
                if(motion_ctrl_config.velocity_kp>100000000)    motion_ctrl_config.velocity_kp=100000000;
                if(motion_ctrl_config.velocity_ki<0)            motion_ctrl_config.velocity_ki=0;
                if(motion_ctrl_config.velocity_ki>100000000)    motion_ctrl_config.velocity_ki=100000000;
                if(motion_ctrl_config.velocity_kd<0)            motion_ctrl_config.velocity_kd=0;
                if(motion_ctrl_config.velocity_kd>100000000)    motion_ctrl_config.velocity_kd=100000000;
                pid_set_parameters(
                        (double)motion_ctrl_config.velocity_kp, (double)motion_ctrl_config.velocity_ki,
                        (double)motion_ctrl_config.velocity_kd, (double)motion_ctrl_config.velocity_integral_limit,
                        POSITION_CONTROL_LOOP_PERIOD, velocity_control_pid_param);

                enable_motorcontrol(motion_ctrl_config, i_torque_control, upstream_control_data.position, special_brake_release_counter, special_brake_release_initial_position, special_brake_release_torque, motion_control_error);

                //start control loop just after
                t :> ts;
                ts = ts - USEC_STD * POSITION_CONTROL_LOOP_PERIOD;
                break;

        case i_motion_control[int i].enable_torque_ctrl():
                torque_enable_flag   =1;
                velocity_enable_flag =0;
                position_enable_flag =0;

                downstream_control_data.torque_cmd = 0;
                downstream_control_data.offset_torque = 0;

                enable_motorcontrol(motion_ctrl_config, i_torque_control, upstream_control_data.position, special_brake_release_counter, special_brake_release_initial_position, special_brake_release_torque, motion_control_error);

                //start control loop just after
                t :> ts;
                ts = ts - USEC_STD * POSITION_CONTROL_LOOP_PERIOD;

                break;

        case i_motion_control[int i].set_motion_control_config(MotionControlConfig in_config):
                //check if we need to update max/min pos limits
                if (in_config.max_pos_range_limit != motion_ctrl_config.max_pos_range_limit || in_config.min_pos_range_limit != motion_ctrl_config.min_pos_range_limit || in_config.polarity != motion_ctrl_config.polarity)
                {
                    //reverse position limits when polarity is inverted
                    if (in_config.polarity == MOTION_POLARITY_INVERTED)
                    {
                        min_position = -in_config.max_pos_range_limit;
                        max_position = -in_config.min_pos_range_limit;
                    }
                    else
                    {
                        min_position = in_config.min_pos_range_limit;
                        max_position = in_config.max_pos_range_limit;
                    }
                }

                //check if we need to update the brake config
                if (in_config.dc_bus_voltage         != motion_ctrl_config.dc_bus_voltage     ||
                        in_config.pull_brake_voltage != motion_ctrl_config.pull_brake_voltage ||
                        in_config.pull_brake_time    != motion_ctrl_config.pull_brake_time    ||
                        in_config.hold_brake_voltage != motion_ctrl_config.hold_brake_voltage)
                {
                    torque_enable_flag   =0;
                    velocity_enable_flag =0;
                    position_enable_flag =0;
                    torque_ref_k = 0;

                    i_torque_control.set_safe_torque_off_enabled();
                    i_torque_control.set_brake_status(DISABLE_BRAKE);

                    update_brake_configuration(in_config, i_torque_control);
                }

                motion_ctrl_config = in_config;

                if(motion_ctrl_config.velocity_kp<0)            motion_ctrl_config.velocity_kp=0;
                if(motion_ctrl_config.velocity_kp>100000000)    motion_ctrl_config.velocity_kp=100000000;
                if(motion_ctrl_config.velocity_ki<0)            motion_ctrl_config.velocity_ki=0;
                if(motion_ctrl_config.velocity_ki>100000000)    motion_ctrl_config.velocity_ki=100000000;
                if(motion_ctrl_config.velocity_kd<0)            motion_ctrl_config.velocity_kd=0;
                if(motion_ctrl_config.velocity_kd>100000000)    motion_ctrl_config.velocity_kd=100000000;
                pid_set_parameters(
                        (double)motion_ctrl_config.velocity_kp, (double)motion_ctrl_config.velocity_ki,
                        (double)motion_ctrl_config.velocity_kd, (double)motion_ctrl_config.velocity_integral_limit,
                        POSITION_CONTROL_LOOP_PERIOD, velocity_control_pid_param);


                if(motion_ctrl_config.position_kp<0)            motion_ctrl_config.position_kp=0;
                if(motion_ctrl_config.position_kp>100000000)    motion_ctrl_config.position_kp=100000000;
                if(motion_ctrl_config.position_ki<0)            motion_ctrl_config.position_ki=0;
                if(motion_ctrl_config.position_ki>100000000)    motion_ctrl_config.position_ki=100000000;
                if(motion_ctrl_config.position_kd<0)            motion_ctrl_config.position_kd=0;
                if(motion_ctrl_config.position_kd>100000000)    motion_ctrl_config.position_kd=100000000;
                pid_set_parameters((double)motion_ctrl_config.position_kp, (double)motion_ctrl_config.position_ki,
                        (double)motion_ctrl_config.position_kd, (double)motion_ctrl_config.position_integral_limit,
                        POSITION_CONTROL_LOOP_PERIOD, position_control_pid_param);

                // set GS controller params
                if(motion_ctrl_config.position_kp_l<0)            motion_ctrl_config.position_kp_l=0;
                if(motion_ctrl_config.position_kp_l>100000000)    motion_ctrl_config.position_kp_l=100000000;
                if(motion_ctrl_config.position_ki_l<0)            motion_ctrl_config.position_ki_l=0;
                if(motion_ctrl_config.position_ki_l>100000000)    motion_ctrl_config.position_ki_l=100000000;
                if(motion_ctrl_config.position_kd_l<0)            motion_ctrl_config.position_kd_l=0;
                if(motion_ctrl_config.position_kd_l>100000000)    motion_ctrl_config.position_kd_l=100000000;
                if(motion_ctrl_config.position_kp_h<0)            motion_ctrl_config.position_kp_h=0;
                if(motion_ctrl_config.position_kp_h>100000000)    motion_ctrl_config.position_kp_h=100000000;
                if(motion_ctrl_config.position_ki_h<0)            motion_ctrl_config.position_ki_h=0;
                if(motion_ctrl_config.position_ki_h>100000000)    motion_ctrl_config.position_ki_h=100000000;
                if(motion_ctrl_config.position_kd_h<0)            motion_ctrl_config.position_kd_h=0;
                if(motion_ctrl_config.position_kd_h>100000000)    motion_ctrl_config.position_kd_h=100000000;
                if(motion_ctrl_config.velocity_kp_l<0)            motion_ctrl_config.velocity_kp_l=0;
                if(motion_ctrl_config.velocity_kp_l>100000000)    motion_ctrl_config.velocity_kp_l=100000000;
                if(motion_ctrl_config.velocity_ki_l<0)            motion_ctrl_config.velocity_ki_l=0;
                if(motion_ctrl_config.velocity_ki_l>100000000)    motion_ctrl_config.velocity_ki_l=100000000;
                if(motion_ctrl_config.velocity_kd_l<0)            motion_ctrl_config.velocity_kd_l=0;
                if(motion_ctrl_config.velocity_kd_l>100000000)    motion_ctrl_config.velocity_kd_l=100000000;
                if(motion_ctrl_config.velocity_kp_h<0)            motion_ctrl_config.velocity_kp_h=0;
                if(motion_ctrl_config.velocity_kp_h>100000000)    motion_ctrl_config.velocity_kp_h=100000000;
                if(motion_ctrl_config.velocity_ki_h<0)            motion_ctrl_config.velocity_ki_h=0;
                if(motion_ctrl_config.velocity_ki_h>100000000)    motion_ctrl_config.velocity_ki_h=100000000;
                if(motion_ctrl_config.velocity_kd_h<0)            motion_ctrl_config.velocity_kd_h=0;
                if(motion_ctrl_config.velocity_kd_h>100000000)    motion_ctrl_config.velocity_kd_h=100000000;
                if(motion_ctrl_config.velocity_lo_l<0)            motion_ctrl_config.velocity_lo_l=0;
                if(motion_ctrl_config.velocity_lo_l>motion_ctrl_config.max_motor_speed)
                    motion_ctrl_config.velocity_lo_l=motion_ctrl_config.max_motor_speed;
                if(motion_ctrl_config.velocity_hi_l<0)            motion_ctrl_config.velocity_hi_l=0;
                if(motion_ctrl_config.velocity_hi_l>motion_ctrl_config.max_motor_speed)
                    motion_ctrl_config.velocity_hi_l=motion_ctrl_config.max_motor_speed;
                gain_scheduling_set_param((double)motion_ctrl_config.position_kp_l, (double)motion_ctrl_config.position_ki_l, (double)motion_ctrl_config.position_kd_l,
                        (double)motion_ctrl_config.position_kp_h, (double)motion_ctrl_config.position_ki_h, (double)motion_ctrl_config.position_kd_h,
                        (double)motion_ctrl_config.velocity_kp_l, (double)motion_ctrl_config.velocity_ki_l, (double)motion_ctrl_config.velocity_kd_l,
                        (double)motion_ctrl_config.velocity_kp_h, (double)motion_ctrl_config.velocity_ki_h, (double)motion_ctrl_config.velocity_kd_h,
                        motion_ctrl_config.velocity_lo_l, motion_ctrl_config.velocity_hi_l, gain_sch_param);


                profiler_param.acceleration_max = (double)(motion_ctrl_config.max_acceleration_profiler);
                profiler_param.deceleration_max = (double)(motion_ctrl_config.max_deceleration_profiler);
                profiler_param.v_max = (double)(motion_ctrl_config.max_speed_profiler);
                profiler_param.torque_rate_max = (double)(motion_ctrl_config.max_torque_rate_profiler);

                lt_position_control_reset(lt_pos_ctrl);
                lt_position_control_set_parameters(lt_pos_ctrl, motion_ctrl_config.max_motor_speed, motion_ctrl_config.resolution, motion_ctrl_config.moment_of_inertia,
                        motion_ctrl_config.position_kp, motion_ctrl_config.position_ki, motion_ctrl_config.position_kd, motion_ctrl_config.position_integral_limit,
                        motion_ctrl_config.max_torque, POSITION_CONTROL_LOOP_PERIOD);
                //reset error
                motion_control_error = MOTION_CONTROL_NO_ERROR;

                second_order_LP_filter_init(motion_ctrl_config.filter, POSITION_CONTROL_LOOP_PERIOD, torque_filter_param);

                break;

        case i_motion_control[int i].get_motion_control_config() ->  MotionControlConfig out_config:
                out_config = motion_ctrl_config;
                break;

        case i_motion_control[int i].update_control_data(DownstreamControlData downstream_control_data_in) -> UpstreamControlData upstream_control_data_out:
                downstream_control_data = downstream_control_data_in;
                upstream_control_data_out = i_torque_control.update_upstream_control_data(downstream_control_data.gpio_output);

                switch (error_phase)
                {
                case A:
                    upstream_control_data_out.error_status = PHASE_FAILURE_L1;
                    break;
                case B:
                    upstream_control_data_out.error_status = PHASE_FAILURE_L2;
                    break;
                case C:
                    upstream_control_data_out.error_status = PHASE_FAILURE_L3;
                    break;
                }

                switch (error_sens)
                {
                case POS_ERR:
                    upstream_control_data_out.sensor_error = SENSOR_POSITION_FAULT;
                    break;
                case SPEED_ERR:
                    upstream_control_data.sensor_error = SENSOR_SPEED_FAULT;
                    break;
                case ANGLE_ERR:
                    if (motorcontrol_config.commutation_sensor == HALL_SENSOR)
                        upstream_control_data_out.sensor_error = SENSOR_HALL_FAULT;
                    else
                        upstream_control_data_out.sensor_error = SENSOR_INCREMENTAL_FAULT;
                    break;
                case PORTS_ERR:
                    upstream_control_data_out.sensor_error = SENSOR_HALL_FAULT;
                    break;
                }

                //reverse position/velocity feedback/commands when polarity is inverted
                if (motion_ctrl_config.polarity == MOTION_POLARITY_INVERTED)
                {
                    //feeedback
                    upstream_control_data_out.position          = -upstream_control_data_out.position;
                    upstream_control_data_out.velocity          = -upstream_control_data_out.velocity;
                    upstream_control_data_out.computed_torque   = -upstream_control_data_out.computed_torque;
                    //commands
                    downstream_control_data.position_cmd = -downstream_control_data.position_cmd;
                    downstream_control_data.velocity_cmd = -downstream_control_data.velocity_cmd;
                    downstream_control_data.torque_cmd   = -downstream_control_data.torque_cmd;
                }

                //apply limits on commands
                //position limits
                if (downstream_control_data.position_cmd > max_position)
                    downstream_control_data.position_cmd = max_position;
                else if (downstream_control_data.position_cmd < min_position)
                    downstream_control_data.position_cmd = min_position;
                //velocity limit
                if (downstream_control_data.velocity_cmd > motion_ctrl_config.max_motor_speed)
                    downstream_control_data.velocity_cmd = motion_ctrl_config.max_motor_speed;
                else if (downstream_control_data.velocity_cmd < -motion_ctrl_config.max_motor_speed)
                    downstream_control_data.velocity_cmd = -motion_ctrl_config.max_motor_speed;

                //error
                upstream_control_data_out.motion_control_error = motion_control_error;

                break;

                case i_motion_control[int i].set_j(int j):
                        motion_ctrl_config.moment_of_inertia = j;
                        lt_position_control_set_parameters(lt_pos_ctrl, motion_ctrl_config.max_motor_speed, motion_ctrl_config.resolution, motion_ctrl_config.moment_of_inertia,
                                motion_ctrl_config.position_kp, motion_ctrl_config.position_ki, motion_ctrl_config.position_kd, motion_ctrl_config.position_integral_limit,
                                motion_ctrl_config.max_torque, POSITION_CONTROL_LOOP_PERIOD);
                        break;

                case i_motion_control[int i].set_torque(int in_target_torque):
                        if (motion_ctrl_config.polarity == MOTION_POLARITY_INVERTED)
                            downstream_control_data.torque_cmd = -in_target_torque;
                        else
                            downstream_control_data.torque_cmd = in_target_torque;
                        break;

                case i_motion_control[int i].get_position() -> int out_position:
                        if (motion_ctrl_config.polarity == MOTION_POLARITY_INVERTED)
                            out_position = -upstream_control_data.position;
                        else
                            out_position = upstream_control_data.position;
                        break;

                case i_motion_control[int i].get_velocity() -> int out_velocity:
                        if (motion_ctrl_config.polarity == MOTION_POLARITY_INVERTED)
                            out_velocity = -upstream_control_data.velocity;
                        else
                            out_velocity = upstream_control_data.velocity;
                        break;

                case i_motion_control[int i].get_motorcontrol_config() -> MotorcontrolConfig out_motorcontrol_config:
                        out_motorcontrol_config = i_torque_control.get_config();
                        break;

                case i_motion_control[int i].set_motorcontrol_config(MotorcontrolConfig in_motorcontrol_config):
                        torque_enable_flag = 0;
                        position_enable_flag = 0;
                        velocity_enable_flag = 0;
                        i_torque_control.set_config(in_motorcontrol_config);
                        break;

                case i_motion_control[int i].set_brake_status(int in_brake_status):
                        i_torque_control.set_brake_status(in_brake_status);
                        break;

                case i_motion_control[int i].update_brake_configuration():
                        torque_enable_flag   =0;
                        velocity_enable_flag =0;
                        position_enable_flag =0;
                        torque_ref_k = 0;

                        i_torque_control.set_safe_torque_off_enabled();
                        i_torque_control.set_brake_status(DISABLE_BRAKE);

                        update_brake_configuration(motion_ctrl_config, i_torque_control);

                        break;

                case i_motion_control[int i].set_offset_detection_enabled() -> MotorcontrolConfig out_motorcontrol_config:
                        //offset detection
                        out_motorcontrol_config = i_torque_control.get_config();
                        out_motorcontrol_config.commutation_angle_offset = -1;
                        i_torque_control.set_offset_detection_enabled();
                        while(out_motorcontrol_config.commutation_angle_offset == -1)
                        {
                            out_motorcontrol_config = i_torque_control.get_config();
                            out_motorcontrol_config.commutation_angle_offset = i_torque_control.get_offset();
                            delay_milliseconds(50);//wait until offset is detected
                        }

                        //check polarity state
                        if(i_torque_control.get_sensor_polarity_state() != 1)
                        {
                            out_motorcontrol_config.commutation_angle_offset = -1;
                        }
                        //write offset in config
                        i_torque_control.set_config(out_motorcontrol_config);

                        torque_enable_flag   = 0;
                        position_enable_flag = 0;
                        velocity_enable_flag = 0;
                        break;

                case i_motion_control[int i].reset_motorcontrol_faults():
                        i_torque_control.reset_faults();
                        error_sens = NO_ERROR;
                        i_torque_control.set_sensor_status(error_sens);
                        error_phase = NO_ERROR;
                        break;

                case i_motion_control[int i].set_safe_torque_off_enabled():
                        i_torque_control.set_brake_status(DISABLE_BRAKE);
                        torque_enable_flag   = 0;
                        velocity_enable_flag = 0;
                        position_enable_flag = 0;
                        i_torque_control.set_safe_torque_off_enabled();
                        break;

                case i_motion_control[int i].enable_cogging_compensation(int flag):
                        if (flag)
                        {
                            i_torque_control.enable_cogging_compensation();
                        }
                        else
                        {
                            i_torque_control.disable_cogging_compensation();
                        }
                        break;


                case i_motion_control[int i].open_phase_detection() -> {int error_phase_out, float resistance_out}:
                        error_phase = open_phase_detection_offline(i_torque_control, motorcontrol_config, downstream_control_data, app_tile_usec, current_ratio, &resist);
                        error_phase_out = error_phase;
                        resistance_out = resist;
                        break;

                case i_motion_control[int i].sensors_evaluation() -> int sensor_status_out :
                        error_sens = sensor_functionality_evaluation(i_torque_control, motorcontrol_config, downstream_control_data, app_tile_usec);
                        sensor_status_out = error_sens;
                        break;
        }

        }
    }


void update_brake_configuration(MotionControlConfig &motion_ctrl_config, client interface TorqueControlInterface i_torque_control)
{

    int pull_brake_voltage=0x00000000; //pull brake voltage in per thousand of nominal V_dc
    int hold_brake_voltage=0x00000000; //hold brake voltage in per thousand of nominal V_dc

    pull_brake_voltage = motion_ctrl_config.pull_brake_voltage/motion_ctrl_config.dc_bus_voltage ;
    hold_brake_voltage = motion_ctrl_config.hold_brake_voltage/motion_ctrl_config.dc_bus_voltage ;

    if(motion_ctrl_config.dc_bus_voltage <= 0)
    {
        printstr("ERROR: NEGATIVE VDC VALUE DEFINED IN SETTINGS");
    }
    else if(motion_ctrl_config.pull_brake_voltage > (motion_ctrl_config.dc_bus_voltage*1000))
    {
        printstr("ERROR: PULL BRAKE VOLTAGE HIGHER THAN VDC");
    }
    else if(motion_ctrl_config.pull_brake_voltage < 0)
    {
        printstr("ERROR: NEGATIVE PULL BRAKE VOLTAGE");
    }
    else if(motion_ctrl_config.hold_brake_voltage > (motion_ctrl_config.dc_bus_voltage*1000))
    {
        printstr("ERROR: HOLD BRAKE VOLTAGE HIGHER THAN VDC");
    }
    else if(motion_ctrl_config.hold_brake_voltage < 0)
    {
        printstr("ERROR: NEGATIVE HOLD BRAKE VOLTAGE");
    }
    else if(motion_ctrl_config.pull_brake_time < 0)
    {
        printstr("ERROR: NEGATIVE PERIOD START BRAKE SETTINGS!");
    }
    else
    {
        i_torque_control.configure_brake(pull_brake_voltage, motion_ctrl_config.pull_brake_time, hold_brake_voltage);
    }
}


void enable_motorcontrol(MotionControlConfig &motion_ctrl_config, client interface TorqueControlInterface i_torque_control, int position,
        int &special_brake_release_counter, int &special_brake_release_initial_position, int &special_brake_release_torque, MotionControlError &motion_control_error)
{

    motion_control_error = MOTION_CONTROL_NO_ERROR;
    //special brake release
    if (motion_ctrl_config.brake_release_strategy > 1)
    {
        special_brake_release_counter = 0;
        special_brake_release_initial_position = position;
        special_brake_release_torque = (motion_ctrl_config.brake_release_strategy*motion_ctrl_config.max_torque)/100;
    }

    if (motion_ctrl_config.brake_release_strategy > 0) {
        i_torque_control.set_brake_status(ENABLE_BRAKE);
    }

    //enable motorcontrol and release brake
    i_torque_control.set_torque_control_enabled();
}
