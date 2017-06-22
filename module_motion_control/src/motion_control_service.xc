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
    POS_ERR = 1,
    SPEED_ERR  = 2,
    ANGLE_ERR = 3,
    PORTS_ERR = 4

} sensor_fault;

int open_phase_detection_function(client interface TorqueControlInterface client i_torque_control,
        MotorcontrolConfig motorcontrol_config, int app_tile_usec, int current_ratio, float * ret)
{
    UpstreamControlData upstream_control_data;
    float I[NR_PHASES] = { 0 };
    int refer[NR_PHASES] = {0, 20, 30, 30};
    unsigned counter = 1;
    float voltage = 0;
    int error_phase = NO_ERROR;
    float rated_curr = (float)motorcontrol_config.rated_current/1000;

    timer tmr;
    unsigned ts;

    i_torque_control.set_torque_control_disabled();
    i_torque_control.start_system_eval();
    i_torque_control.set_evaluation_references(refer[A], refer[B], refer[C]);

    while ((I[A] < 1|| I[B] < 1 || I[C] < 1) || (I[A] != (2*I[B]) || I[A] != (2*I[C])))
    {
        upstream_control_data = i_torque_control.update_upstream_control_data();

        I[B] = (float)upstream_control_data.I_b/(float)(current_ratio);
        I[C] = (float)upstream_control_data.I_c/(float)(current_ratio);
        I[A] = -(I[B]+I[C]);

        for(int i = 0; i < NR_PHASES; i++)
            if (I[i] < 0)
                I[i] = -I[i];

        if (counter % 100 == 0)
        {
            refer[B] += 1;
            refer[C] += 1;
            i_torque_control.set_evaluation_references(refer[A], refer[B], refer[C]);
        }

        if (refer[B] > 60 || I[A] > 0.8*rated_curr || I[B] > 0.8*rated_curr || I[C] > 0.8*rated_curr)
        {
            if (I[A] < rated_curr/4)
                error_phase = A;
            else if (I[B] < rated_curr/4)
                error_phase = B;
            else if (I[C] < rated_curr/4)
                error_phase = C;
            break;
        }

        tmr :> ts;
        tmr when timerafter (ts + 1000*app_tile_usec) :> ts;

        ++counter;
    }

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
 * @param pos_velocity_control_config   Configuration for ttorque/velocity/position controllers.
 * @param i_torque_control Communication  interface to the Motor Control Service.
 * @param i_motion_control[3]         array of MotionControlInterfaces to communicate with upto 3 clients
 * @param i_update_brake                Interface to update brake configuration in PWM service
 *
 * @return void
 *  */
void motion_control_service(MotionControlConfig &motion_ctrl_config,
        interface TorqueControlInterface client i_torque_control,
        interface MotionControlInterface server i_motion_control[3],client interface UpdateBrake i_update_brake)
{
    timer t;
    unsigned int ts;

    int torque_measurement;
    int torque_buffer [8] = {0};
    int index = 0;

    // structure definition
    UpstreamControlData upstream_control_data;
    DownstreamControlData downstream_control_data;

    PIDparam velocity_control_pid_param;

    PIDparam position_control_pid_param;

    NonlinearPositionControl nl_pos_ctrl;


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

    AutoTuneParam velocity_auto_tune;

    //autotune initialization
    init_velocity_auto_tuner(velocity_auto_tune, TUNING_VELOCITY, SETTLING_TIME);

    downstream_control_data.velocity_cmd = 0;
    motion_ctrl_config.enable_velocity_auto_tuner == 0;

    double position_ref_in_k = 0.00;
    double position_ref_in_k_1n = 0.00;
    double position_ref_in_k_2n = 0.00;
    double position_ref_in_k_3n = 0.00;
    double position_k   = 0.00, position_k_1=0.00;

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
    int temperature_ratio = motorcontrol_config.temperature_ratio;
    int current_ratio = motorcontrol_config.current_ratio;

    nl_position_control_reset(nl_pos_ctrl);
    nl_position_control_set_parameters(nl_pos_ctrl, motion_ctrl_config, POSITION_CONTROL_LOOP_PERIOD);

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


    downstream_control_data.position_cmd = 0;
    downstream_control_data.velocity_cmd = 0;
    downstream_control_data.torque_cmd   = 0;
    downstream_control_data.offset_torque = 0;

    upstream_control_data = i_torque_control.update_upstream_control_data();

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

    // open phase detection
    float resist = 0;
    int error_phase = open_phase_detection_function(i_torque_control, motorcontrol_config, app_tile_usec, current_ratio, &resist);

    int angle = 0;
    int velocity = 0;
    int count = 0;
    int position = 0;
    int hall_state = 0;

    unsigned position_ctr = (1<<16), angle_ctr = (1<<12), counter = 0;
    int max_pos = (1<<16), mean_pos = 0, real_mean_pos = 0, tq_pos = 0, real_tq_pos = 0, fq_pos = 0, real_fq_pos = 0;
    int max_angle = (1<<12), mean_angle = 0, real_mean_angle = 0, tq_angle = 0, real_tq_angle = 0, fq_angle = 0, real_fq_angle = 0;
    int old_position, old_angle, hall_state_old = 0, qei_pos = 0, qei_pos_old = 0;
    int index_v = 0, velocity_arr[500], index_d = 0, difference = 0, difference_arr[500];
    int filter_diff = 0, filter_vel = 0;
    int hall_state_ch[NR_PHASES] = { 0 }, hall_order_err = 0, qei_err_counter = 0, flag_lost_ticks = 0;
    int max_pos_found = 0, max_angle_found = 0;
    sensor_fault error_sens = NO_ERROR;
    timer tm_hall;
    unsigned ts_hall;

    // testing the angle, position, velocity from the sensor
    // testing Hall ports
    if (!error_phase)
    {
        position_ctr = 0;
        angle_ctr = 0;
        max_pos = 0;
        max_angle = 0;
        i_torque_control.enable_index_detection();

        while(counter < 20000)
        {
            upstream_control_data = i_torque_control.update_upstream_control_data();
            count = upstream_control_data.position;
            position = upstream_control_data.singleturn;
            velocity = upstream_control_data.velocity;
            angle = upstream_control_data.angle;
            hall_state = upstream_control_data.hall_state;

            if (counter == 0)
            {
                old_position = position;
                old_angle = angle;
                if (motorcontrol_config.commutation_sensor == HALL_SENSOR)
                    hall_state_old = hall_state;
            }
            else
            {
                if (old_position != position)
                {
                    difference = position - old_position;
                    old_position = position;
                    ++position_ctr;
                }
                if (old_angle != angle)
                {
                    old_angle = angle;
                    ++angle_ctr;
                }

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
            }

            if (angle > 4000 && !max_angle_found)
            {
                max_angle_found = 1;
                max_angle = angle;
                mean_angle = max_angle / 2;
                tq_angle = (max_angle + mean_angle) / 2;
                fq_angle = mean_angle / 2;
            }

            if (max_angle_found)
            {
                if (angle > mean_angle - 50 && angle < mean_angle + 50)
                    real_mean_angle = angle;
                if (angle > tq_angle - 50 && angle < tq_angle + 50)
                    real_tq_angle = angle;
                if (angle > fq_angle - 50 && angle < fq_angle + 50)
                    real_fq_angle = angle;
            }

            if (!max_pos_found && position > 60000)
            {
                max_pos_found = 1;
                max_pos = position;
                mean_pos = max_pos / 2;
                tq_pos = (max_pos + mean_pos) / 2;
                fq_pos = mean_pos / 2;
            }

            if (max_pos_found)
            {
                if (position > mean_pos -100 && position < mean_pos + 100)
                    real_mean_pos = position;
                if (position > tq_pos - 100 && position < tq_pos + 100)
                    real_tq_pos = position;
                if (position > fq_pos - 100 && position < fq_pos + 100)
                    real_fq_pos = position;
            }

            velocity_arr[index_v] = velocity;
            index_v++;
            if (index_v == 500)
            {
                index_v = 0;
                filter_vel = 0;
                for (int i = 0; i < 500; i++)
                    filter_vel += velocity_arr[i];
                filter_vel /= 500;
                if (filter_vel < 0)
                    filter_vel = -filter_vel;
            }

            difference_arr[index_d] = difference;
            index_d++;
            if (index_d == 500)
            {
                index_d = 0;
                filter_diff = 0;
                for (int i = 0; i < 500; i++)
                    filter_diff += difference_arr[i];
                filter_diff /= 500;
                if (filter_diff < 0)
                    filter_diff = -filter_diff;
            }

            ++counter;

            if (counter == 20000)
            {
                printf("%d %d\n", filter_vel, filter_diff);
                if (filter_vel < 50)
                {
                    if (filter_vel - filter_diff < -30 || filter_vel - filter_diff > 30)
                        error_sens = SPEED_ERR;
                }
                else
                {
                    if (filter_vel - filter_diff < -60 || filter_vel - filter_diff > 60)
                        error_sens = SPEED_ERR;
                }

                if ((mean_pos-real_mean_pos < -100 || mean_pos-real_mean_pos > 100)
                        || (tq_pos - real_tq_pos < -100 || tq_pos - real_tq_pos > 100)
                        || (fq_pos - real_fq_pos < -100 || fq_pos - real_fq_pos > 100)
                        || position_ctr < 2000  || max_pos < 60000)
                    error_sens = POS_ERR;

                if ((mean_angle-real_mean_angle < -50 || mean_angle-real_mean_angle > 50)
                        || (tq_angle - real_tq_angle < -50 || tq_angle - real_tq_angle > 50)
                        || (fq_angle - real_fq_angle < -50 || fq_angle - real_fq_angle > 50)
                        || max_angle < 4000 || angle_ctr < 2000)
                    error_sens = ANGLE_ERR;

                if (motorcontrol_config.commutation_sensor == HALL_SENSOR)
                {
                    if (!hall_state_ch[A] || !hall_state_ch[B] || !hall_state_ch[C])
                        error_sens = PORTS_ERR;
                }
            }

            t when timerafter (ts + 500*app_tile_usec) :> ts;
        }

        i_torque_control.disable_index_detection();
    }

    //QEI index calibration
    if (motorcontrol_config.commutation_sensor == QEI_SENSOR && !error_phase && !error_sens)
    {
        printf("Find encoder index \n");
        int index_found = 0;
        i_torque_control.enable_index_detection();
        while (!index_found)
        {
            upstream_control_data = i_torque_control.update_upstream_control_data();
            index_found = upstream_control_data.qei_index_found;
        }
        printf("Incremental encoder index found\n");
        i_torque_control.disable_index_detection();
    }

    //brake
    int special_brake_release_counter = BRAKE_RELEASE_DURATION+1;
    int special_brake_release_initial_position = 0;
    int special_brake_release_torque = 0;
    int brake_shutdown_counter = 0;
    unsigned int update_brake_configuration_time;
    i_torque_control.set_safe_torque_off_enabled();
    i_torque_control.set_brake_status(0);
    t :> update_brake_configuration_time;
    update_brake_configuration_time += BRAKE_UPDATE_CONFIG_WAIT*1000*app_tile_usec;
    int update_brake_configuration_flag = 1;

    printstr(">>   SOMANET POSITION CONTROL SERVICE STARTING...\n");

    t :> ts;
    tm_hall :> ts_hall;

    while(1)
    {
#pragma ordered
        select
        {
        case t when timerafter(ts + app_tile_usec * POSITION_CONTROL_LOOP_PERIOD) :> ts:

                upstream_control_data = i_torque_control.update_upstream_control_data();

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

                        velocity_controller_auto_tune(velocity_auto_tune, velocity_ref_in_k, velocity_k, POSITION_CONTROL_LOOP_PERIOD);

                        torque_ref_k = pid_update(velocity_ref_in_k, velocity_k, POSITION_CONTROL_LOOP_PERIOD, velocity_control_pid_param);

                        if(velocity_auto_tune.enable == 0)
                        {
                            torque_ref_k=0;
                            torque_enable_flag   =0;
                            velocity_enable_flag =0;
                            position_enable_flag =0;
                            i_torque_control.set_torque_control_disabled();

                            motion_ctrl_config.enable_velocity_auto_tuner = 0;

                            printf("kp:%i ki:%i kd:%i \n",  ((int)(velocity_auto_tune.kp)), ((int)(velocity_auto_tune.ki)), ((int)(velocity_auto_tune.kd)));

                            motion_ctrl_config.velocity_kp = ((int)(velocity_auto_tune.kp));
                            motion_ctrl_config.velocity_ki = ((int)(velocity_auto_tune.ki));
                            motion_ctrl_config.velocity_kd = ((int)(velocity_auto_tune.kd));
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
                                    ct_parameters.back_and_forth = 0;
                                    motion_ctrl_config.enable_compensation_recording = 0;
                                    velocity_ref_k = 0;
                                }
                                ct_parameters.remaining_cells = COGGING_TORQUE_ARRAY_SIZE;
                                i_torque_control.set_config(motorcontrol_config);
                                ct_parameters.torque_recording_started = 0;
                                ct_parameters.delay_counter = 0;
                                i_torque_control.set_torque_control_enabled();
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
                        velocity_ref_k =pid_update(position_ref_in_k, position_k, POSITION_CONTROL_LOOP_PERIOD, position_control_pid_param);
                        if(velocity_ref_k> motion_ctrl_config.max_motor_speed) velocity_ref_k = motion_ctrl_config.max_motor_speed;
                        if(velocity_ref_k<-motion_ctrl_config.max_motor_speed) velocity_ref_k =-motion_ctrl_config.max_motor_speed;
                        torque_ref_k   =pid_update(velocity_ref_k   , velocity_k, POSITION_CONTROL_LOOP_PERIOD, velocity_control_pid_param);
                    }
                    else if (pos_control_mode == NL_POSITION_CONTROLLER)
                    {
                        torque_ref_k = update_nl_position_control(nl_pos_ctrl, position_ref_in_k, position_k_1, position_k);
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
                    i_torque_control.set_brake_status(0);
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
                    i_torque_control.set_brake_status(0);
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

                i_torque_control.set_torque(((int)(torque_ref_k)));

                //update brake config when ready
                if (update_brake_configuration_flag && timeafter(ts, update_brake_configuration_time)) {
                    update_brake_configuration(motion_ctrl_config, i_torque_control, i_update_brake);
                    update_brake_configuration_flag = 0;
                    if (torque_enable_flag+velocity_enable_flag+position_enable_flag) { //one of the control is enabled, start motorcontrol and brake
                        enable_motorcontrol(motion_ctrl_config, i_torque_control, upstream_control_data.position, special_brake_release_counter, special_brake_release_initial_position, special_brake_release_torque, motion_control_error);
                    }
                }

                torque_measurement = filter(torque_buffer, index, 8, torque_ref_k);

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

                switch (error_sens)
                {
                    case SPEED_ERR:
                        upstream_control_data.error_status = SPEED_FAULT;
                        break;
                    case POS_ERR:
                        upstream_control_data.error_status = POSITION_FAULT;
                        break;
                    case ANGLE_ERR:
                        if (motorcontrol_config.commutation_sensor == HALL_SENSOR)
                            upstream_control_data.error_status = HALL_SENSOR_FAULT;
                        else
                            upstream_control_data.error_status = INCREMENTAL_SENSOR_1_FAULT;
                        break;
                    case PORTS_ERR:
                        upstream_control_data.error_status = HALL_SENSOR_FAULT;
                        break;
                }

                if (motorcontrol_config.commutation_sensor == QEI_SENSOR)
                    if (upstream_control_data.ticks_notification)
                        flag_lost_ticks = 1;

                if (flag_lost_ticks)
                    upstream_control_data.error_status = QEI_INDEX_LOSING_TICKS;


#ifdef XSCOPE_POSITION_CTRL
                xscope_int(VELOCITY, upstream_control_data.velocity);
                xscope_int(POSITION, upstream_control_data.position);
                xscope_int(VELOCITY_SECONDARY, upstream_control_data.secondary_velocity);
                xscope_int(POSITION_SECONDARY, upstream_control_data.secondary_position);
                xscope_int(TORQUE,   upstream_control_data.computed_torque);
                xscope_int(POSITION_CMD, (int)position_ref_in_k);
                xscope_int(VELOCITY_CMD, (int)velocity_ref_in_k);
                xscope_int(TORQUE_CMD, torque_ref_k);
                xscope_int(FAULT_CODE, upstream_control_data.error_status*1000);
                xscope_int(SENSOR_ERROR_X100, upstream_control_data.sensor_error*100);
#endif

#ifdef XSCOPE_ANALOGUE_MEASUREMENT
                xscope_int(V_DC, upstream_control_data.V_dc);
                xscope_int(I_DC, upstream_control_data.analogue_input_b_2);
                xscope_int(TEMPERATURE, (upstream_control_data.temperature/temperature_ratio));
                xscope_int(I_B, upstream_control_data.I_b);
                xscope_int(I_C, upstream_control_data.I_c);
                xscope_int(AI_A1, upstream_control_data.analogue_input_a_1);
                xscope_int(AI_A2, upstream_control_data.analogue_input_a_2);
                xscope_int(AI_B1, upstream_control_data.analogue_input_b_1);
                xscope_int(AI_B2, upstream_control_data.analogue_input_b_2);
#endif


                break;

        case tm_hall when timerafter (ts_hall + 100*app_tile_usec) :> ts_hall :

                /*
                 * check hall states if the Hall sensor is used
                 */
                if (motorcontrol_config.commutation_sensor == HALL_SENSOR)
                {
                    hall_state = upstream_control_data.hall_state;

                    if (hall_state_old == 0)
                    {
                        hall_state_old = hall_state;
                    }

                    if (hall_state != hall_state_old)
                    {
                        switch (hall_state_old)
                        {
                        case 1:
                            if (hall_state != 5 && hall_state != 3)
                                ++hall_order_err;
                            break;
                        case 2:
                            if (hall_state != 3 && hall_state != 6)
                                ++hall_order_err;
                            break;
                        case 3:
                            if (hall_state != 1 && hall_state != 2)
                                ++hall_order_err;
                            break;
                        case 4:
                            if (hall_state != 6 && hall_state != 5)
                                ++hall_order_err;
                            break;
                        case 5:
                            if (hall_state != 4 && hall_state != 1)
                                ++hall_order_err;
                            break;
                        case 6:
                            if (hall_state != 2 && hall_state != 4)
                                ++hall_order_err;
                            break;
                        }
                    }

                    hall_state_old = hall_state;

                    if (hall_order_err == 1)
                    {
                        error_sens = PORTS_ERR;
                    }
                }

                if (motorcontrol_config.commutation_sensor == QEI_SENSOR)
                {
                    qei_pos = upstream_control_data.singleturn;

                    if(qei_pos != qei_pos_old)
                    {
                       if (qei_pos == 0)
                           qei_err_counter = 1;
                       else
                           qei_err_counter = 0;
                    }

                    if (qei_err_counter && qei_pos == 0)
                        ++qei_err_counter;

                    if (qei_err_counter == 1000)
                        error_sens = POS_ERR;

                    qei_pos_old = qei_pos;
                }

                break;

        case i_motion_control[int i].disable():

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
                nl_position_control_reset(nl_pos_ctrl);
                nl_position_control_set_parameters(nl_pos_ctrl, motion_ctrl_config, POSITION_CONTROL_LOOP_PERIOD);
                pid_reset(position_control_pid_param);


                //start motorcontrol and release brake if update_brake_configuration is not ongoing
                if (update_brake_configuration_flag == 0) {
                    enable_motorcontrol(motion_ctrl_config, i_torque_control, upstream_control_data.position, special_brake_release_counter, special_brake_release_initial_position, special_brake_release_torque, motion_control_error);
                }

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


                //start motorcontrol and release brake if update_brake_configuration is not ongoing
                if (update_brake_configuration_flag == 0) {
                    enable_motorcontrol(motion_ctrl_config, i_torque_control, upstream_control_data.position, special_brake_release_counter, special_brake_release_initial_position, special_brake_release_torque, motion_control_error);
                }

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

                //start motorcontrol and release brake if update_brake_configuration is not ongoing
                if (update_brake_configuration_flag == 0) {
                    enable_motorcontrol(motion_ctrl_config, i_torque_control, upstream_control_data.position, special_brake_release_counter, special_brake_release_initial_position, special_brake_release_torque, motion_control_error);
                }

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
                if (in_config.dc_bus_voltage != motion_ctrl_config.dc_bus_voltage ||
                        in_config.pull_brake_voltage != motion_ctrl_config.pull_brake_voltage ||
                        in_config.pull_brake_time != motion_ctrl_config.pull_brake_time ||
                        in_config.hold_brake_voltage != motion_ctrl_config.hold_brake_voltage)
                {
                    torque_enable_flag   =0;
                    velocity_enable_flag =0;
                    position_enable_flag =0;
                    torque_ref_k = 0;

                    i_torque_control.set_safe_torque_off_enabled();
                    i_torque_control.set_brake_status(0);

                    t :> update_brake_configuration_time;
                    update_brake_configuration_time += BRAKE_UPDATE_CONFIG_WAIT*1000*app_tile_usec;
                    update_brake_configuration_flag = 1;
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

                profiler_param.acceleration_max = (double)(motion_ctrl_config.max_acceleration_profiler);
                profiler_param.deceleration_max = (double)(motion_ctrl_config.max_deceleration_profiler);
                profiler_param.v_max = (double)(motion_ctrl_config.max_speed_profiler);
                profiler_param.torque_rate_max = (double)(motion_ctrl_config.max_torque_rate_profiler);

                nl_position_control_reset(nl_pos_ctrl);
                nl_position_control_set_parameters(nl_pos_ctrl, motion_ctrl_config, POSITION_CONTROL_LOOP_PERIOD);

                //reset error
                motion_control_error = MOTION_CONTROL_NO_ERROR;
                break;

        case i_motion_control[int i].get_motion_control_config() ->  MotionControlConfig out_config:
                out_config = motion_ctrl_config;
                break;

        case i_motion_control[int i].update_control_data(DownstreamControlData downstream_control_data_in) -> UpstreamControlData upstream_control_data_out:
                upstream_control_data_out = i_torque_control.update_upstream_control_data();
                downstream_control_data = downstream_control_data_in;

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
                    case SPEED_ERR:
                        upstream_control_data_out.error_status = SPEED_FAULT;
                        break;
                    case POS_ERR:
                        upstream_control_data_out.error_status = POSITION_FAULT;
                        break;
                    case ANGLE_ERR:
                        if (motorcontrol_config.commutation_sensor == HALL_SENSOR)
                            upstream_control_data_out.error_status = HALL_SENSOR_FAULT;
                        else
                            upstream_control_data_out.error_status = INCREMENTAL_SENSOR_1_FAULT;
                        break;
                    case PORTS_ERR:
                        upstream_control_data_out.error_status = HALL_SENSOR_FAULT;
                        break;
                }

                if (flag_lost_ticks)
                    upstream_control_data_out.error_status = QEI_INDEX_LOSING_TICKS;

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
                nl_position_control_set_parameters(nl_pos_ctrl, motion_ctrl_config, POSITION_CONTROL_LOOP_PERIOD);
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
                if(in_brake_status==1)
                {
                    i_torque_control.set_brake_status(1);
                }
                else if (in_brake_status==0)
                {
                    i_torque_control.set_brake_status(in_brake_status);
                }
                break;


        case i_motion_control[int i].update_brake_configuration():
                torque_enable_flag   =0;
                velocity_enable_flag =0;
                position_enable_flag =0;
                torque_ref_k = 0;

                i_torque_control.set_safe_torque_off_enabled();
                i_torque_control.set_brake_status(0);

                t :> update_brake_configuration_time;
                update_brake_configuration_time += BRAKE_UPDATE_CONFIG_WAIT*1000*app_tile_usec;
                update_brake_configuration_flag = 1;
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
                hall_order_err = 0;
                qei_err_counter = 0;
                flag_lost_ticks = 0;
                error_phase = NO_ERROR;
                break;

        case i_motion_control[int i].set_safe_torque_off_enabled():
                i_torque_control.set_brake_status(0);
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
                error_phase = open_phase_detection_function(i_torque_control, motorcontrol_config, app_tile_usec, current_ratio, &resist);
                error_phase_out = error_phase;
                resistance_out = resist;
                break;
        }
    }
}


void update_brake_configuration(MotionControlConfig &motion_ctrl_config, client interface TorqueControlInterface i_torque_control, client interface UpdateBrake i_update_brake)
{
    int error=0;
    int duty_min=0, duty_max=0, duty_divider=0;
    int duty_start_brake =0, duty_maintain_brake=0, period_start_brake=0;


    if(motion_ctrl_config.dc_bus_voltage <= 0)
    {
        printstr("ERROR: NEGATIVE VDC VALUE DEFINED IN SETTINGS");
        return;
    }

    if(motion_ctrl_config.pull_brake_voltage > (motion_ctrl_config.dc_bus_voltage*1000))
    {
        printstr("ERROR: PULL BRAKE VOLTAGE HIGHER THAN VDC");
        return;
    }

    if(motion_ctrl_config.pull_brake_voltage < 0)
    {
        printstr("ERROR: NEGATIVE PULL BRAKE VOLTAGE");
        return;
    }

    if(motion_ctrl_config.hold_brake_voltage > (motion_ctrl_config.dc_bus_voltage*1000))
    {
        printstr("ERROR: HOLD BRAKE VOLTAGE HIGHER THAN VDC");
        return;
    }

    if(motion_ctrl_config.hold_brake_voltage < 0)
    {
        printstr("ERROR: NEGATIVE HOLD BRAKE VOLTAGE");
        return;
    }

    if(period_start_brake < 0)
    {
        printstr("ERROR: NEGATIVE PERIOD START BRAKE SETTINGS!");
        return;
    }


    MotorcontrolConfig motorcontrol_config = i_torque_control.get_config();

    if(motorcontrol_config.ifm_tile_usec==250)
    {
        duty_min = 1500;
        duty_max = 13000;
        duty_divider = 16384;
        period_start_brake = (motion_ctrl_config.pull_brake_time * 15); //pwm is runnig at 15 kHz
    }
    else if(motorcontrol_config.ifm_tile_usec==100)
    {
        duty_min = 600;
        duty_max = 7000;
        duty_divider = 8192;
        period_start_brake = (motion_ctrl_config.pull_brake_time * 12); //pwm is runnig at 12 kHz
    }
    else if (motorcontrol_config.ifm_tile_usec!=100 && motorcontrol_config.ifm_tile_usec!=250)
    {
        error = 1;
    }

    duty_start_brake    = (duty_divider * motion_ctrl_config.pull_brake_voltage)/(1000*motion_ctrl_config.dc_bus_voltage);
    if(duty_start_brake < duty_min) duty_start_brake = duty_min;
    if(duty_start_brake > duty_max) duty_start_brake = duty_max;

    duty_maintain_brake = (duty_divider * motion_ctrl_config.hold_brake_voltage)/(1000*motion_ctrl_config.dc_bus_voltage);
    if(duty_maintain_brake < duty_min) duty_maintain_brake = duty_min;
    if(duty_maintain_brake > duty_max) duty_maintain_brake = duty_max;

    i_update_brake.update_brake_control_data(duty_start_brake, duty_maintain_brake, period_start_brake);
}


void enable_motorcontrol(MotionControlConfig &motion_ctrl_config, client interface TorqueControlInterface i_torque_control, int position,
        int &special_brake_release_counter, int &special_brake_release_initial_position, int &special_brake_release_torque, MotionControlError &motion_control_error)
{

    motion_control_error = MOTION_CONTROL_NO_ERROR;
    //special brake release
    if (motion_ctrl_config.brake_release_strategy > 1) {
        special_brake_release_counter = 0;
        special_brake_release_initial_position = position;
        special_brake_release_torque = (motion_ctrl_config.brake_release_strategy*motion_ctrl_config.max_torque)/100;
    }
    if (motion_ctrl_config.brake_release_strategy > 0) {
        i_torque_control.set_brake_status(1);
    }

    //enable motorcontrol and release brake
    i_torque_control.set_torque_control_enabled();
}
