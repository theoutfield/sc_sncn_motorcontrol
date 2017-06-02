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


#define AUTO_TUNE_STEP_AMPLITUDE    20000
#define AUTO_TUNE_COUNTER_MAX       3000
#define PER_THOUSAND_OVERSHOOT      10

/**
 * @brief Structure type containing auto_tuning parameters of velocity/position controllers
 */
typedef struct {

    double position_init;
    double position_ref;
    int rising_edge;

    int activate;
    int counter;
    int counter_max;

    int step1_counter;
    int step1_completed;

    double step_amplitude;

    double err;
    double err_energy;
    double err_energy_int;
    double err_energy_int_max;
    double dynamic_error_energy_int_max;

    double err_ss;
    double err_energy_ss;
    double err_energy_ss_int;
    double err_energy_ss_int_min;

    double overshoot;
    double overshoot_max;
    double overshoot_min;
    double overshot_per_thousand;
    int overshoot_counter;
    int overshoot_1st_round_damped;

    int rise_time;
    int rise_time_opt;

    int tuning_process_ended;

} NLPosCtrlAutoTuneParam;

/**
 *
 **/
int init_nl_pos_ctrl_autotune(NLPosCtrlAutoTuneParam &nl_pos_ctrl_auto_tune, int step_amplitude, int counter_max, int overshot_per_thousand)
{
    nl_pos_ctrl_auto_tune.position_init = 0.00;
    nl_pos_ctrl_auto_tune.position_ref  = 0.00;

    nl_pos_ctrl_auto_tune.activate=0;
    nl_pos_ctrl_auto_tune.counter=0;
    nl_pos_ctrl_auto_tune.counter_max=counter_max;

    nl_pos_ctrl_auto_tune.step_amplitude = step_amplitude;

    nl_pos_ctrl_auto_tune.err=0.00;
    nl_pos_ctrl_auto_tune.err_energy=0.00;
    nl_pos_ctrl_auto_tune.err_energy_int    =0.00;
    nl_pos_ctrl_auto_tune.err_energy_int_max    = ((2*nl_pos_ctrl_auto_tune.step_amplitude)/1000) * ((2*nl_pos_ctrl_auto_tune.step_amplitude)/1000) * nl_pos_ctrl_auto_tune.counter_max;
    nl_pos_ctrl_auto_tune.dynamic_error_energy_int_max=0.00;

    nl_pos_ctrl_auto_tune.err_ss=0.00;
    nl_pos_ctrl_auto_tune.err_energy_ss=0.00;
    nl_pos_ctrl_auto_tune.err_energy_ss_int=0.00;
    nl_pos_ctrl_auto_tune.err_energy_ss_int_min = nl_pos_ctrl_auto_tune.err_energy_int_max /10;// we are considering the last 10% of the error!

    nl_pos_ctrl_auto_tune.step1_counter=0;
    nl_pos_ctrl_auto_tune.step1_completed=0;

    nl_pos_ctrl_auto_tune.rising_edge=0;

    nl_pos_ctrl_auto_tune.overshoot=0.00;
    nl_pos_ctrl_auto_tune.overshoot_max=0.00;
    nl_pos_ctrl_auto_tune.overshoot_min=nl_pos_ctrl_auto_tune.step_amplitude;
    nl_pos_ctrl_auto_tune.overshot_per_thousand = ((double)(overshot_per_thousand));
    nl_pos_ctrl_auto_tune.overshoot_counter=0;
    nl_pos_ctrl_auto_tune.overshoot_1st_round_damped=0;

    nl_pos_ctrl_auto_tune.rise_time=0;
    nl_pos_ctrl_auto_tune.rise_time_opt=0;

    nl_pos_ctrl_auto_tune.tuning_process_ended=0;

    return 0;
}


/**
 *
 **/
int nl_pos_ctrl_autotune(NLPosCtrlAutoTuneParam &nl_pos_ctrl_auto_tune, MotionControlConfig &motion_ctrl_config, double position_k)
{

    nl_pos_ctrl_auto_tune.counter++;

    if(nl_pos_ctrl_auto_tune.activate==0)
    {
        nl_pos_ctrl_auto_tune.position_init = position_k;

        motion_ctrl_config.position_kp = 10000 ;
        motion_ctrl_config.position_ki = 1000  ;
        motion_ctrl_config.position_kd = 40000 ;
        motion_ctrl_config.position_integral_limit = 1;
        motion_ctrl_config.moment_of_inertia       = 0;

        nl_pos_ctrl_auto_tune.activate = 1;
        nl_pos_ctrl_auto_tune.counter=0;
    }

    if(nl_pos_ctrl_auto_tune.counter==nl_pos_ctrl_auto_tune.counter_max)
    {

        if(nl_pos_ctrl_auto_tune.position_ref == (nl_pos_ctrl_auto_tune.position_init + nl_pos_ctrl_auto_tune.step_amplitude))
        {
            nl_pos_ctrl_auto_tune.position_ref = nl_pos_ctrl_auto_tune.position_init - nl_pos_ctrl_auto_tune.step_amplitude;
            nl_pos_ctrl_auto_tune.rising_edge=0;
        }
        else
        {
            nl_pos_ctrl_auto_tune.position_ref = nl_pos_ctrl_auto_tune.position_init + nl_pos_ctrl_auto_tune.step_amplitude;
            nl_pos_ctrl_auto_tune.rising_edge=1;
        }

        if(nl_pos_ctrl_auto_tune.step1_completed==0) // force the load to follow reference value
        {
            if(nl_pos_ctrl_auto_tune.err_energy_int < (nl_pos_ctrl_auto_tune.err_energy_int_max/10))
            {
                nl_pos_ctrl_auto_tune.step1_counter++;
            }
            else
            {
                motion_ctrl_config.position_integral_limit += 50;
                nl_pos_ctrl_auto_tune.step1_counter=0;
            }

            if(nl_pos_ctrl_auto_tune.step1_counter==10)
            {
                nl_pos_ctrl_auto_tune.step1_completed=1;
                nl_pos_ctrl_auto_tune.rise_time_opt = nl_pos_ctrl_auto_tune.counter_max;

            }
        }
        else if(nl_pos_ctrl_auto_tune.step1_completed==1)
        {
            /*
             * damp oscillation
             */
            if(nl_pos_ctrl_auto_tune.overshoot_max<((nl_pos_ctrl_auto_tune.overshot_per_thousand*nl_pos_ctrl_auto_tune.step_amplitude)/1000))
            {
                nl_pos_ctrl_auto_tune.overshoot_counter++;

                /*
                 * save overshoot min over the first round
                 */
                if(nl_pos_ctrl_auto_tune.overshoot_max<nl_pos_ctrl_auto_tune.overshoot_min && nl_pos_ctrl_auto_tune.overshoot_max!=0 && nl_pos_ctrl_auto_tune.overshoot_1st_round_damped==0)
                    nl_pos_ctrl_auto_tune.overshoot_min = nl_pos_ctrl_auto_tune.overshoot_max;

            }
            else
            {
                if(nl_pos_ctrl_auto_tune.tuning_process_ended==0)
                    motion_ctrl_config.position_ki -= 10;

                if(motion_ctrl_config.position_ki<0 && nl_pos_ctrl_auto_tune.tuning_process_ended==0)
                    motion_ctrl_config.position_ki += 10;

                nl_pos_ctrl_auto_tune.overshoot_counter=0;
            }

            /*
             * ovreshoot is low enough for 10 consequtive times after the reduction of ki
             * - update the energy of error
             * - for the first time, set the overshoot_1st_round_damped to 1.
             */
            if(nl_pos_ctrl_auto_tune.overshoot_counter==10)
            {
                nl_pos_ctrl_auto_tune.dynamic_error_energy_int_max = nl_pos_ctrl_auto_tune.err_energy_int;
                nl_pos_ctrl_auto_tune.overshoot_1st_round_damped=1;
            }

            /*
             * now the overshoot is reduced enough, and we can reduce the energy of the error
             */
            if(nl_pos_ctrl_auto_tune.overshoot_counter>10)
            {
                if(nl_pos_ctrl_auto_tune.err_energy_int > (nl_pos_ctrl_auto_tune.dynamic_error_energy_int_max/10) && nl_pos_ctrl_auto_tune.tuning_process_ended==0 )
                {
                    motion_ctrl_config.position_integral_limit += 200;
                }

                if(nl_pos_ctrl_auto_tune.rise_time<nl_pos_ctrl_auto_tune.rise_time_opt && nl_pos_ctrl_auto_tune.rise_time!=0) nl_pos_ctrl_auto_tune.rise_time_opt = nl_pos_ctrl_auto_tune.rise_time;

                if(nl_pos_ctrl_auto_tune.err_energy_ss_int<nl_pos_ctrl_auto_tune.err_energy_ss_int_min && nl_pos_ctrl_auto_tune.err_energy_ss_int!=0)
                    nl_pos_ctrl_auto_tune.err_energy_ss_int_min = nl_pos_ctrl_auto_tune.err_energy_ss_int;


                if(nl_pos_ctrl_auto_tune.err_energy_ss_int>(nl_pos_ctrl_auto_tune.err_energy_ss_int_min*10))
                {
                    nl_pos_ctrl_auto_tune.tuning_process_ended=1;
                    motion_ctrl_config.position_control_autotune = 0;
                    nl_pos_ctrl_auto_tune.activate=0;

                    motion_ctrl_config.position_kp *= motion_ctrl_config.position_integral_limit ;
                    motion_ctrl_config.position_ki *= motion_ctrl_config.position_integral_limit;
                    motion_ctrl_config.position_kd *= motion_ctrl_config.position_integral_limit;
                    motion_ctrl_config.position_integral_limit = 1000;
                    motion_ctrl_config.moment_of_inertia       = 0;

                    motion_ctrl_config.position_kp /= 1500;
                    motion_ctrl_config.position_ki /= 1500;
                    motion_ctrl_config.position_kd /= 1500;

                    printf("END OF POSITION CONTROL TUNING \n");
                    printf("kp:%i ki:%i kd:%i kl:%d \n",  motion_ctrl_config.position_kp, motion_ctrl_config.position_ki, motion_ctrl_config.position_kd, motion_ctrl_config.position_integral_limit);
                }



            }

            /*
             * do not decreas ki when rise-time decreases.
             */
            if(nl_pos_ctrl_auto_tune.rise_time > (150*nl_pos_ctrl_auto_tune.rise_time_opt)/100)
                motion_ctrl_config.position_ki += 10;
        }

        /*        nl_position_control_reset(nl_pos_ctrl);
        nl_position_control_set_parameters(nl_pos_ctrl, motion_ctrl_config, POSITION_CONTROL_LOOP_PERIOD);*/

        nl_pos_ctrl_auto_tune.overshoot=0;
        nl_pos_ctrl_auto_tune.overshoot_max=0;

        nl_pos_ctrl_auto_tune.err=0.00;
        nl_pos_ctrl_auto_tune.err_energy =0.00;
        nl_pos_ctrl_auto_tune.err_energy_int=0.00;

        nl_pos_ctrl_auto_tune.err_ss=0.00;
        nl_pos_ctrl_auto_tune.err_energy_ss =0.00;
        nl_pos_ctrl_auto_tune.err_energy_ss_int = 0.00;

        nl_pos_ctrl_auto_tune.rise_time = 0;

        nl_pos_ctrl_auto_tune.counter=0;
    }

    /*
     * measurement of error energy
     */
    nl_pos_ctrl_auto_tune.err = (nl_pos_ctrl_auto_tune.position_ref - position_k)/1000.00;
    nl_pos_ctrl_auto_tune.err_energy = nl_pos_ctrl_auto_tune.err * nl_pos_ctrl_auto_tune.err;
    nl_pos_ctrl_auto_tune.err_energy_int += nl_pos_ctrl_auto_tune.err_energy;


    /*
     * measurement of overshoot and rise time
     */
    if(nl_pos_ctrl_auto_tune.rising_edge==1)
    {
        nl_pos_ctrl_auto_tune.overshoot = position_k - nl_pos_ctrl_auto_tune.position_ref;

        if(nl_pos_ctrl_auto_tune.overshoot > nl_pos_ctrl_auto_tune.overshoot_max)
            nl_pos_ctrl_auto_tune.overshoot_max=nl_pos_ctrl_auto_tune.overshoot;

        if(position_k > (nl_pos_ctrl_auto_tune.position_init+(60*nl_pos_ctrl_auto_tune.step_amplitude)/100)  && nl_pos_ctrl_auto_tune.rise_time==0)
            nl_pos_ctrl_auto_tune.rise_time = nl_pos_ctrl_auto_tune.counter;

    }

    /*
     * measurement of error energy after steady state
     */
    if((90*nl_pos_ctrl_auto_tune.counter_max)/100<nl_pos_ctrl_auto_tune.counter && nl_pos_ctrl_auto_tune.counter<(98*nl_pos_ctrl_auto_tune.counter_max)/100 && nl_pos_ctrl_auto_tune.rising_edge==1)
    {
        nl_pos_ctrl_auto_tune.err_ss = (nl_pos_ctrl_auto_tune.position_ref - position_k);
        nl_pos_ctrl_auto_tune.err_energy_ss = nl_pos_ctrl_auto_tune.err_ss * nl_pos_ctrl_auto_tune.err_ss;
        nl_pos_ctrl_auto_tune.err_energy_ss_int += nl_pos_ctrl_auto_tune.err_energy_ss;
    }

    return 0;
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
    unsigned time_start=0, time_start_old=0, time_loop=0, time_end=0, time_free=0, time_used=0;

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

    //**************************************************************************
    //**************************************************************************
    //**************************************************************************
    NLPosCtrlAutoTuneParam nl_pos_ctrl_auto_tune;

    int step_amplitude = AUTO_TUNE_STEP_AMPLITUDE;
    int counter_max    = AUTO_TUNE_COUNTER_MAX   ;
    int per_thousand_overshoot = PER_THOUSAND_OVERSHOOT;

    // initialization of position control automatic tuning:
    motion_ctrl_config.position_control_autotune =0;


    init_nl_pos_ctrl_autotune(nl_pos_ctrl_auto_tune, step_amplitude, counter_max, per_thousand_overshoot);
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
    int temperature_ratio = motorcontrol_config.temperature_ratio;

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

                        if(motion_ctrl_config.position_control_autotune == 1)
                        {
                            if(nl_pos_ctrl_auto_tune.activate==0)
                            {
                                init_nl_pos_ctrl_autotune(nl_pos_ctrl_auto_tune, step_amplitude, counter_max, per_thousand_overshoot);
                            }

                            nl_pos_ctrl_autotune(nl_pos_ctrl_auto_tune, motion_ctrl_config, position_k);

                            if(nl_pos_ctrl_auto_tune.counter==0)
                            {
                                nl_position_control_reset(nl_pos_ctrl);
                                nl_position_control_set_parameters(nl_pos_ctrl, motion_ctrl_config, POSITION_CONTROL_LOOP_PERIOD);
                                printf("kp:%i ki:%i kd:%i j:%d \n",  motion_ctrl_config.position_kp, motion_ctrl_config.position_ki, motion_ctrl_config.position_kd, motion_ctrl_config.moment_of_inertia);

                            }


                            /*
                             * position controller
                             */
                            torque_ref_k = update_nl_position_control(nl_pos_ctrl, nl_pos_ctrl_auto_tune.position_ref, position_k_1, position_k);
                        }
                        //*************************  END OF AUTOMATIC TUNING  *************************
                        else
                        {
                            torque_ref_k = update_nl_position_control(nl_pos_ctrl, position_ref_in_k, position_k_1, position_k);
                        }


                        /*
                         * XSCOPE CALLS
                         */
                        //                            xscope_int(FIRST_STEP_COUNTER, (1000*step1_counter));
                        //                            xscope_int(FIRST_STEP_COMPLETED, (1000*step1_completed));
                        xscope_int(KI, motion_ctrl_config.position_ki);
                        //                            xscope_int(ERROR, ((int)(error)));
                        //                            xscope_int(ERROR_ENERGY, ((int)(error_energy)));
                        //                            xscope_int(ERROR_ENERGY_INTEGRAL, ((int)(error_energy_int)));
                        //                            xscope_int(ERROR_ENERGY_INTEGRAL_MAX, ((int)(error_energy_int_max)));
                        xscope_int(ERROR_ENERGY_STEADY_STATE, ((int)(nl_pos_ctrl_auto_tune.err_energy_ss_int)));
                        xscope_int(ERROR_ENERGY_STEADY_STATE_MIN, ((int)(nl_pos_ctrl_auto_tune.err_energy_ss_int_min)));
                        xscope_int(RISE_TIME, nl_pos_ctrl_auto_tune.rise_time);
                        xscope_int(RISE_TIME_OPT, nl_pos_ctrl_auto_tune.rise_time_opt);
                        //                            xscope_int(OVERSHOOT_OPT_KI, overshoot_min);
                        //xscope_int(TUNING_PROCESS_ENDED, tuning_process_ended*1000);
                        //                            xscope_int(OVERSHOOT_MAX, (int)(position_ref-initial_position+overshoot_max));
                        xscope_int(POSITION_CMD, (int)(nl_pos_ctrl_auto_tune.position_ref-nl_pos_ctrl_auto_tune.position_init)-20000);
                        xscope_int(POSITION,     (int)(position_k-nl_pos_ctrl_auto_tune.position_init)-20000);
                        //                            xscope_int(RISING_EDGE, (1000*rising_edge));
                        //                xscope_int(TUNING_COUNTER, counter);
                        //                xscope_int(KP, ((int)(tuning_kp_opt)));
                        //                xscope_int(KD, ((int)(tuning_kd_opt)));
                        xscope_int(KL, motion_ctrl_config.position_integral_limit);
                        //                xscope_int(TORQUE_REF, torque_ref_k);
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
                    //increase limit by threashold
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








                /*
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
                xscope_int(AI_A1, upstream_control_data.analogue_input_a_1);
                xscope_int(AI_A2, upstream_control_data.analogue_input_a_2);
                xscope_int(AI_B1, upstream_control_data.analogue_input_b_1);
                xscope_int(AI_B2, upstream_control_data.analogue_input_b_2);
#endif
                 */
                //xscope_int(TIME_FREE, (time_free/app_tile_usec));
                //xscope_int(TIME_LOOP, (time_loop/app_tile_usec));
                //xscope_int(TIME_USED, (time_used/app_tile_usec));

                if((time_used/app_tile_usec)>290)
                {
                    while(1)
                    {
                        printf("TIMING ERROR \n");
                    }
                }

                t :> time_end;
                time_used = time_end - time_start;

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
                break;

        case i_motion_control[int i].set_safe_torque_off_enabled():
                i_torque_control.set_brake_status(0);
                torque_enable_flag   = 0;
                velocity_enable_flag = 0;
                position_enable_flag = 0;
                i_torque_control.set_safe_torque_off_enabled();
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
