/**
 * @file auto_tune.xc
 * @brief Controllers Libraries
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <auto_tune.h>

/**
 * @brief Initializes the structure of type AutoTuneParam to start the auto tuning procedure.
 *
 * @param velocity_auto_tune    structure of type AutoTuneParam which contains velocity_auto_tuning parameters
 * @param velocity_ref          The reference velocity which will be used in auto_tuning procedure.
 *                              Note: velocity_ref should be between 50% to 100% of the rated velocity. Moreover, the supply voltage should be at its nominal value while auto_tuning is in progress.
 * @param settling_time         settling time for automatic tuning of velocity pid controller
 *
 * @return int                  the function returns 0 by default
 *  */
int init_velocity_auto_tuner(AutoTuneParam &velocity_auto_tune, int velocity_ref, double settling_time)
{
    velocity_auto_tune.enable=0;
    velocity_auto_tune.counter=0;
    velocity_auto_tune.save_counter=0;
    velocity_auto_tune.array_length=1000;
    for(int i=1; i<=velocity_auto_tune.array_length; i++) velocity_auto_tune.actual_velocity[i]=0;

    velocity_auto_tune.j = 0.00;
    velocity_auto_tune.f = 0.00;
    velocity_auto_tune.z = 0.70;
    velocity_auto_tune.st= settling_time;

    velocity_auto_tune.kp   = 0.00;
    velocity_auto_tune.ki   = 0.00;
    velocity_auto_tune.kd   = 0.00;

    velocity_auto_tune.velocity_ref = velocity_ref;
    if(velocity_auto_tune.velocity_ref>2000)
        velocity_auto_tune.velocity_ref=2000;//set the maximum value for speed evaluation to 2000 rpm

    return 0;
}

/**
 * @brief Executes the auto tuning procedure for a PID velocity controller. The results of this procedure will be the PID constants for velocity controller.
 *
 * @param velocity_auto_tune    structure of type AutoTuneParam which contains velocity_auto_tuning parameters
 * @param velocity_ref_in_k     The reference velocity which will be used in auto_tuning procedure.
 * @param velocity_k            Actual velocity of motor (in rpm) which is measured by position feedback service
 * @param period                Velocity control execution period (in micro-seconds).

 * @return int                  the function returns 0 by default
 *  */
int velocity_controller_auto_tune(AutoTuneParam &velocity_auto_tune, double &velocity_ref_in_k, double velocity_k, int period)
{

    double wn_auto_tune   = 0.00;

    double steady_state = 0.00;
    double k = 0.00;
    double g_speed = 0.00;
    double speed_integral = 0.00;
    int    saving_reduction_factor = 4;//reduces the number of saved states

    velocity_auto_tune.save_counter++;
    if(velocity_auto_tune.save_counter==saving_reduction_factor)
    {
        velocity_auto_tune.save_counter=0;
        velocity_auto_tune.counter++;
    }

    if(1<=velocity_auto_tune.counter && velocity_auto_tune.counter<=velocity_auto_tune.array_length)
    {
        velocity_auto_tune.enable = 1;

        velocity_ref_in_k = velocity_auto_tune.velocity_ref;
        velocity_auto_tune.actual_velocity[velocity_auto_tune.counter] = ((int)(velocity_k));

        if(velocity_auto_tune.counter==velocity_auto_tune.array_length)
        {
            steady_state = 0;
            for(int i=(velocity_auto_tune.array_length-100); i<=velocity_auto_tune.array_length; i++)
                steady_state +=  ((double)velocity_auto_tune.actual_velocity[i]);
            steady_state = steady_state/100.00;

            k = (steady_state / velocity_auto_tune.velocity_ref);
            g_speed = 60.00/(2.00*3.1416);

            speed_integral=0.00;
            for(int i=1; i<=velocity_auto_tune.array_length; i++)
                speed_integral += (steady_state-((double)(velocity_auto_tune.actual_velocity[i])));

            speed_integral *= ((saving_reduction_factor*period)/1000000.00);
            speed_integral /= g_speed;

            velocity_auto_tune.j = (speed_integral*0.001);                  //k_m = 0.001
            velocity_auto_tune.j/= (steady_state/g_speed);
            velocity_auto_tune.j/= (steady_state/g_speed);
            velocity_auto_tune.j*= (velocity_auto_tune.velocity_ref);

            velocity_auto_tune.f = (velocity_auto_tune.velocity_ref*0.001);
            velocity_auto_tune.f/= (steady_state/g_speed);
            velocity_auto_tune.f-= (0.001*g_speed);

            if(velocity_auto_tune.f>0 && velocity_auto_tune.j>0)
            {
                wn_auto_tune = 4.00 / (velocity_auto_tune.z * velocity_auto_tune.st);
                velocity_auto_tune.kp = 1.00/(0.001*g_speed);
                velocity_auto_tune.kp*= ((2.00 * velocity_auto_tune.z * wn_auto_tune*velocity_auto_tune.j)-velocity_auto_tune.f);

                velocity_auto_tune.ki = (wn_auto_tune*wn_auto_tune*velocity_auto_tune.j);
                velocity_auto_tune.ki/= (0.001*g_speed);

                velocity_auto_tune.kp *= 1000000.00;
                velocity_auto_tune.ki *= 1000.00;

                if(velocity_auto_tune.kp<0 || velocity_auto_tune.ki<0) //wrong calculation
                {
                    velocity_auto_tune.kp = 00.00;
                    velocity_auto_tune.ki = 00.00;
                }
            }
            else //wrong calculation
            {
                velocity_auto_tune.kp = 00.00;
                velocity_auto_tune.ki = 00.00;
            }
        }
    }
    else if (velocity_auto_tune.counter<=((velocity_auto_tune.array_length*5)/4))//try to reach 0 rpm in 20% of testint time
    {
        velocity_auto_tune.enable = 1;
        velocity_ref_in_k = 0;
    }
    else
    {
        velocity_auto_tune.enable = 0;
        velocity_auto_tune.counter=0;
        for(int i=0; i<=velocity_auto_tune.array_length; i++) velocity_auto_tune.actual_velocity[i] = 0;
    }

    return 0;
}



/**
 * @brief function to initialize the structure of automatic tuner for limited torque position controller.
 *
 * @param lt_pos_ctrl_auto_tune         structure of type LTPosCtrlAutoTuneParam which will be used during the tuning procedure
 * @param step_amplitude Communication  The tuning procedure uses steps to evaluate the response of controller. This input is equal to half of step command amplitude.
 * @param counter_max                   The period of step commands in ticks. Each tick is corresponding to one execution sycle of motion_control_service. As a result, 3000 ticks when the frequency of motion_control_service is 1 ms leads to a period equal to 3 seconds for each step command.
 * @param overshot_per_thousand         Overshoot limit while tuning (it is set as per thousand of step amplitude)
 * @param rise_time_freedom_percent     This value helps the tuner to find out whether the ki is high enough or not. By default set this value to 300, and if the tuner is not able to find proper values (and the response is having oscillations), increase this value to 400 or 500.
 *
 * @return void
 *  */
int init_lt_pos_ctrl_autotune(LTPosCtrlAutoTuneParam &lt_pos_ctrl_auto_tune, int controller_type, int counter_max_autotune, int step_amplitude_autotune, int per_thousand_overshoot_autotune, int rise_time_freedom_percent_autotune)
{
    lt_pos_ctrl_auto_tune.controller = controller_type;

    lt_pos_ctrl_auto_tune.position_init = 0.00;
    lt_pos_ctrl_auto_tune.position_ref  = 0.00;
    lt_pos_ctrl_auto_tune.auto_tune  = 0;

    lt_pos_ctrl_auto_tune.activate=0;
    lt_pos_ctrl_auto_tune.counter=0;
    lt_pos_ctrl_auto_tune.counter_max=counter_max_autotune;

    lt_pos_ctrl_auto_tune.step_amplitude = step_amplitude_autotune;

    lt_pos_ctrl_auto_tune.err=0.00;
    lt_pos_ctrl_auto_tune.err_energy=0.00;
    lt_pos_ctrl_auto_tune.err_energy_int    =0.00;
    lt_pos_ctrl_auto_tune.err_energy_int_max    = ((2*lt_pos_ctrl_auto_tune.step_amplitude)/1000) * ((2*lt_pos_ctrl_auto_tune.step_amplitude)/1000) * lt_pos_ctrl_auto_tune.counter_max;
    lt_pos_ctrl_auto_tune.dynamic_err_energy_int_max=0.00;

    lt_pos_ctrl_auto_tune.err_ss=0.00;
    lt_pos_ctrl_auto_tune.err_energy_ss=0.00;
    lt_pos_ctrl_auto_tune.err_energy_ss_int=0.00;
    //steady state error is measured between 90% and 98% of the period, and its min value is when real position is having a margin of 0.01 with reference position
    lt_pos_ctrl_auto_tune.err_energy_ss_limit_soft = (((lt_pos_ctrl_auto_tune.step_amplitude)/100) * ((lt_pos_ctrl_auto_tune.step_amplitude)/100) * lt_pos_ctrl_auto_tune.counter_max * 8.00) /100.00;
    lt_pos_ctrl_auto_tune.err_energy_ss_int_min    = lt_pos_ctrl_auto_tune.err_energy_ss_limit_soft;
    lt_pos_ctrl_auto_tune.err_energy_ss_int_min_counter=0;

    lt_pos_ctrl_auto_tune.active_step_counter=0;
    if(lt_pos_ctrl_auto_tune.controller==LIMITED_TORQUE)
        lt_pos_ctrl_auto_tune.active_step=LT_POS_CTRL_STEP1;
    else if(lt_pos_ctrl_auto_tune.controller==CASCADED)
        lt_pos_ctrl_auto_tune.active_step=CASCADED_POS_CTRL_STEP1;

    lt_pos_ctrl_auto_tune.rising_edge=0;

    lt_pos_ctrl_auto_tune.overshoot=0.00;
    lt_pos_ctrl_auto_tune.overshoot_max=0.00;
    lt_pos_ctrl_auto_tune.overshoot_min=lt_pos_ctrl_auto_tune.step_amplitude;
    lt_pos_ctrl_auto_tune.overshot_per_thousand = ((double)(per_thousand_overshoot_autotune));
    lt_pos_ctrl_auto_tune.overshoot_counter=0;
    lt_pos_ctrl_auto_tune.overshoot_1st_round_damped=0;

    lt_pos_ctrl_auto_tune.rise_time=0;
    lt_pos_ctrl_auto_tune.rise_time_opt=0;
    lt_pos_ctrl_auto_tune.rise_time_freedom_percent=rise_time_freedom_percent_autotune;

    lt_pos_ctrl_auto_tune.tuning_process_ended=0;

    return 0;
}


/**
 * @brief function to automatically tune the limited torque position controller.
 *
 * @param lt_pos_ctrl_auto_tune            Structure containing all parameters of motion control service
 * @param lt_pos_ctrl_auto_tune         structure of type LTPosCtrlAutoTuneParam which will be used during the tuning procedure
 * @param position_k                    The actual position of the system while (double)
 *
 * @return void
 *  */
int lt_pos_ctrl_autotune(LTPosCtrlAutoTuneParam &lt_pos_ctrl_auto_tune, double position_k)
{

    lt_pos_ctrl_auto_tune.counter++;

    if(lt_pos_ctrl_auto_tune.controller == CASCADED)
    {
        if(lt_pos_ctrl_auto_tune.activate==0)
        {
            lt_pos_ctrl_auto_tune.position_init = position_k;


            lt_pos_ctrl_auto_tune.kpp = 0 ;
            lt_pos_ctrl_auto_tune.kpi = 0 ;
            lt_pos_ctrl_auto_tune.kpd = 0 ;
            lt_pos_ctrl_auto_tune.kpl = 10000000;
            lt_pos_ctrl_auto_tune.j  = 0;

            lt_pos_ctrl_auto_tune.kvp = 0 ;
            lt_pos_ctrl_auto_tune.kvi = 0 ;
            lt_pos_ctrl_auto_tune.kvd = 0 ;
            lt_pos_ctrl_auto_tune.kvl = 10000000;

            lt_pos_ctrl_auto_tune.activate = 1;
            lt_pos_ctrl_auto_tune.counter=0;
        }

        if(lt_pos_ctrl_auto_tune.counter==lt_pos_ctrl_auto_tune.counter_max)
        {
            //change of reference value in each cycle
            if(lt_pos_ctrl_auto_tune.position_ref == (lt_pos_ctrl_auto_tune.position_init + lt_pos_ctrl_auto_tune.step_amplitude))
            {
                lt_pos_ctrl_auto_tune.position_ref = lt_pos_ctrl_auto_tune.position_init - lt_pos_ctrl_auto_tune.step_amplitude;
                lt_pos_ctrl_auto_tune.rising_edge=0;
            }
            else
            {
                lt_pos_ctrl_auto_tune.position_ref = lt_pos_ctrl_auto_tune.position_init + lt_pos_ctrl_auto_tune.step_amplitude;
                lt_pos_ctrl_auto_tune.rising_edge=1;
            }

            // force the load to follow the reference value
            if(lt_pos_ctrl_auto_tune.active_step==CASCADED_POS_CTRL_STEP1)//step 1
            {
                if(lt_pos_ctrl_auto_tune.err_energy_int < (lt_pos_ctrl_auto_tune.err_energy_int_max/10))
                {
                    lt_pos_ctrl_auto_tune.active_step_counter++;
                }
                else
                {
                    if(lt_pos_ctrl_auto_tune.active_step_counter==0)
                        lt_pos_ctrl_auto_tune.kvp += 50000;
                    else
                        lt_pos_ctrl_auto_tune.kvp += 10000;

                        lt_pos_ctrl_auto_tune.kpp = lt_pos_ctrl_auto_tune.kvp/10;

                    lt_pos_ctrl_auto_tune.active_step_counter=0;
                }

                if(lt_pos_ctrl_auto_tune.active_step_counter==10)
                {
                    lt_pos_ctrl_auto_tune.active_step=CASCADED_POS_CTRL_STEP2;
                    lt_pos_ctrl_auto_tune.active_step_counter=0;
                }

                if(lt_pos_ctrl_auto_tune.err_energy_ss_int>lt_pos_ctrl_auto_tune.err_energy_ss_int_min)
                    lt_pos_ctrl_auto_tune.err_energy_ss_int_min = (lt_pos_ctrl_auto_tune.err_energy_ss_int_min+lt_pos_ctrl_auto_tune.err_energy_ss_int)/2;
            }


            //increase kvp until it starts to vibrate
            if(lt_pos_ctrl_auto_tune.active_step==CASCADED_POS_CTRL_STEP2)// step 2
            {
                if(lt_pos_ctrl_auto_tune.err_energy_ss_int < (3*lt_pos_ctrl_auto_tune.err_energy_ss_int_min)/2)
                {
                    lt_pos_ctrl_auto_tune.kvp = (lt_pos_ctrl_auto_tune.kvp*1005)/1000;
                    lt_pos_ctrl_auto_tune.kvi = lt_pos_ctrl_auto_tune.kvp/10000;

                    lt_pos_ctrl_auto_tune.kpp = lt_pos_ctrl_auto_tune.kvp/10;
                    lt_pos_ctrl_auto_tune.kpi = lt_pos_ctrl_auto_tune.kpp/10000;
                }
                else
                {
                    lt_pos_ctrl_auto_tune.active_step_counter++;
                }

                if(100<lt_pos_ctrl_auto_tune.err_energy_ss_int && lt_pos_ctrl_auto_tune.err_energy_ss_int<lt_pos_ctrl_auto_tune.err_energy_ss_int_min)
                    lt_pos_ctrl_auto_tune.err_energy_ss_int_min = (2*lt_pos_ctrl_auto_tune.err_energy_ss_int+lt_pos_ctrl_auto_tune.err_energy_ss_int_min)/3;


                if(lt_pos_ctrl_auto_tune.active_step_counter==10)
                {
                    lt_pos_ctrl_auto_tune.active_step=CASCADED_POS_CTRL_STEP3;

                    lt_pos_ctrl_auto_tune.kvp=(lt_pos_ctrl_auto_tune.kvp*90)/100;
                    lt_pos_ctrl_auto_tune.kvi = lt_pos_ctrl_auto_tune.kvp/10000;
                    lt_pos_ctrl_auto_tune.kpp = lt_pos_ctrl_auto_tune.kvp/10;
                    lt_pos_ctrl_auto_tune.kpi = lt_pos_ctrl_auto_tune.kpp/10000;

                    lt_pos_ctrl_auto_tune.active_step_counter=0;
                }
            }

            //increase kvi until overshoot is less than 1%
            if(lt_pos_ctrl_auto_tune.active_step==CASCADED_POS_CTRL_STEP3)//step 3
            {
                if(lt_pos_ctrl_auto_tune.overshoot_max>((10*lt_pos_ctrl_auto_tune.step_amplitude)/1000))
                {
                    lt_pos_ctrl_auto_tune.active_step_counter++;
                }
                else
                {
                    lt_pos_ctrl_auto_tune.kvi = (lt_pos_ctrl_auto_tune.kvi*110)/100;
                }

                if(lt_pos_ctrl_auto_tune.err_energy_ss_int>lt_pos_ctrl_auto_tune.err_energy_ss_int_min)
                         lt_pos_ctrl_auto_tune.err_energy_ss_int_min = (lt_pos_ctrl_auto_tune.err_energy_ss_int_min+lt_pos_ctrl_auto_tune.err_energy_ss_int)/2;

                if(lt_pos_ctrl_auto_tune.active_step_counter==10)
                {
                    lt_pos_ctrl_auto_tune.active_step=CASCADED_POS_CTRL_STEP4;
                    lt_pos_ctrl_auto_tune.kvi = (lt_pos_ctrl_auto_tune.kvi*90)/100;
                    lt_pos_ctrl_auto_tune.err_energy_ss_int_min =
                    lt_pos_ctrl_auto_tune.active_step_counter=0;
                }
            }

            //increase kpp until it starts to vibrate
            if(lt_pos_ctrl_auto_tune.active_step==CASCADED_POS_CTRL_STEP4)// step 4
            {
                if(lt_pos_ctrl_auto_tune.err_energy_ss_int < (2*lt_pos_ctrl_auto_tune.err_energy_ss_int_min))
                {
                    lt_pos_ctrl_auto_tune.kpp = (lt_pos_ctrl_auto_tune.kpp*1010)/1000;
                }
                else
                {
                    lt_pos_ctrl_auto_tune.active_step_counter++;
                }

                if(lt_pos_ctrl_auto_tune.err_energy_ss_int<lt_pos_ctrl_auto_tune.err_energy_ss_int_min)
                    lt_pos_ctrl_auto_tune.err_energy_ss_int_min = (2*lt_pos_ctrl_auto_tune.err_energy_ss_int+lt_pos_ctrl_auto_tune.err_energy_ss_int_min)/3;


                if(lt_pos_ctrl_auto_tune.active_step_counter==10)
                {
                    lt_pos_ctrl_auto_tune.active_step=CASCADED_POS_CTRL_STEP5;

                    lt_pos_ctrl_auto_tune.kpp = (lt_pos_ctrl_auto_tune.kpp*90)/100;

                    lt_pos_ctrl_auto_tune.active_step_counter=0;
                }
            }
            //
            //    if(lt_pos_ctrl_auto_tune.active_step==LT_POS_CTRL_STEP3)// step 3
            //    {
            //        if(lt_pos_ctrl_auto_tune.err_energy_ss_int < 10*lt_pos_ctrl_auto_tune.err_energy_ss_int_min)
            //        {
            //            lt_pos_ctrl_auto_tune.kpl+=100;
            //        }
            //        else
            //        {
            //            lt_pos_ctrl_auto_tune.active_step_counter++;
            //        }
            //
            //        if(lt_pos_ctrl_auto_tune.err_energy_ss_int<lt_pos_ctrl_auto_tune.err_energy_ss_int_min)
            //            lt_pos_ctrl_auto_tune.err_energy_ss_int_min = (lt_pos_ctrl_auto_tune.err_energy_ss_int+(3.00*lt_pos_ctrl_auto_tune.err_energy_ss_int_min))/4;
            //
            //
            //        if(lt_pos_ctrl_auto_tune.active_step_counter==10)
            //        {
            //            lt_pos_ctrl_auto_tune.active_step=LT_POS_CTRL_STEP4;
            //            lt_pos_ctrl_auto_tune.kpl= (lt_pos_ctrl_auto_tune.kpl*100)/140;
            //            lt_pos_ctrl_auto_tune.active_step_counter=0;
            //        }
            //    }
            //
            //    if(lt_pos_ctrl_auto_tune.active_step==LT_POS_CTRL_STEP4)//step 4
            //    {
            //        if(lt_pos_ctrl_auto_tune.overshoot_max<((20*lt_pos_ctrl_auto_tune.step_amplitude)/1000) && lt_pos_ctrl_auto_tune.err_energy_ss_int<(lt_pos_ctrl_auto_tune.err_energy_ss_limit_soft))
            //        {
            //            lt_pos_ctrl_auto_tune.active_step_counter++;
            //        }
            //        else
            //        {
            //            if(lt_pos_ctrl_auto_tune.tuning_process_ended==0)
            //                lt_pos_ctrl_auto_tune.kpi -= 5;
            //
            //            if(lt_pos_ctrl_auto_tune.kpi<5 && lt_pos_ctrl_auto_tune.tuning_process_ended==0)
            //            {
            //                lt_pos_ctrl_auto_tune.kpi = 5;
            //                lt_pos_ctrl_auto_tune.active_step_counter++;
            //            }
            //            else
            //                lt_pos_ctrl_auto_tune.active_step_counter=0;
            //        }
            //
            //        if(lt_pos_ctrl_auto_tune.active_step_counter==10)
            //        {
            //            lt_pos_ctrl_auto_tune.active_step=LT_POS_CTRL_STEP5;
            //            lt_pos_ctrl_auto_tune.active_step_counter=0;
            //
            //            lt_pos_ctrl_auto_tune.err_energy_ss_int_min    = lt_pos_ctrl_auto_tune.err_energy_ss_limit_soft;
            //        }
            //
            //    }
            //
            //    if(lt_pos_ctrl_auto_tune.active_step==LT_POS_CTRL_STEP5)// step 5
            //    {
            //
            //        if(lt_pos_ctrl_auto_tune.err_energy_ss_int < 10*lt_pos_ctrl_auto_tune.err_energy_ss_int_min)
            //        {
            //            lt_pos_ctrl_auto_tune.kpl+=50;
            //        }
            //        else
            //        {
            //            lt_pos_ctrl_auto_tune.active_step_counter++;
            //        }
            //
            //        if(lt_pos_ctrl_auto_tune.err_energy_ss_int<lt_pos_ctrl_auto_tune.err_energy_ss_int_min)
            //            lt_pos_ctrl_auto_tune.err_energy_ss_int_min = (lt_pos_ctrl_auto_tune.err_energy_ss_int+(3.00*lt_pos_ctrl_auto_tune.err_energy_ss_int_min))/4.00;
            //
            //        if(lt_pos_ctrl_auto_tune.active_step_counter==10)
            //        {
            //            lt_pos_ctrl_auto_tune.active_step=END;
            //
            //            lt_pos_ctrl_auto_tune.tuning_process_ended=1;
            //            lt_pos_ctrl_auto_tune.auto_tune = 0;
            //            lt_pos_ctrl_auto_tune.activate=0;
            //
            //            lt_pos_ctrl_auto_tune.kpp *= lt_pos_ctrl_auto_tune.kpl;
            //            lt_pos_ctrl_auto_tune.kpi *= lt_pos_ctrl_auto_tune.kpl;
            //            lt_pos_ctrl_auto_tune.kpd *= lt_pos_ctrl_auto_tune.kpl;
            //            lt_pos_ctrl_auto_tune.j       = 0;
            //            lt_pos_ctrl_auto_tune.kpl = 1000;
            //
            //            lt_pos_ctrl_auto_tune.kpp /= 1000;
            //            lt_pos_ctrl_auto_tune.kpi /= 1000;
            //            lt_pos_ctrl_auto_tune.kpd /= 1000;
            //
            //            lt_pos_ctrl_auto_tune.active_step_counter=0;
            //        }
            //    }

            lt_pos_ctrl_auto_tune.overshoot=0;
            lt_pos_ctrl_auto_tune.overshoot_max=0;

            lt_pos_ctrl_auto_tune.err=0.00;
            lt_pos_ctrl_auto_tune.err_energy =0.00;
            lt_pos_ctrl_auto_tune.err_energy_int=0.00;

            lt_pos_ctrl_auto_tune.err_ss=0.00;
            lt_pos_ctrl_auto_tune.err_energy_ss =0.00;
            lt_pos_ctrl_auto_tune.err_energy_ss_int = 0.00;

            lt_pos_ctrl_auto_tune.rise_time = 0;

            lt_pos_ctrl_auto_tune.counter=0;
        }
    }
    else if(lt_pos_ctrl_auto_tune.controller == LIMITED_TORQUE)
    {
        if(lt_pos_ctrl_auto_tune.activate==0)
        {
            lt_pos_ctrl_auto_tune.position_init = position_k;

            lt_pos_ctrl_auto_tune.kpp = 10000 ;
            lt_pos_ctrl_auto_tune.kpi = 1000  ;
            lt_pos_ctrl_auto_tune.kpd = 40000 ;
            lt_pos_ctrl_auto_tune.kpl = 1;
            lt_pos_ctrl_auto_tune.j  = 0;

            lt_pos_ctrl_auto_tune.activate = 1;
            lt_pos_ctrl_auto_tune.counter=0;
        }

        if(lt_pos_ctrl_auto_tune.counter==lt_pos_ctrl_auto_tune.counter_max)
        {
            //change of reference value in each cycle
            if(lt_pos_ctrl_auto_tune.position_ref == (lt_pos_ctrl_auto_tune.position_init + lt_pos_ctrl_auto_tune.step_amplitude))
            {
                lt_pos_ctrl_auto_tune.position_ref = lt_pos_ctrl_auto_tune.position_init - lt_pos_ctrl_auto_tune.step_amplitude;
                lt_pos_ctrl_auto_tune.rising_edge=0;
            }
            else
            {
                lt_pos_ctrl_auto_tune.position_ref = lt_pos_ctrl_auto_tune.position_init + lt_pos_ctrl_auto_tune.step_amplitude;
                lt_pos_ctrl_auto_tune.rising_edge=1;
            }

            if(lt_pos_ctrl_auto_tune.active_step==LT_POS_CTRL_STEP1)//step 1
            {
                if(lt_pos_ctrl_auto_tune.err_energy_int < (lt_pos_ctrl_auto_tune.err_energy_int_max/10))
                {
                    lt_pos_ctrl_auto_tune.active_step_counter++;
                }
                else
                {
                    lt_pos_ctrl_auto_tune.kpl += 50;
                    lt_pos_ctrl_auto_tune.active_step_counter=0;
                }

                if(lt_pos_ctrl_auto_tune.active_step_counter==10)
                {
                    lt_pos_ctrl_auto_tune.active_step=LT_POS_CTRL_STEP2;
                    lt_pos_ctrl_auto_tune.active_step_counter=0;
                }
            }

            if(lt_pos_ctrl_auto_tune.active_step==LT_POS_CTRL_STEP2)//step 2
            {
                if(lt_pos_ctrl_auto_tune.overshoot_max<((100*lt_pos_ctrl_auto_tune.step_amplitude)/1000) && lt_pos_ctrl_auto_tune.err_energy_ss_int<lt_pos_ctrl_auto_tune.err_energy_ss_limit_soft)
                {
                    lt_pos_ctrl_auto_tune.active_step_counter++;
                }
                else
                {
                    if(lt_pos_ctrl_auto_tune.tuning_process_ended==0)
                        lt_pos_ctrl_auto_tune.kpi = (lt_pos_ctrl_auto_tune.kpi*100)/120;

                    if(lt_pos_ctrl_auto_tune.kpi<10 && lt_pos_ctrl_auto_tune.tuning_process_ended==0)
                        lt_pos_ctrl_auto_tune.kpi = 10;

                    lt_pos_ctrl_auto_tune.active_step_counter=0;
                }

                if(lt_pos_ctrl_auto_tune.active_step_counter==10)
                {
                    lt_pos_ctrl_auto_tune.active_step=LT_POS_CTRL_STEP3;
                    lt_pos_ctrl_auto_tune.active_step_counter=0;

                }

            }

            if(lt_pos_ctrl_auto_tune.active_step==LT_POS_CTRL_STEP3)// step 3
            {
                if(lt_pos_ctrl_auto_tune.err_energy_ss_int < 10*lt_pos_ctrl_auto_tune.err_energy_ss_int_min)
                {
                    lt_pos_ctrl_auto_tune.kpl+=100;
                }
                else
                {
                    lt_pos_ctrl_auto_tune.active_step_counter++;
                }

                if(lt_pos_ctrl_auto_tune.err_energy_ss_int<lt_pos_ctrl_auto_tune.err_energy_ss_int_min)
                    lt_pos_ctrl_auto_tune.err_energy_ss_int_min = (lt_pos_ctrl_auto_tune.err_energy_ss_int+(3.00*lt_pos_ctrl_auto_tune.err_energy_ss_int_min))/4;


                if(lt_pos_ctrl_auto_tune.active_step_counter==10)
                {
                    lt_pos_ctrl_auto_tune.active_step=LT_POS_CTRL_STEP4;
                    lt_pos_ctrl_auto_tune.kpl= (lt_pos_ctrl_auto_tune.kpl*100)/140;
                    lt_pos_ctrl_auto_tune.active_step_counter=0;
                }
            }

            if(lt_pos_ctrl_auto_tune.active_step==LT_POS_CTRL_STEP4)//step 4
            {
                if(lt_pos_ctrl_auto_tune.overshoot_max<((20*lt_pos_ctrl_auto_tune.step_amplitude)/1000) && lt_pos_ctrl_auto_tune.err_energy_ss_int<(lt_pos_ctrl_auto_tune.err_energy_ss_limit_soft))
                {
                    lt_pos_ctrl_auto_tune.active_step_counter++;
                }
                else
                {
                    if(lt_pos_ctrl_auto_tune.tuning_process_ended==0)
                        lt_pos_ctrl_auto_tune.kpi -= 5;

                    if(lt_pos_ctrl_auto_tune.kpi<5 && lt_pos_ctrl_auto_tune.tuning_process_ended==0)
                    {
                        lt_pos_ctrl_auto_tune.kpi = 5;
                        lt_pos_ctrl_auto_tune.active_step_counter++;
                    }
                    else
                        lt_pos_ctrl_auto_tune.active_step_counter=0;
                }

                if(lt_pos_ctrl_auto_tune.active_step_counter==10)
                {
                    lt_pos_ctrl_auto_tune.active_step=LT_POS_CTRL_STEP5;
                    lt_pos_ctrl_auto_tune.active_step_counter=0;

                    lt_pos_ctrl_auto_tune.err_energy_ss_int_min    = lt_pos_ctrl_auto_tune.err_energy_ss_limit_soft;
                }

            }

            if(lt_pos_ctrl_auto_tune.active_step==LT_POS_CTRL_STEP5)// step 5
            {

                if(lt_pos_ctrl_auto_tune.err_energy_ss_int < 10*lt_pos_ctrl_auto_tune.err_energy_ss_int_min)
                {
                    lt_pos_ctrl_auto_tune.kpl+=50;
                }
                else
                {
                    lt_pos_ctrl_auto_tune.active_step_counter++;
                }

                if(lt_pos_ctrl_auto_tune.err_energy_ss_int<lt_pos_ctrl_auto_tune.err_energy_ss_int_min)
                    lt_pos_ctrl_auto_tune.err_energy_ss_int_min = (lt_pos_ctrl_auto_tune.err_energy_ss_int+(3.00*lt_pos_ctrl_auto_tune.err_energy_ss_int_min))/4.00;

                if(lt_pos_ctrl_auto_tune.active_step_counter==10)
                {
                    lt_pos_ctrl_auto_tune.active_step=END;

                    lt_pos_ctrl_auto_tune.tuning_process_ended=1;
                    lt_pos_ctrl_auto_tune.auto_tune = 0;
                    lt_pos_ctrl_auto_tune.activate=0;

                    lt_pos_ctrl_auto_tune.kpp *= lt_pos_ctrl_auto_tune.kpl;
                    lt_pos_ctrl_auto_tune.kpi *= lt_pos_ctrl_auto_tune.kpl;
                    lt_pos_ctrl_auto_tune.kpd *= lt_pos_ctrl_auto_tune.kpl;
                    lt_pos_ctrl_auto_tune.j       = 0;
                    lt_pos_ctrl_auto_tune.kpl = 1000;

                    lt_pos_ctrl_auto_tune.kpp /= 1000;
                    lt_pos_ctrl_auto_tune.kpi /= 1000;
                    lt_pos_ctrl_auto_tune.kpd /= 1000;

                    lt_pos_ctrl_auto_tune.active_step_counter=0;
                }
            }

            lt_pos_ctrl_auto_tune.overshoot=0;
            lt_pos_ctrl_auto_tune.overshoot_max=0;

            lt_pos_ctrl_auto_tune.err=0.00;
            lt_pos_ctrl_auto_tune.err_energy =0.00;
            lt_pos_ctrl_auto_tune.err_energy_int=0.00;

            lt_pos_ctrl_auto_tune.err_ss=0.00;
            lt_pos_ctrl_auto_tune.err_energy_ss =0.00;
            lt_pos_ctrl_auto_tune.err_energy_ss_int = 0.00;

            lt_pos_ctrl_auto_tune.rise_time = 0;

            lt_pos_ctrl_auto_tune.counter=0;
        }
    }
    /*
     * measurement of error energy
     */
    lt_pos_ctrl_auto_tune.err = (lt_pos_ctrl_auto_tune.position_ref - position_k)/1000.00;
    lt_pos_ctrl_auto_tune.err_energy = lt_pos_ctrl_auto_tune.err * lt_pos_ctrl_auto_tune.err;
    lt_pos_ctrl_auto_tune.err_energy_int += lt_pos_ctrl_auto_tune.err_energy;

    /*
     * measurement of overshoot and rise time
     */
    if(lt_pos_ctrl_auto_tune.rising_edge==1)
    {
        lt_pos_ctrl_auto_tune.overshoot = position_k - lt_pos_ctrl_auto_tune.position_ref;

        if(lt_pos_ctrl_auto_tune.overshoot > lt_pos_ctrl_auto_tune.overshoot_max)
            lt_pos_ctrl_auto_tune.overshoot_max=lt_pos_ctrl_auto_tune.overshoot;

        if(position_k > (lt_pos_ctrl_auto_tune.position_init+(60*lt_pos_ctrl_auto_tune.step_amplitude)/100)  && lt_pos_ctrl_auto_tune.rise_time==0)
            lt_pos_ctrl_auto_tune.rise_time = lt_pos_ctrl_auto_tune.counter;

    }

    /*
     * measurement of error energy after steady state
     */
    if((90*lt_pos_ctrl_auto_tune.counter_max)/100<lt_pos_ctrl_auto_tune.counter && lt_pos_ctrl_auto_tune.counter<(98*lt_pos_ctrl_auto_tune.counter_max)/100 && lt_pos_ctrl_auto_tune.rising_edge==1)
    {
        lt_pos_ctrl_auto_tune.err_ss = (lt_pos_ctrl_auto_tune.position_ref - position_k);
        lt_pos_ctrl_auto_tune.err_energy_ss = lt_pos_ctrl_auto_tune.err_ss * lt_pos_ctrl_auto_tune.err_ss;
        lt_pos_ctrl_auto_tune.err_energy_ss_int += lt_pos_ctrl_auto_tune.err_energy_ss;
    }

    return 0;
}




