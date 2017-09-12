/**
 * @file auto_tune.xc
 * @brief Controllers Libraries
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <auto_tune.h>

int init_velocity_auto_tuner(VelCtrlAutoTuneParam &velocity_auto_tune, MotionControlConfig &motion_ctrl_config, int velocity_ref, double settling_time)
{
    velocity_auto_tune.enable=0;
    velocity_auto_tune.counter=0;
    velocity_auto_tune.save_counter=0;
    for(int i=1; i<=MEASUREMENT_ARRAY_LENGTH; i++) velocity_auto_tune.actual_velocity[i]=0;

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

int velocity_controller_auto_tune(VelCtrlAutoTuneParam &velocity_auto_tune, MotionControlConfig &motion_ctrl_config, double &velocity_ref_in_k, double velocity_k, int period)
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

    if(1<=velocity_auto_tune.counter && velocity_auto_tune.counter<=MEASUREMENT_ARRAY_LENGTH)
    {
        velocity_ref_in_k = velocity_auto_tune.velocity_ref;
        velocity_auto_tune.actual_velocity[velocity_auto_tune.counter] = ((int)(velocity_k));

        if(velocity_auto_tune.counter==MEASUREMENT_ARRAY_LENGTH)
        {
            steady_state = 0;
            for(int i=(MEASUREMENT_ARRAY_LENGTH-100); i<=MEASUREMENT_ARRAY_LENGTH; i++)
                steady_state +=  ((double)velocity_auto_tune.actual_velocity[i]);
            steady_state = steady_state/100.00;

            k = (steady_state / velocity_auto_tune.velocity_ref);
            g_speed = 60.00/(2.00*3.1416);

            speed_integral=0.00;
            for(int i=1; i<=MEASUREMENT_ARRAY_LENGTH; i++)
                speed_integral += (steady_state-((double)(velocity_auto_tune.actual_velocity[i])));

            speed_integral *= ((saving_reduction_factor*period)/1000000.00);
            speed_integral /= g_speed;

            velocity_auto_tune.j = ((speed_integral*motion_ctrl_config.velocity_kp)/1000000000);                  //k_m = 0.001 * kp
            velocity_auto_tune.j/= (steady_state/g_speed);
            velocity_auto_tune.j/= (steady_state/g_speed);
            velocity_auto_tune.j*= (velocity_auto_tune.velocity_ref);

            velocity_auto_tune.f = ((velocity_auto_tune.velocity_ref*motion_ctrl_config.velocity_kp)/1000000000);//k_m = 0.001 * kp
            velocity_auto_tune.f/= (steady_state/g_speed);
            velocity_auto_tune.f-= ((g_speed*motion_ctrl_config.velocity_kp)/1000000000);

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
    else if (velocity_auto_tune.counter<=((MEASUREMENT_ARRAY_LENGTH*5)/4))//try to reach 0 rpm in 20% of testint time
    {
        velocity_ref_in_k = 0;
    }
    else if (velocity_auto_tune.counter>((MEASUREMENT_ARRAY_LENGTH*5)/4))
    {
        velocity_auto_tune.enable = 0;
        velocity_auto_tune.counter=0;

        motion_ctrl_config.enable_velocity_auto_tuner = 0;
        motion_ctrl_config.velocity_kp = ((int)(velocity_auto_tune.kp));
        motion_ctrl_config.velocity_ki = ((int)(velocity_auto_tune.ki));
        motion_ctrl_config.velocity_kd = ((int)(velocity_auto_tune.kd));

        for(int i=0; i<=MEASUREMENT_ARRAY_LENGTH; i++) velocity_auto_tune.actual_velocity[i] = 0;
    }

    return 0;
}

int init_pos_ctrl_autotune(PosCtrlAutoTuneParam &pos_ctrl_auto_tune, MotionControlConfig &motion_ctrl_config, int controller_type)
{
    pos_ctrl_auto_tune.controller = controller_type;

    pos_ctrl_auto_tune.position_init = 0.00;
    pos_ctrl_auto_tune.position_ref  = 0.00;

    pos_ctrl_auto_tune.position_act_k= 0.00;
    pos_ctrl_auto_tune.position_act_k_1= 0.00;
    pos_ctrl_auto_tune.position_act_k_2= 0.00;

    pos_ctrl_auto_tune.velocity_k  = 0.00;
    pos_ctrl_auto_tune.velocity_k_1= 0.00;

    pos_ctrl_auto_tune.acceleration_k= 0.00;
    pos_ctrl_auto_tune.acceleration_k_filtered= 0.00;

    pos_ctrl_auto_tune.jerk_energy_limit = 0.00;

    pos_ctrl_auto_tune.auto_tune  = 0;

    pos_ctrl_auto_tune.activate=0;
    pos_ctrl_auto_tune.counter=0;
    pos_ctrl_auto_tune.counter_max=motion_ctrl_config.counter_max_autotune;

    pos_ctrl_auto_tune.step_amplitude = motion_ctrl_config.step_amplitude_autotune;

    pos_ctrl_auto_tune.err=0.00;
    pos_ctrl_auto_tune.err_energy=0.00;
    pos_ctrl_auto_tune.err_energy_int    =0.00;
    pos_ctrl_auto_tune.err_energy_int_max    = ((2*pos_ctrl_auto_tune.step_amplitude)/1000) * ((2*pos_ctrl_auto_tune.step_amplitude)/1000) * pos_ctrl_auto_tune.counter_max;

    pos_ctrl_auto_tune.err_ss=0.00;
    pos_ctrl_auto_tune.err_energy_ss=0.00;
    pos_ctrl_auto_tune.err_energy_ss_int=0.00;
    pos_ctrl_auto_tune.err_energy_ss_limit_soft = (((pos_ctrl_auto_tune.step_amplitude    )/100 ) * ((pos_ctrl_auto_tune.step_amplitude    )/100 ) * pos_ctrl_auto_tune.counter_max * 8.00) /100.00;    //steady state error is measured between 90% and 98% of the period
    pos_ctrl_auto_tune.err_energy_ss_limit_hard = (((2*pos_ctrl_auto_tune.step_amplitude  )/100 ) * ((2*pos_ctrl_auto_tune.step_amplitude  )/100 ) * pos_ctrl_auto_tune.counter_max * 8.00) /100.00;    //steady state error is measured between 90% and 98% of the period
    pos_ctrl_auto_tune.err_energy_ss_int_min    = pos_ctrl_auto_tune.err_energy_ss_limit_soft;

    pos_ctrl_auto_tune.active_step_counter=0;
    pos_ctrl_auto_tune.active_step=AUTO_TUNE_STEP_1;

    pos_ctrl_auto_tune.rising_edge=0;

    pos_ctrl_auto_tune.overshoot=0.00;
    pos_ctrl_auto_tune.overshoot_max=0.00;

    pos_ctrl_auto_tune.max_motor_speed=motion_ctrl_config.max_motor_speed;                /**< Parameter for setting the maximum motor speed */
    return 0;
}


int pos_ctrl_autotune(PosCtrlAutoTuneParam &pos_ctrl_auto_tune, MotionControlConfig &motion_ctrl_config, double position_k)
{

    pos_ctrl_auto_tune.auto_tune =  motion_ctrl_config.position_control_autotune;
    pos_ctrl_auto_tune.counter++;

    if(pos_ctrl_auto_tune.controller == CASCADED)
    {

        if(pos_ctrl_auto_tune.activate==0)
        {
            pos_ctrl_auto_tune.position_init = position_k;

            pos_ctrl_auto_tune.kpp = 0 ;
            pos_ctrl_auto_tune.kpi = 0 ;
            pos_ctrl_auto_tune.kpd = 0 ;
            pos_ctrl_auto_tune.kpl = motion_ctrl_config.position_integral_limit;
            pos_ctrl_auto_tune.j   = 0 ;

            pos_ctrl_auto_tune.kvp = 0 ;
            pos_ctrl_auto_tune.kvi = 0 ;
            pos_ctrl_auto_tune.kvd = 0 ;
            pos_ctrl_auto_tune.kvl = motion_ctrl_config.velocity_integral_limit;

            pos_ctrl_auto_tune.activate = 1;
            pos_ctrl_auto_tune.counter=0;
        }

        if(pos_ctrl_auto_tune.counter==pos_ctrl_auto_tune.counter_max)
        {
            //change of reference value in each cycle
            if(pos_ctrl_auto_tune.position_ref == (pos_ctrl_auto_tune.position_init + pos_ctrl_auto_tune.step_amplitude))
            {
                pos_ctrl_auto_tune.position_ref = pos_ctrl_auto_tune.position_init - pos_ctrl_auto_tune.step_amplitude;
                pos_ctrl_auto_tune.rising_edge=0;
            }
            else
            {
                pos_ctrl_auto_tune.position_ref = pos_ctrl_auto_tune.position_init + pos_ctrl_auto_tune.step_amplitude;
                pos_ctrl_auto_tune.rising_edge=1;
            }

            // step 1: increase kvp and kpp until it starts to vibrate or until the steady state error is less than 5%
            if(pos_ctrl_auto_tune.active_step==AUTO_TUNE_STEP_1)
            {
                if(pos_ctrl_auto_tune.err_energy_ss_int<(25*pos_ctrl_auto_tune.err_energy_ss_limit_hard))
                {
                    pos_ctrl_auto_tune.active_step_counter++;
                }
                else
                {
                    if(pos_ctrl_auto_tune.active_step_counter==0)
                        pos_ctrl_auto_tune.kvp += 50000;            //at first step, increase kvp by 50000 (corresponding to 0.05 after scaling in pid controller)
                    else                                            //and in the next steps increase it by 10000.
                        pos_ctrl_auto_tune.kvp += 10000;

                    pos_ctrl_auto_tune.kpp = pos_ctrl_auto_tune.kvp/10;

                    pos_ctrl_auto_tune.active_step_counter=0;
                }

                if(pos_ctrl_auto_tune.active_step_counter==10)
                {
                    pos_ctrl_auto_tune.active_step=AUTO_TUNE_STEP_2;
                    pos_ctrl_auto_tune.active_step_counter=0;
                }

                if(pos_ctrl_auto_tune.err_energy_ss_int>pos_ctrl_auto_tune.err_energy_ss_int_min)
                    pos_ctrl_auto_tune.err_energy_ss_int_min = (2*pos_ctrl_auto_tune.err_energy_ss_int_min+pos_ctrl_auto_tune.err_energy_ss_int)/3;
            }



            //step 2: increase kvp until it starts to vibrate or until the steady state error is less than 1%
            if(pos_ctrl_auto_tune.active_step==AUTO_TUNE_STEP_2 && pos_ctrl_auto_tune.rising_edge==0)
            {
                if(pos_ctrl_auto_tune.err_energy_ss_int < (5*pos_ctrl_auto_tune.err_energy_ss_int_min) && pos_ctrl_auto_tune.err_energy_ss_int>(pos_ctrl_auto_tune.err_energy_ss_limit_hard))
                {
                    pos_ctrl_auto_tune.kvp = (pos_ctrl_auto_tune.kvp*110)/100;//increase kvp by 10% in each step
                    pos_ctrl_auto_tune.kpp = pos_ctrl_auto_tune.kvp/10;       //set kpp equal to 0.1 of kvp in this step (the focus is on tuning of velocity loop)
                }
                else
                {
                    pos_ctrl_auto_tune.active_step_counter++;
                }

                if(100<pos_ctrl_auto_tune.err_energy_ss_int && pos_ctrl_auto_tune.err_energy_ss_int<pos_ctrl_auto_tune.err_energy_ss_int_min)
                    pos_ctrl_auto_tune.err_energy_ss_int_min = (2*pos_ctrl_auto_tune.err_energy_ss_int+pos_ctrl_auto_tune.err_energy_ss_int_min)/3;

                if(pos_ctrl_auto_tune.active_step_counter==10)
                {
                    pos_ctrl_auto_tune.active_step=AUTO_TUNE_STEP_5;

                    pos_ctrl_auto_tune.kvp=(pos_ctrl_auto_tune.kvp*90)/100; //decrease all constants by 10% after this step is completed
                    pos_ctrl_auto_tune.kpp = pos_ctrl_auto_tune.kvp/10;
                    pos_ctrl_auto_tune.active_step_counter=0;
                }
            }

            //step 5: increase kpp until steady state error becomes less than 0.3%
            if(pos_ctrl_auto_tune.active_step==AUTO_TUNE_STEP_5 && pos_ctrl_auto_tune.rising_edge==0)
            {
                if(pos_ctrl_auto_tune.err_energy_ss_int>pos_ctrl_auto_tune.err_energy_ss_limit_hard/10)
                {
                    pos_ctrl_auto_tune.kpp = (pos_ctrl_auto_tune.kpp*1050)/1000;    //increase kpp by 5% in each cycle of step-5

                    if(pos_ctrl_auto_tune.kpp<100) pos_ctrl_auto_tune.kpp =100;
                }
                else
                {
                    pos_ctrl_auto_tune.active_step_counter++;
                }

                if(pos_ctrl_auto_tune.err_energy_ss_int<pos_ctrl_auto_tune.err_energy_ss_int_min)
                    pos_ctrl_auto_tune.err_energy_ss_int_min = (2*pos_ctrl_auto_tune.err_energy_ss_int+pos_ctrl_auto_tune.err_energy_ss_int_min)/3;

                if(pos_ctrl_auto_tune.active_step_counter==10)
                {
                    pos_ctrl_auto_tune.active_step=AUTO_TUNE_STEP_6;

                    pos_ctrl_auto_tune.kpp = (pos_ctrl_auto_tune.kpp*90)/100;       //decrease kpp by 10% after step-5 of autotuning is finished
                    pos_ctrl_auto_tune.active_step_counter=0;
                }
            }

            //step 6: increase kpi until overshoot is more than 2% or steady state error is less than 0.33%
            if(pos_ctrl_auto_tune.active_step==AUTO_TUNE_STEP_6 && pos_ctrl_auto_tune.rising_edge==0)
            {
                if(pos_ctrl_auto_tune.overshoot_max>((20*2*pos_ctrl_auto_tune.step_amplitude)/1000) || pos_ctrl_auto_tune.err_energy_ss_int<pos_ctrl_auto_tune.err_energy_ss_limit_hard/10)
                {
                    pos_ctrl_auto_tune.active_step_counter++;
                }
                else
                {
                    pos_ctrl_auto_tune.kpi = (pos_ctrl_auto_tune.kpi*110)/100;      //increase kpi by 10% in each cycle of step-6
                    if(pos_ctrl_auto_tune.kpi<10) pos_ctrl_auto_tune.kpi=10;
                }

                if(pos_ctrl_auto_tune.active_step_counter==10)
                {
                    pos_ctrl_auto_tune.active_step=END;

                    pos_ctrl_auto_tune.kpp = (pos_ctrl_auto_tune.kpp*90)/100;       //decrease all constants by 10% after step-6 is completed
                    pos_ctrl_auto_tune.kpi = (pos_ctrl_auto_tune.kpi*90)/100;       //decrease all constants by 10% after step-6 is completed

                    pos_ctrl_auto_tune.kvp = (pos_ctrl_auto_tune.kvp*90)/100;       //decrease all constants by 10% after step-6 is completed

                    pos_ctrl_auto_tune.active_step_counter=0;
                }
            }

            //step 7: End autotuning
            if(pos_ctrl_auto_tune.active_step==END)
            {
                pos_ctrl_auto_tune.auto_tune = 0;
                pos_ctrl_auto_tune.activate  = 0;

                pos_ctrl_auto_tune.active_step_counter=0;
            }

            pos_ctrl_auto_tune.overshoot=0;
            pos_ctrl_auto_tune.overshoot_max=0;

            pos_ctrl_auto_tune.err=0.00;
            pos_ctrl_auto_tune.err_energy =0.00;
            pos_ctrl_auto_tune.err_energy_int=0.00;

            pos_ctrl_auto_tune.err_ss=0.00;
            pos_ctrl_auto_tune.err_energy_ss =0.00;
            pos_ctrl_auto_tune.err_energy_ss_int = 0.00;

            pos_ctrl_auto_tune.counter=0;
        }

    }
    else if(pos_ctrl_auto_tune.controller == LIMITED_TORQUE)
    {
        if(pos_ctrl_auto_tune.activate==0)
        {
            pos_ctrl_auto_tune.position_init = position_k;

            pos_ctrl_auto_tune.kpp = 10000 ;
            pos_ctrl_auto_tune.kpi = 10    ;
            pos_ctrl_auto_tune.kpd = 40000 ;
            pos_ctrl_auto_tune.kpl = 1;
            pos_ctrl_auto_tune.j   = 0;

            pos_ctrl_auto_tune.kvp = 0 ;
            pos_ctrl_auto_tune.kvi = 0 ;
            pos_ctrl_auto_tune.kvd = 0 ;
            pos_ctrl_auto_tune.kvl = 0 ;

            pos_ctrl_auto_tune.activate = 1;
            pos_ctrl_auto_tune.counter=0;
        }

        if(pos_ctrl_auto_tune.counter==pos_ctrl_auto_tune.counter_max)
        {
            //change of reference value in each cycle
            if(pos_ctrl_auto_tune.position_ref == (pos_ctrl_auto_tune.position_init + pos_ctrl_auto_tune.step_amplitude))
            {
                pos_ctrl_auto_tune.position_ref = pos_ctrl_auto_tune.position_init - pos_ctrl_auto_tune.step_amplitude;
                pos_ctrl_auto_tune.rising_edge=0;
            }
            else
            {
                pos_ctrl_auto_tune.position_ref = pos_ctrl_auto_tune.position_init + pos_ctrl_auto_tune.step_amplitude;
                pos_ctrl_auto_tune.rising_edge=1;
            }

            //step 1: increase kpl until the load follows the reference value (steady state error is less than 20%)
            if(pos_ctrl_auto_tune.active_step==AUTO_TUNE_STEP_1)
            {
                if(pos_ctrl_auto_tune.err_energy_ss_int < pos_ctrl_auto_tune.err_energy_ss_limit_hard*400)
                {
                    pos_ctrl_auto_tune.active_step_counter++;
                }
                else
                {
                    int increment = (pos_ctrl_auto_tune.kpl*5)/100;
                    if(increment<10)
                        pos_ctrl_auto_tune.kpl += 10;
                    else
                        pos_ctrl_auto_tune.kpl += increment;

                    pos_ctrl_auto_tune.active_step_counter=0;
                }

                if(pos_ctrl_auto_tune.active_step_counter==10)
                {
                    pos_ctrl_auto_tune.active_step=AUTO_TUNE_STEP_2;
                    pos_ctrl_auto_tune.err_energy_ss_int_min = pos_ctrl_auto_tune.err_energy_ss_limit_hard;
                    pos_ctrl_auto_tune.active_step_counter=0;
                }
            }

            //step 2: incraese kpi until overshoot is less than 15%
            if(pos_ctrl_auto_tune.active_step==AUTO_TUNE_STEP_2 && pos_ctrl_auto_tune.rising_edge==0)
            {
                if(pos_ctrl_auto_tune.overshoot_max>(15*pos_ctrl_auto_tune.step_amplitude)/100)
                {
                    pos_ctrl_auto_tune.active_step_counter++;
                }
                else
                {
                    pos_ctrl_auto_tune.kpi = (pos_ctrl_auto_tune.kpi*110)/100;      //increase kpi by 10% in each cycle of step-2 of autotuning

                    if(pos_ctrl_auto_tune.kpi<1000)
                        pos_ctrl_auto_tune.active_step_counter=0;
                    else
                        pos_ctrl_auto_tune.active_step_counter=10;
                }

                if(pos_ctrl_auto_tune.active_step_counter==10)
                {
                    pos_ctrl_auto_tune.active_step=AUTO_TUNE_STEP_3;
                    pos_ctrl_auto_tune.kpi_min = (pos_ctrl_auto_tune.kpi*80)/100;   //decrease all constants by 20% in case step-2 of autotuning is completed
                    pos_ctrl_auto_tune.active_step_counter=0;
                }

                if(pos_ctrl_auto_tune.err_energy_ss_int<pos_ctrl_auto_tune.err_energy_ss_int_min)
                    pos_ctrl_auto_tune.err_energy_ss_int_min = (pos_ctrl_auto_tune.err_energy_ss_int+(3.00*pos_ctrl_auto_tune.err_energy_ss_int_min))/4.00;

            }

            //step 3: increase kpl until vibration appears , and at the same time, keep overshoot less than 15% by reducing kpi. moreover update err_ss_min at all times
            if(pos_ctrl_auto_tune.active_step==AUTO_TUNE_STEP_3 && pos_ctrl_auto_tune.rising_edge==0)
            {
                if (pos_ctrl_auto_tune.overshoot_max>(15*pos_ctrl_auto_tune.step_amplitude)/100 && pos_ctrl_auto_tune.kpi_min<pos_ctrl_auto_tune.kpi)
                {
                    pos_ctrl_auto_tune.kpi = (pos_ctrl_auto_tune.kpi*98)/100;
                }
                else if(pos_ctrl_auto_tune.err_energy_ss_int < 5*pos_ctrl_auto_tune.err_energy_ss_int_min)
                {
                    int increment = (pos_ctrl_auto_tune.kpl*4)/100;
                    if(increment<10)
                        pos_ctrl_auto_tune.kpl += 10;
                    else
                        pos_ctrl_auto_tune.kpl += increment;
                    pos_ctrl_auto_tune.active_step_counter=0;
                }
                else
                {
                    pos_ctrl_auto_tune.active_step_counter++;
                }

                if(pos_ctrl_auto_tune.err_energy_ss_int<pos_ctrl_auto_tune.err_energy_ss_int_min)
                    pos_ctrl_auto_tune.err_energy_ss_int_min = (pos_ctrl_auto_tune.err_energy_ss_int+(3.00*pos_ctrl_auto_tune.err_energy_ss_int_min))/4;


                if(pos_ctrl_auto_tune.active_step_counter==10)
                {
                    pos_ctrl_auto_tune.kpl = (9*pos_ctrl_auto_tune.kpl)/10;
                    pos_ctrl_auto_tune.active_step=AUTO_TUNE_STEP_4;
                    pos_ctrl_auto_tune.kpi_min = (pos_ctrl_auto_tune.kpi*80)/100;
                    pos_ctrl_auto_tune.active_step_counter=0;
                }
            }

            //step 4: reduce kpi until overshoot is less than 2%, and at the same time, update err_energy_ss_int_min value
            if(pos_ctrl_auto_tune.active_step==AUTO_TUNE_STEP_4 && pos_ctrl_auto_tune.rising_edge==0)
            {
                if(pos_ctrl_auto_tune.overshoot_max<(2*pos_ctrl_auto_tune.step_amplitude)/100 || pos_ctrl_auto_tune.kpi_min>pos_ctrl_auto_tune.kpi)
                {
                    pos_ctrl_auto_tune.active_step_counter++;
                }
                else
                {
                    pos_ctrl_auto_tune.kpi = (pos_ctrl_auto_tune.kpi*98)/100;   //in case of unsuccessful autotuning, decrease all constants slowly until they are all 0
                    if(pos_ctrl_auto_tune.kpi==0) pos_ctrl_auto_tune.active_step=UNSUCCESSFUL;
                    pos_ctrl_auto_tune.active_step_counter=0;
                }

                pos_ctrl_auto_tune.err_energy_ss_int_min = (pos_ctrl_auto_tune.err_energy_ss_int+(3.00*pos_ctrl_auto_tune.err_energy_ss_int_min))/4;

                if(pos_ctrl_auto_tune.active_step_counter==10)
                {
                    pos_ctrl_auto_tune.active_step=AUTO_TUNE_STEP_5;
                    pos_ctrl_auto_tune.active_step_counter=0;
                }
            }

            //step 5: increase kpl until vibration appears
            if(pos_ctrl_auto_tune.active_step==AUTO_TUNE_STEP_5 && pos_ctrl_auto_tune.rising_edge==0)
            {
                if(pos_ctrl_auto_tune.err_energy_ss_int < 3*pos_ctrl_auto_tune.err_energy_ss_int_min)
                {
                    int increment = (pos_ctrl_auto_tune.kpl*3)/100;
                    if(increment<10)
                        pos_ctrl_auto_tune.kpl += 10;
                    else
                        pos_ctrl_auto_tune.kpl += increment;
                    pos_ctrl_auto_tune.active_step_counter=0;
                }
                else
                {
                    pos_ctrl_auto_tune.active_step_counter++;
                }

                if(pos_ctrl_auto_tune.err_energy_ss_int<pos_ctrl_auto_tune.err_energy_ss_int_min)
                    pos_ctrl_auto_tune.err_energy_ss_int_min = (pos_ctrl_auto_tune.err_energy_ss_int+(3.00*pos_ctrl_auto_tune.err_energy_ss_int_min))/4;

                if(pos_ctrl_auto_tune.active_step_counter==10)
                {
                    pos_ctrl_auto_tune.kpl = (8*pos_ctrl_auto_tune.kpl)/10;
                    pos_ctrl_auto_tune.err_energy_ss_int_min *= 10;
                    pos_ctrl_auto_tune.active_step=AUTO_TUNE_STEP_6;
                    pos_ctrl_auto_tune.active_step_counter=0;
                }
            }

            //correct moment of inertia
            if(pos_ctrl_auto_tune.active_step==AUTO_TUNE_STEP_6 && pos_ctrl_auto_tune.rising_edge==0)
            {
                if(pos_ctrl_auto_tune.err_energy_ss_int < 5*pos_ctrl_auto_tune.err_energy_ss_int_min)
                {
                    int increment = (pos_ctrl_auto_tune.j*4)/100;
                    if(increment<100)
                        pos_ctrl_auto_tune.j += 100;
                    else
                        pos_ctrl_auto_tune.j += increment;
                    pos_ctrl_auto_tune.active_step_counter=0;
                }
                else
                {
                    pos_ctrl_auto_tune.active_step_counter++;
                }

                if(pos_ctrl_auto_tune.err_energy_ss_int<pos_ctrl_auto_tune.err_energy_ss_int_min)
                    pos_ctrl_auto_tune.err_energy_ss_int_min = (pos_ctrl_auto_tune.err_energy_ss_int+(3*pos_ctrl_auto_tune.err_energy_ss_int_min))/4;

                if(pos_ctrl_auto_tune.active_step_counter==10)
                {
                    pos_ctrl_auto_tune.j = (pos_ctrl_auto_tune.j*6)/10;
                    pos_ctrl_auto_tune.kpl = (pos_ctrl_auto_tune.kpl*9)/10;

                    pos_ctrl_auto_tune.active_step=END;
                    pos_ctrl_auto_tune.auto_tune = 0;
                    pos_ctrl_auto_tune.activate=0;

                    pos_ctrl_auto_tune.kpp *= pos_ctrl_auto_tune.kpl;
                    pos_ctrl_auto_tune.kpi *= pos_ctrl_auto_tune.kpl;
                    pos_ctrl_auto_tune.kpd *= pos_ctrl_auto_tune.kpl;
                    pos_ctrl_auto_tune.kpl = 1000;

                    pos_ctrl_auto_tune.kpp /= 1000;
                    pos_ctrl_auto_tune.kpi /= 1000;
                    pos_ctrl_auto_tune.kpd /= 1000;

                    pos_ctrl_auto_tune.active_step_counter=0;
                }

                pos_ctrl_auto_tune.kvl = pos_ctrl_auto_tune.j ;

            }

            if(pos_ctrl_auto_tune.active_step==UNSUCCESSFUL)
            {
                pos_ctrl_auto_tune.kpl =(80*pos_ctrl_auto_tune.kpl)/100;

                if(pos_ctrl_auto_tune.kpl==0)
                {
                    pos_ctrl_auto_tune.auto_tune = 0;
                    pos_ctrl_auto_tune.activate=0;

                    pos_ctrl_auto_tune.kpp =0;
                    pos_ctrl_auto_tune.kpi =0;
                    pos_ctrl_auto_tune.kpd =0;
                    pos_ctrl_auto_tune.j   =0;
                    pos_ctrl_auto_tune.kpl =0;
                    pos_ctrl_auto_tune.active_step_counter=0;
                }
            }

            //show the tuning step with kvp (which is not important in the case of limited torque position controller
            pos_ctrl_auto_tune.kvp = pos_ctrl_auto_tune.active_step ;

            pos_ctrl_auto_tune.overshoot=0;
            pos_ctrl_auto_tune.overshoot_max=0;

            pos_ctrl_auto_tune.err=0.00;
            pos_ctrl_auto_tune.err_energy =0.00;
            pos_ctrl_auto_tune.err_energy_int=0.00;

            pos_ctrl_auto_tune.err_ss=0.00;
            pos_ctrl_auto_tune.err_energy_ss =0.00;
            pos_ctrl_auto_tune.err_energy_ss_int = 0.00;

            // vibration index computation

            pos_ctrl_auto_tune.velocity_k_1=0.00;
            pos_ctrl_auto_tune.velocity_k  =0.00;

            pos_ctrl_auto_tune.velocity_k_filtered_k_1 =0.00;
            pos_ctrl_auto_tune.velocity_k_filtered     =0.00;

            pos_ctrl_auto_tune.acceleration_k_1 =0.00;
            pos_ctrl_auto_tune.acceleration_k   =0.00;

            pos_ctrl_auto_tune.acceleration_k_filtered_k_1 =0.00;
            pos_ctrl_auto_tune.acceleration_k_filtered     =0.00;

            pos_ctrl_auto_tune.jerk_k_1                    =0.00;
            pos_ctrl_auto_tune.jerk_k                      =0.00;
            pos_ctrl_auto_tune.jerk_k_transition           =0;

            pos_ctrl_auto_tune.jerk_k_filtered             =0.00;
            pos_ctrl_auto_tune.jerk_filtered_energy        =0.00;


            pos_ctrl_auto_tune.counter=0;
        }
    }

    /*
     * measurement of error energy
     */
    pos_ctrl_auto_tune.err = (pos_ctrl_auto_tune.position_ref - position_k)/1000.00;
    pos_ctrl_auto_tune.err_energy = pos_ctrl_auto_tune.err * pos_ctrl_auto_tune.err;
    pos_ctrl_auto_tune.err_energy_int += pos_ctrl_auto_tune.err_energy;

    /*
     * measurement of overshoot and rise time
     */
    pos_ctrl_auto_tune.overshoot = position_k - pos_ctrl_auto_tune.position_ref;

    if(pos_ctrl_auto_tune.overshoot > pos_ctrl_auto_tune.overshoot_max)
        pos_ctrl_auto_tune.overshoot_max=pos_ctrl_auto_tune.overshoot;

    /*
     * measurement of error energy after steady state
     */
    if((90*pos_ctrl_auto_tune.counter_max)/100<pos_ctrl_auto_tune.counter && pos_ctrl_auto_tune.counter<(98*pos_ctrl_auto_tune.counter_max)/100)
    {
        pos_ctrl_auto_tune.err_ss = (pos_ctrl_auto_tune.position_ref - position_k);
        pos_ctrl_auto_tune.err_energy_ss = pos_ctrl_auto_tune.err_ss * pos_ctrl_auto_tune.err_ss;
        pos_ctrl_auto_tune.err_energy_ss_int += pos_ctrl_auto_tune.err_energy_ss;
    }

    pos_ctrl_auto_tune.position_act_k_2 = pos_ctrl_auto_tune.position_act_k_1;
    pos_ctrl_auto_tune.position_act_k_1 = pos_ctrl_auto_tune.position_act_k;
    pos_ctrl_auto_tune.position_act_k   = position_k;

    pos_ctrl_auto_tune.velocity_k_1= pos_ctrl_auto_tune.velocity_k;
    pos_ctrl_auto_tune.velocity_k  = pos_ctrl_auto_tune.position_act_k - pos_ctrl_auto_tune.position_act_k_1;

    pos_ctrl_auto_tune.velocity_k_filtered_k_1 = pos_ctrl_auto_tune.velocity_k_filtered;
    pos_ctrl_auto_tune.velocity_k_filtered = (9*pos_ctrl_auto_tune.velocity_k_filtered + pos_ctrl_auto_tune.velocity_k)/10;

    pos_ctrl_auto_tune.acceleration_k_1= pos_ctrl_auto_tune.acceleration_k;
    pos_ctrl_auto_tune.acceleration_k= pos_ctrl_auto_tune.velocity_k_filtered-pos_ctrl_auto_tune.velocity_k_filtered_k_1;

    pos_ctrl_auto_tune.acceleration_k_filtered_k_1 = pos_ctrl_auto_tune.acceleration_k_filtered;
    pos_ctrl_auto_tune.acceleration_k_filtered= (19*pos_ctrl_auto_tune.acceleration_k_filtered+pos_ctrl_auto_tune.acceleration_k)/20;

    pos_ctrl_auto_tune.jerk_k_1= pos_ctrl_auto_tune.jerk_k;
    pos_ctrl_auto_tune.jerk_k= pos_ctrl_auto_tune.acceleration_k_filtered-pos_ctrl_auto_tune.acceleration_k_filtered_k_1;

    pos_ctrl_auto_tune.jerk_k_filtered= (49*pos_ctrl_auto_tune.jerk_k_filtered+pos_ctrl_auto_tune.jerk_k)/50;


    if(pos_ctrl_auto_tune.rising_edge==1 &&
            pos_ctrl_auto_tune.position_init-pos_ctrl_auto_tune.step_amplitude+(85*2*pos_ctrl_auto_tune.step_amplitude)/100<position_k &&
            position_k<pos_ctrl_auto_tune.position_init-pos_ctrl_auto_tune.step_amplitude+(90*2*pos_ctrl_auto_tune.step_amplitude)/100)
    {
        pos_ctrl_auto_tune.jerk_counter_limit = (4*pos_ctrl_auto_tune.jerk_counter_limit+pos_ctrl_auto_tune.counter)/5;
    }

    /*
     * jerk measurement
     * from the moment jerk becomes negative, measure the energy of it over positive states
     */
    if(pos_ctrl_auto_tune.rising_edge==1 && pos_ctrl_auto_tune.counter<(110*pos_ctrl_auto_tune.jerk_counter_limit)/100)
    {
        if(pos_ctrl_auto_tune.jerk_k_filtered<0) pos_ctrl_auto_tune.jerk_k_transition=1;

        if(pos_ctrl_auto_tune.jerk_k_transition==1 && 0<pos_ctrl_auto_tune.jerk_k_filtered)
            pos_ctrl_auto_tune.jerk_filtered_energy += (pos_ctrl_auto_tune.jerk_k_filtered*pos_ctrl_auto_tune.jerk_k_filtered);
    }


    if(pos_ctrl_auto_tune.counter==0)
    {
        motion_ctrl_config.velocity_kp = pos_ctrl_auto_tune.kvp;
        motion_ctrl_config.velocity_ki = pos_ctrl_auto_tune.kvi;
        motion_ctrl_config.velocity_kd = pos_ctrl_auto_tune.kvd;
        motion_ctrl_config.velocity_integral_limit = pos_ctrl_auto_tune.kvl;

        motion_ctrl_config.position_kp = pos_ctrl_auto_tune.kpp;
        motion_ctrl_config.position_ki = pos_ctrl_auto_tune.kpi;
        motion_ctrl_config.position_kd = pos_ctrl_auto_tune.kpd;
        motion_ctrl_config.position_integral_limit = pos_ctrl_auto_tune.kpl;

        motion_ctrl_config.moment_of_inertia       = pos_ctrl_auto_tune.j;

        motion_ctrl_config.position_control_autotune = pos_ctrl_auto_tune.auto_tune;

        if(pos_ctrl_auto_tune.controller == LIMITED_TORQUE)//show j in the place of kvp while tuning lt-pos-ctrl in app_master_tuning_ethercat
            motion_ctrl_config.velocity_kp = motion_ctrl_config.moment_of_inertia;


    }

    return 0;
}
