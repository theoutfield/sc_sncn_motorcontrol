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
