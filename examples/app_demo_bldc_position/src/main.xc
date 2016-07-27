/* INCLUDE BOARD SUPPORT FILES FROM module_board-support */
//#include <CORE_BOARD_REQUIRED>
//#include <IFM_BOARD_REQUIRED>
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC100-rev-b.bsp>

/**
 * @file test_position-ctrl.xc
 * @brief Test illustrates usage of profile position control
 * @author Synapticon GmbH (www.synapticon.com)
 */

//BLDC Motor drive libs
#include <qei_service.h>
#include <hall_service.h>
#include <ams_service.h>
#include <biss_service.h>
#include <pwm_service.h>
#include <watchdog_service.h>
#include <motorcontrol_service.h>

//Position control + profile libs
#include <position_ctrl_service.h>
#include <profile_control.h>

//Configuration headers
#include <user_config.h>

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
ADCPorts adc_ports = SOMANET_IFM_ADC_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
SPIPorts spi_ports = SOMANET_IFM_AMS_PORTS;

/* Test Profile Position function */
void position_profile_test(interface PositionControlInterface client i_position_control,
                           interface HallInterface client ?i_hall,
                           interface QEIInterface client ?i_qei,
                           interface BISSInterface client ?i_biss,
                           interface AMSInterface client ?i_ams,
                           interface CONTELECInterface client ?i_contelec)
{
    const int target = 1000000;
//    const int target = 2620000;
    int target_position = target;        // HALL: 1 rotation = 4096 x nr. pole pairs; QEI: your encoder documented resolution x 4 = one rotation
    int velocity        = 500;         // rpm
    int acceleration    = 100;         // rpm/s
    int deceleration    = 100;         // rpm/s
    int follow_error = 0;
    int actual_position = 0;

    ProfilerConfig profiler_config;
    profiler_config.polarity = POLARITY;
    profiler_config.max_position = MAX_POSITION_LIMIT;
    profiler_config.min_position = MIN_POSITION_LIMIT;

    profiler_config.max_velocity = MAX_VELOCITY;
    profiler_config.max_acceleration = MAX_ACCELERATION;
    profiler_config.max_deceleration = MAX_DECELERATION;

    /* Initialise the position profile generator */
    init_position_profiler(profiler_config, i_position_control, i_hall, i_qei, i_biss, i_ams, i_contelec);

    delay_milliseconds(500);//let the servers start before sending clien requests

    /* Set new target position for profile position control */
    set_profile_position(target_position, velocity, acceleration, deceleration, i_position_control);

    while(1)
    {
        // Read actual position from the Position Control Server
        actual_position = i_position_control.get_position();
        follow_error = target_position - actual_position;

        /*
        xscope_core_int(0, actual_position);
        xscope_core_int(1, target_position);
        xscope_core_int(2, follow_error);
        */
        // Keep motor turning when reaching target position
        if ((target_position == target) && (follow_error < 200)) {

            target_position = 0;
            set_profile_position(target_position, velocity, acceleration, deceleration, i_position_control);

        } else if ((target_position == 0) && (follow_error < 200)) {

            target_position = target;
            set_profile_position(target_position, velocity, acceleration, deceleration, i_position_control);
        }
        delay_milliseconds(1);
    }
}


int main(void)
{
    // Motor control channels
    chan c_pwm_ctrl, c_adctrig;            // pwm channel

    interface WatchdogInterface i_watchdog[2];
    interface ADCInterface i_adc[2];
    interface BrakeInterface i_brake;
    interface MotorcontrolInterface i_motorcontrol[4];
    interface CONTELECInterface i_contelec[5];

    interface shared_memory_interface i_shared_memory[2];
    interface PositionControlInterface i_position_control[3];

    par
    {
        /* Test Profile Position Client function*/
        on tile[APP_TILE]:
        {
           position_profile_test(i_position_control[0], null, null, null, null, i_contelec[2]);      // test PPM on slave side
        }

        on tile[APP_TILE]:
        /* XScope monitoring */
        {
            int actual_position, target_position;

            while(1)
            {
                /* Read actual position from the Position Control Server */
                actual_position = i_position_control[1].get_position();
                target_position = i_position_control[1].get_target_position();

                //xscope_int(REAL_TARGET, 1000);
                xscope_int(TARGET_POSITION, target_position/10); //Divided by 10 for better displaying
                xscope_int(ACTUAL_POSITION, actual_position/10); //Divided by 10 for better displaying
                xscope_int(FOLLOW_ERROR, (target_position-actual_position)/10); //Divided by 10 for better displaying

                delay_milliseconds(1); /* 1 ms wait */
            }
        }

        on tile[APP_TILE]:
        /* Position Control Loop */
        {
            ControlConfig position_control_config;

            position_control_config.feedback_sensor = MOTOR_FEEDBACK_SENSOR;

            position_control_config.Kp_n = POSITION_Kp;    // Divided by 10000
            position_control_config.Ki_n = POSITION_Ki;    // Divided by 10000
            position_control_config.Kd_n = POSITION_Kd;    // Divided by 10000

            position_control_config.control_loop_period = CONTROL_LOOP_PERIOD; //us
            position_control_config.cascade_with_torque = 0;

            /* Control Loop */

            position_control_service(position_control_config, null, null, null, null, i_contelec[1], i_motorcontrol[0],
                                     i_position_control);
        }

        /************************************************************
         * IFM_TILE
         ************************************************************/
        on tile[IFM_TILE]:
        {
            par
            {
                /* Triggered PWM Service */
                pwm_triggered_service( pwm_ports, c_adctrig, c_pwm_ctrl, null);

                /* Watchdog Service */
                watchdog_service(wd_ports, i_watchdog);

                /* ADC Service */
                adc_service(adc_ports, c_adctrig, i_adc, i_watchdog[1]);

                {
                    CONTELECConfig contelec_config;
                    contelec_config.resolution_bits = CONTELEC_RESOLUTION;
                    contelec_config.polarity = CONTELEC_POLARITY_INVERTED;
                    contelec_config.offset = 33120;//24542;//24267;//19236;
                    contelec_config.timeout = CONTELEC_TIMEOUT;
                    contelec_config.velocity_loop = CONTELEC_VELOCITY_LOOP;
                    contelec_config.max_ticks = 0x7fffffff;
                    contelec_config.pole_pairs = POLE_PAIRS;
                    contelec_config.filter = CONTELEC_FILTER;
                    contelec_config.enable_push_service = PushAll;

                    contelec_service(spi_ports, contelec_config, i_shared_memory[1], i_contelec);
                }

                {
                    memory_manager(i_shared_memory, 2);
                }

                /* Motor Control Service */
                {
                    MotorcontrolConfig motorcontrol_config;
                    motorcontrol_config.motor_type = BLDC_MOTOR;
                    motorcontrol_config.commutation_method = FOC;
                    motorcontrol_config.polarity_type = MOTOR_POLARITY; //INVERTED_POLARITY;//
                    motorcontrol_config.commutation_sensor = MOTOR_COMMUTATION_SENSOR;
                    motorcontrol_config.bldc_winding_type = BLDC_WINDING_TYPE;
                    motorcontrol_config.hall_offset[0] = 3700;//COMMUTATION_OFFSET_CLK;
                    motorcontrol_config.hall_offset[1] = 3700;//COMMUTATION_OFFSET_CCLK;
                    motorcontrol_config.commutation_loop_period =  COMMUTATION_LOOP_PERIOD;

                    motorcontrol_service(fet_driver_ports, motorcontrol_config,
                                         c_pwm_ctrl, i_adc[0], null, null, null, null, null, i_shared_memory[0], i_watchdog[0], null, i_motorcontrol);
                }
            }
        }
    }

    return 0;
}
