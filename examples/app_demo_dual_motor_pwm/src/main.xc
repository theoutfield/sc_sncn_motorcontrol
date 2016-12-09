/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC1K-rev-c3.bsp>


/**
 * @brief Test illustrates usage of module_commutation
 * @date 17/06/2014
 */

//#include <pwm_service.h>
#include <pwm_server.h>
#include <adc_service.h>
#include <user_config.h>
#include <motor_control_interfaces.h>
#include <advanced_motor_control.h>
#include <advanced_motorcontrol_licence.h>
#include <position_feedback_service.h>

//PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
PwmPortsGeneral pwm_ports = SOMANET_IFM_PWM_PORTS_GENERAL;

//WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;

//FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;

//ADCPorts adc_ports = SOMANET_IFM_ADC_PORTS;
//HallPorts hall_ports = SOMANET_IFM_HALL_PORTS;
//SPIPorts spi_ports = SOMANET_IFM_AMS_PORTS;
//QEIPorts qei_ports = SOMANET_IFM_QEI_PORTS;



void send_pwm_values(
        client interface update_pwm_general i_update_pwm,
        interface WatchdogInterface client i_watchdog,
        int ref_clk_frq,
        int pwm_clk_frq,
        int commutation_frq)
{
    timer t;
    unsigned time=0, ts=0;
    int safe_torque_off_mode=0;
    int pwm_on=1;

    int pwm_max=0, pwm_min=0, pwm_dif=0, pwm_av=0;

    int pwm_counter=0;
    int pwm_value=2000;
    int pwm_values[6];

    unsigned int sync_inc=0;


    //proper task startup
    t :> ts;
    t when timerafter (ts + (5000*20*250)) :> void;

//    while(i_shared_memory.status()!=ACTIVE);
    while(i_watchdog.status()!=ACTIVE);
//    while(i_adc.status()!=ACTIVE);
    while(i_update_pwm.status()!=ACTIVE);



     pwm_dif =  pwm_max -  pwm_min;
     pwm_av  =( pwm_max +  pwm_min)/2;

     pwm_values[0] =  pwm_av;
     pwm_values[1] =  pwm_av;
     pwm_values[2] =  pwm_av;

     pwm_on  =0;



    if(ref_clk_frq==100)
    {
        if(pwm_clk_frq==100)
        {
            if(commutation_frq==12)
            {
                sync_inc= 8192;
                pwm_max = 7000;
                pwm_min = 600;
            }
            else if (commutation_frq==24)
            {
                sync_inc= 8192;
                pwm_max = 3200;
                pwm_min =  600;
            }
            else
            {
                printstr("ERROR: PWM SETTINGS NOT SUPPORTED \n");
                while(1);
            }
        }
        else
        {
            printstr("ERROR: PWM SETTINGS NOT SUPPORTED \n");
            while(1);
        }
    }
    else if(ref_clk_frq==250)
    {
        if(pwm_clk_frq==250)
        {
            if(commutation_frq==15)
            {
                sync_inc= 16384;
                pwm_max = 13000;
                pwm_min = 1500;
            }
            else
            {
                printstr("ERROR: PWM SETTINGS NOT SUPPORTED \n");
                while(1);
            }
        }
        else
        {
            printstr("ERROR: PWM SETTINGS NOT SUPPORTED \n");
            while(1);
        }
    }
    else
    {
        printstr("ERROR: PWM SETTINGS NOT SUPPORTED \n");
        while(1);
    }
    t :> time;
    while (1)
    {
        select
        {
        case t when timerafter(time) :> void:

            pwm_counter++;
            if(pwm_counter>10)
            {
                pwm_counter=0;

                pwm_value++;
                if(pwm_value>pwm_max) pwm_value= pwm_min;

                if(pwm_value<pwm_min) pwm_value= pwm_min;
                if(pwm_value>pwm_max) pwm_value= pwm_max;

                pwm_values[0]=pwm_value;
                pwm_values[1]=pwm_value;
                pwm_values[2]=pwm_value;

                pwm_values[3]=pwm_value;
                pwm_values[4]=pwm_value;
                pwm_values[5]=pwm_value;

                i_update_pwm.update_server_control_data(
                        pwm_values[0],  pwm_values[1],  pwm_values[2],
                        pwm_values[3],  pwm_values[4],  pwm_values[5],
                        pwm_on,  safe_torque_off_mode);
            }

            time +=sync_inc;
            break;
        }
    }
}


int main(void) {

    // Motor control interfaces
    interface WatchdogInterface i_watchdog[2];
    interface update_pwm_general i_update_pwm;
    interface ADCInterface i_adc[2];
    interface MotorcontrolInterface i_motorcontrol[2];
    //interface PositionVelocityCtrlInterface i_position_control[3];
    //interface PositionFeedbackInterface i_position_feedback[3];
    interface shared_memory_interface i_shared_memory[2];
    //interface PositionLimiterInterface i_position_limiter;

    par
    {
        /* WARNING: only one blocking task is possible per tile. */
        /* Waiting for a user input blocks other tasks on the same tile from execution. */
        //        on tile[APP_TILE]: demo_torque_position_velocity_control(i_position_control[0]);

        //        on tile[APP_TILE_2]:
        //        /* Position Control Loop */
        //        {
        //            PosVelocityControlConfig pos_velocity_ctrl_config;
        //            /* Control Loop */
        //            pos_velocity_ctrl_config.control_loop_period =                  CONTROL_LOOP_PERIOD; //us
        //
        //            pos_velocity_ctrl_config.min_pos =                              MIN_POSITION_LIMIT;
        //            pos_velocity_ctrl_config.max_pos =                              MAX_POSITION_LIMIT;
        //            pos_velocity_ctrl_config.pos_limit_threshold =                  POSITION_LIMIT_THRESHOLD;
        //            pos_velocity_ctrl_config.max_speed =                            MAX_SPEED;
        //            pos_velocity_ctrl_config.max_torque =                           TORQUE_CONTROL_LIMIT;
        //            pos_velocity_ctrl_config.polarity =                             POLARITY;
        //
        //            pos_velocity_ctrl_config.enable_profiler =                      ENABLE_PROFILER;
        //            pos_velocity_ctrl_config.max_acceleration_profiler =            MAX_ACCELERATION_PROFILER;
        //            pos_velocity_ctrl_config.max_speed_profiler =                   MAX_SPEED_PROFILER;
        //
        //            pos_velocity_ctrl_config.control_mode =                         NL_POSITION_CONTROLLER;
        //
        //            pos_velocity_ctrl_config.P_pos =                                POSITION_Kp;
        //            pos_velocity_ctrl_config.I_pos =                                POSITION_Ki;
        //            pos_velocity_ctrl_config.D_pos =                                POSITION_Kd;
        //            pos_velocity_ctrl_config.integral_limit_pos =                   POSITION_INTEGRAL_LIMIT;
        //            pos_velocity_ctrl_config.j =                                    MOMENT_OF_INERTIA;
        //
        //            pos_velocity_ctrl_config.P_velocity =                           VELOCITY_Kp;
        //            pos_velocity_ctrl_config.I_velocity =                           VELOCITY_Ki;
        //            pos_velocity_ctrl_config.D_velocity =                           VELOCITY_Kd;
        //            pos_velocity_ctrl_config.integral_limit_velocity =              VELOCITY_INTEGRAL_LIMIT;
        //
        //            pos_velocity_ctrl_config.position_fc =                          POSITION_FC;
        //            pos_velocity_ctrl_config.velocity_fc =                          VELOCITY_FC;
        //            pos_velocity_ctrl_config.resolution  =                          POSITION_SENSOR_RESOLUTION;
        //            pos_velocity_ctrl_config.special_brake_release =                ENABLE_SHAKE_BRAKE;
        //            pos_velocity_ctrl_config.brake_shutdown_delay =                 BRAKE_SHUTDOWN_DELAY;
        //
        //
        //            position_velocity_control_service(pos_velocity_ctrl_config, i_motorcontrol[0], i_position_control);
        //        }


        on tile[IFM_TILE]:
        {
            par
            {
                /* PWM Service */
                {
                    pwm_config_general(pwm_ports, REF_CLK_FRQ_MHZ_, PWM_CLK_FRQ_MHZ_);

                    if (!isnull(fet_driver_ports.p_esf_rst_pwml_pwmh) && !isnull(fet_driver_ports.p_coast))
                        predriver(fet_driver_ports);

                    pwm_service_general(
                            pwm_ports, i_update_pwm,
                            REF_CLK_FRQ_MHZ_, PWM_CLK_FRQ_MHZ_, COMMUTATION_FRQ_KHZ_);

                }

                /* ADC Service */
                //                {
                //                    adc_service(adc_ports, null/*c_trigger*/, i_adc /*ADCInterface*/, i_watchdog[1], IFM_TILE_USEC);
                //                }

                /* Watchdog Service */
                {
                    watchdog_service(wd_ports, i_watchdog, REF_CLK_FRQ_MHZ_);//20ms
                }

                /* Motor Control Service */
                {
                    MotorcontrolConfig motorcontrol_config;

                    motorcontrol_config.licence =  ADVANCED_MOTOR_CONTROL_LICENCE;
                    motorcontrol_config.v_dc =  VDC;
                    motorcontrol_config.commutation_loop_period =  COMMUTATION_LOOP_PERIOD;
                    motorcontrol_config.polarity_type=MOTOR_POLARITY;
                    motorcontrol_config.current_P_gain =  TORQUE_Kp;
                    motorcontrol_config.current_I_gain =  TORQUE_Ki;
                    motorcontrol_config.current_D_gain =  TORQUE_Kd;
                    motorcontrol_config.pole_pair =  POLE_PAIRS;
                    motorcontrol_config.commutation_sensor=MOTOR_COMMUTATION_SENSOR;
                    motorcontrol_config.commutation_angle_offset=COMMUTATION_OFFSET_CLK;
                    motorcontrol_config.hall_state_1_angle=HALL_STATE_1_ANGLE;
                    motorcontrol_config.hall_state_2_angle=HALL_STATE_2_ANGLE;
                    motorcontrol_config.hall_state_3_angle=HALL_STATE_3_ANGLE;
                    motorcontrol_config.hall_state_4_angle=HALL_STATE_4_ANGLE;
                    motorcontrol_config.hall_state_5_angle=HALL_STATE_5_ANGLE;
                    motorcontrol_config.hall_state_6_angle=HALL_STATE_6_ANGLE;
                    motorcontrol_config.max_torque =  MAXIMUM_TORQUE;
                    motorcontrol_config.phase_resistance =  PHASE_RESISTANCE;
                    motorcontrol_config.phase_inductance =  PHASE_INDUCTANCE;
                    motorcontrol_config.torque_constant =  PERCENT_TORQUE_CONSTANT;
                    motorcontrol_config.current_ratio =  CURRENT_RATIO;
                    motorcontrol_config.rated_current =  RATED_CURRENT;
                    motorcontrol_config.rated_torque  =  RATED_TORQUE;
                    motorcontrol_config.percent_offset_torque =  PERCENT_OFFSET_TORQUE;
                    motorcontrol_config.recuperation = RECUPERATION;
                    motorcontrol_config.battery_e_max = BATTERY_E_MAX;
                    motorcontrol_config.battery_e_min = BATTERY_E_MIN;
                    motorcontrol_config.regen_p_max = REGEN_P_MAX;
                    motorcontrol_config.regen_p_min = REGEN_P_MIN;
                    motorcontrol_config.regen_speed_max = REGEN_SPEED_MAX;
                    motorcontrol_config.regen_speed_min = REGEN_SPEED_MIN;
                    motorcontrol_config.protection_limit_over_current =  I_MAX;
                    motorcontrol_config.protection_limit_over_voltage =  V_DC_MAX;
                    motorcontrol_config.protection_limit_under_voltage = V_DC_MIN;

                    //100ms
                    motor_control_service(motorcontrol_config, i_adc[0], i_shared_memory[1],
                            i_watchdog[0], i_motorcontrol, i_update_pwm,
                            REF_CLK_FRQ_MHZ_, PWM_CLK_FRQ_MHZ_, COMMUTATION_FRQ_KHZ_);
                    send_pwm_values(
                            i_update_pwm,
                            i_watchdog[0],
                             REF_CLK_FRQ_MHZ_,
                             PWM_CLK_FRQ_MHZ_,
                             COMMUTATION_FRQ_KHZ_);

                }

                //                /* Shared memory Service */
                //                [[distribute]] memory_manager(i_shared_memory, 2);

                //                /* Position feedback service */
                //                {
                //                    PositionFeedbackConfig position_feedback_config;
                //                    position_feedback_config.sensor_type = MOTOR_COMMUTATION_SENSOR;
                //                    position_feedback_config.polarity    = SENSOR_POLARITY;
                //                    position_feedback_config.pole_pairs  = POLE_PAIRS;
                //                    position_feedback_config.resolution  = POSITION_SENSOR_RESOLUTION;
                //                    position_feedback_config.offset      = 0;
                //                    position_feedback_config.enable_push_service = PushAll;
                //
                //                    position_feedback_config.biss_config.multiturn_length = BISS_MULTITURN_LENGTH;
                //                    position_feedback_config.biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
                //                    position_feedback_config.biss_config.singleturn_length = BISS_SINGLETURN_LENGTH;
                //                    position_feedback_config.biss_config.status_length = BISS_STATUS_LENGTH;
                //                    position_feedback_config.biss_config.crc_poly = BISS_CRC_POLY;
                //                    position_feedback_config.biss_config.clock_dividend = BISS_CLOCK_DIVIDEND;
                //                    position_feedback_config.biss_config.clock_divisor = BISS_CLOCK_DIVISOR;
                //                    position_feedback_config.biss_config.timeout = BISS_TIMEOUT;
                //                    position_feedback_config.biss_config.max_ticks = BISS_MAX_TICKS;
                //                    position_feedback_config.biss_config.velocity_loop = BISS_VELOCITY_LOOP;
                //
                //                    position_feedback_config.contelec_config.filter = CONTELEC_FILTER;
                //                    position_feedback_config.contelec_config.timeout = CONTELEC_TIMEOUT;
                //                    position_feedback_config.contelec_config.velocity_loop = CONTELEC_VELOCITY_LOOP;
                //
                //                    position_feedback_config.qei_config.index_type = QEI_SENSOR_INDEX_TYPE;
                //                    position_feedback_config.qei_config.signal_type = QEI_SENSOR_SIGNAL_TYPE;
                //
                //                    position_feedback_config.ams_config.factory_settings = 1;
                //                    position_feedback_config.ams_config.hysteresis = 1;
                //                    position_feedback_config.ams_config.noise_setting = AMS_NOISE_NORMAL;
                //                    position_feedback_config.ams_config.uvw_abi = 0;
                //                    position_feedback_config.ams_config.dyn_angle_comp = 0;
                //                    position_feedback_config.ams_config.data_select = 0;
                //                    position_feedback_config.ams_config.pwm_on = AMS_PWM_OFF;
                //                    position_feedback_config.ams_config.abi_resolution = 0;
                //                    position_feedback_config.ams_config.max_ticks = 0x7fffffff;
                //                    position_feedback_config.ams_config.cache_time = AMS_CACHE_TIME;
                //                    position_feedback_config.ams_config.velocity_loop = AMS_VELOCITY_LOOP;
                //
                //                    position_feedback_service(hall_ports, qei_ports, spi_ports,
                //                            position_feedback_config, i_shared_memory[0], i_position_feedback,
                //                            null, null, null);
                //                }
            }
        }
    }

    return 0;
}
