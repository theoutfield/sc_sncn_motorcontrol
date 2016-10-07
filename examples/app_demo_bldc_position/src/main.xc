/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>

/**
 * @file test_position-ctrl.xc
 * @brief Test illustrates usage of profile position control
 * @author Synapticon GmbH (www.synapticon.com)
 */

#include <pwm_server.h>
#include <adc_service.h>
#include <user_config.h>
#include <motor_control_interfaces.h>
#include <advanced_motor_control.h>
#include <advanced_motorcontrol_licence.h>
#include <position_feedback_service.h>

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
ADCPorts adc_ports = SOMANET_IFM_ADC_PORTS;
HallPorts hall_ports = SOMANET_IFM_HALL_PORTS;
SPIPorts spi_ports = SOMANET_IFM_AMS_PORTS;
QEIPorts qei_ports = SOMANET_IFM_QEI_PORTS;



//Position control + profile libs
#include <position_ctrl_service.h>
#include <profile_control.h>



/* Test Profile Position function */
void position_profile_test(interface PositionVelocityCtrlInterface client i_position_control, client interface PositionFeedbackInterface ?i_position_feedback)
{
    const int target = 16000;
//    const int target = 2620000;
    int target_position = target;        // HALL: 1 rotation = 4096 x nr. pole pairs; QEI: your encoder documented resolution x 4 = one rotation
    int velocity        = 500;         // rpm
    int acceleration    = 500;         // rpm/s
    int deceleration    = 500;         // rpm/s
    int follow_error = 0;
    int actual_position = 0;

    ProfilerConfig profiler_config;
    profiler_config.polarity = POLARITY;
    profiler_config.max_position = MAX_POSITION_LIMIT;
    profiler_config.min_position = MIN_POSITION_LIMIT;
    if (!isnull(i_position_feedback)) {
        profiler_config.ticks_per_turn = i_position_feedback.get_ticks_per_turn();
    } else {
        profiler_config.ticks_per_turn = QEI_SENSOR_RESOLUTION;
    }
    profiler_config.max_velocity = MAX_VELOCITY;
    profiler_config.max_acceleration = MAX_ACCELERATION;
    profiler_config.max_deceleration = MAX_DECELERATION;

    DownstreamControlData downstream_control_data;
    downstream_control_data.velocity_cmd = 0;
    downstream_control_data.torque_cmd = 0;
    downstream_control_data.offset_torque = 0;
    downstream_control_data.position_cmd = 0;

    int start_position = i_position_control.get_position();
    target_position = start_position + target;

    /* Initialise the position profile generator */
    init_position_profiler(profiler_config);

    delay_milliseconds(500);//let the servers start before sending client requests

    downstream_control_data.position_cmd = target_position;
    /* Set new target position for profile position control */
    set_profile_position(downstream_control_data, velocity, acceleration, deceleration, i_position_control);

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
        if (follow_error < 200 && follow_error > -200){
            if (target_position == (start_position + target)){
                target_position = start_position - target;
            } else {
                target_position = start_position + target;
            }
            downstream_control_data.position_cmd = target_position;
            set_profile_position(downstream_control_data, velocity, acceleration, deceleration, i_position_control);
        }
        delay_milliseconds(1);
    }
}


int main(void)
{
    // Motor control channels

    interface WatchdogInterface i_watchdog[2];
    interface ADCInterface i_adc[2];
    interface MotorcontrolInterface i_motorcontrol[4];
    interface update_pwm i_update_pwm;
    interface shared_memory_interface i_shared_memory[2];
    interface PositionVelocityCtrlInterface i_position_control[3];
    interface PositionFeedbackInterface i_position_feedback[3];

    par
    {
        /* Test Profile Position Client function*/
        on tile[APP_TILE]:
        {
            position_profile_test(i_position_control[0], i_position_feedback[0]);      // test PPM on slave side
        }

        on tile[APP_TILE]:
        /* XScope monitoring */
        {
            int actual_position, target_position, actual_velocity;

            while(1)
            {
                /* Read actual position from the Position Control Server */
                actual_velocity = i_position_control[1].get_velocity();
                actual_position = i_position_control[1].get_position();
                //target_position = i_position_control[1].get_target_position();

                //xscope_int(TARGET_POSITION, target_position); //Divided by 10 for better displaying
                xscope_int(ACTUAL_POSITION, actual_position); //Divided by 10 for better displaying
                xscope_int(VELOCITY, actual_velocity);
                //xscope_int(FOLLOW_ERROR, (target_position-actual_position)); //Divided by 10 for better displaying

                delay_milliseconds(1); /* 1 ms wait */
            }
        }

        on tile[APP_TILE]:
        /* Position Control Loop */
        {
            PosVelocityControlConfig pos_velocity_ctrl_config;
            /* Control Loop */
            pos_velocity_ctrl_config.control_loop_period =                  CONTROL_LOOP_PERIOD; //us

            pos_velocity_ctrl_config.min_pos =                              MIN_POSITION_LIMIT;
            pos_velocity_ctrl_config.max_pos =                              MAX_POSITION_LIMIT;
            pos_velocity_ctrl_config.max_speed =                            MAX_VELOCITY;
            pos_velocity_ctrl_config.max_torque =                           MAX_TORQUE;

            pos_velocity_ctrl_config.enable_profiler =                      ENABLE_PROFILER;
            pos_velocity_ctrl_config.max_acceleration_profiler =            MAX_ACCELERATION_PROFILER;
            pos_velocity_ctrl_config.max_speed_profiler =                   MAX_SPEED_PROFILER;

            pos_velocity_ctrl_config.control_mode =                         NL_POSITION_CONTROLLER;

            pos_velocity_ctrl_config.P_pos =                                Kp_POS_PID;
            pos_velocity_ctrl_config.I_pos =                                Ki_POS_PID;
            pos_velocity_ctrl_config.D_pos =                                Kd_POS_PID;
            pos_velocity_ctrl_config.integral_limit_pos =                   INTEGRAL_LIMIT_POS_PID;

            pos_velocity_ctrl_config.P_velocity =                           Kp_VELOCITY_PID;
            pos_velocity_ctrl_config.I_velocity =                           Ki_VELOCITY_PID;
            pos_velocity_ctrl_config.D_velocity =                           Kd_VELOCITY_PID;
            pos_velocity_ctrl_config.integral_limit_velocity =              INTEGRAL_LIMIT_VELOCITY_PID;

            pos_velocity_ctrl_config.P_pos_Integral_optimum =               Kp_POS_INTEGRAL_OPTIMUM;
            pos_velocity_ctrl_config.I_pos_Integral_optimum =               Ki_POS_INTEGRAL_OPTIMUM;
            pos_velocity_ctrl_config.D_pos_Integral_optimum =               Kd_POS_INTEGRAL_OPTIMUM;
            pos_velocity_ctrl_config.integral_limit_pos_Integral_optimum =  INTEGRAL_LIMIT_POS_INTEGRAL_OPTIMUM;

            pos_velocity_ctrl_config.position_fc =                          POSITION_FC;
            pos_velocity_ctrl_config.velocity_fc =                          VELOCITY_FC;

            pos_velocity_ctrl_config.P_nl_position_controller =         Kp_NL_POS_CONTROL;
            pos_velocity_ctrl_config.I_nl_position_controller =         Ki_NL_POS_CONTROL;
            pos_velocity_ctrl_config.D_nl_position_controller =         Kd_NL_POS_CONTROL;

            pos_velocity_ctrl_config.gain_p =                               GAIN_P;
            pos_velocity_ctrl_config.gain_i =                               GAIN_I;
            pos_velocity_ctrl_config.gain_d =                               GAIN_D;

            pos_velocity_ctrl_config.k_fb =                                 K_FB;
            pos_velocity_ctrl_config.k_m =                                  K_M;

            pos_velocity_ctrl_config.j =                                    MOMENT_OF_INERTIA;

            position_velocity_control_service(pos_velocity_ctrl_config, i_motorcontrol[3], i_position_control);
        }

        /************************************************************
         * IFM_TILE
         ************************************************************/
        on tile[IFM_TILE]:
        {
            par
            {
                /* PWM Service */
                {
                    pwm_config(pwm_ports);

                    delay_milliseconds(10);
                    if (!isnull(fet_driver_ports.p_esf_rst_pwml_pwmh) && !isnull(fet_driver_ports.p_coast))
                        predriver(fet_driver_ports);

                    delay_milliseconds(5);
                    //pwm_check(pwm_ports);//checks if pulses can be generated on pwm ports or not
                    pwm_service_task(MOTOR_ID, pwm_ports, i_update_pwm, DUTY_START_BRAKE, DUTY_MAINTAIN_BRAKE, PERIOD_START_BRAKE);
                }

                /* ADC Service */
                {
                    delay_milliseconds(10);
                    adc_service(adc_ports, null/*c_trigger*/, i_adc /*ADCInterface*/, i_watchdog[1]);
                }

                /* Watchdog Service */
                {
                    delay_milliseconds(5);
                    watchdog_service(wd_ports, i_watchdog);
                }

                /* Motor Control Service */
                {
                    delay_milliseconds(20);

                    MotorcontrolConfig motorcontrol_config;

                    motorcontrol_config.licence =  ADVANCED_MOTOR_CONTROL_LICENCE;
                    motorcontrol_config.v_dc =  VDC;
                    motorcontrol_config.commutation_loop_period =  COMMUTATION_LOOP_PERIOD;
                    motorcontrol_config.commutation_angle_offset=COMMUTATION_OFFSET_CLK;
                    motorcontrol_config.polarity_type=MOTOR_POLARITY;
                    motorcontrol_config.current_P_gain =  TORQUE_Kp;
                    motorcontrol_config.current_I_gain =  TORQUE_Ki;
                    motorcontrol_config.current_D_gain =  TORQUE_Kd;
                    motorcontrol_config.pole_pair =  POLE_PAIRS;
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

                    Motor_Control_Service(motorcontrol_config, i_adc[0], i_shared_memory[1],
                            i_watchdog[0], i_motorcontrol, i_update_pwm);
                }

                /* Shared memory Service */
                [[distribute]] memory_manager(i_shared_memory, 2);

                /* Position feedback service */
                {
                    PositionFeedbackConfig position_feedback_config;
                    position_feedback_config.sensor_type = MOTOR_COMMUTATION_SENSOR;

                    position_feedback_config.biss_config.multiturn_length = BISS_MULTITURN_LENGTH;
                    position_feedback_config.biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
                    position_feedback_config.biss_config.singleturn_length = BISS_SINGLETURN_LENGTH;
                    position_feedback_config.biss_config.singleturn_resolution = BISS_SINGLETURN_RESOLUTION;
                    position_feedback_config.biss_config.status_length = BISS_STATUS_LENGTH;
                    position_feedback_config.biss_config.crc_poly = BISS_CRC_POLY;
                    position_feedback_config.biss_config.pole_pairs = POLE_PAIRS;
                    position_feedback_config.biss_config.polarity = BISS_POLARITY;
                    position_feedback_config.biss_config.clock_dividend = BISS_CLOCK_DIVIDEND;
                    position_feedback_config.biss_config.clock_divisor = BISS_CLOCK_DIVISOR;
                    position_feedback_config.biss_config.timeout = BISS_TIMEOUT;
                    position_feedback_config.biss_config.max_ticks = BISS_MAX_TICKS;
                    position_feedback_config.biss_config.velocity_loop = BISS_VELOCITY_LOOP;
                    position_feedback_config.biss_config.offset_electrical = BISS_OFFSET_ELECTRICAL;
                    position_feedback_config.biss_config.enable_push_service = PushAll;

                    position_feedback_config.contelec_config.filter = CONTELEC_FILTER;
                    position_feedback_config.contelec_config.polarity = CONTELEC_POLARITY;
                    position_feedback_config.contelec_config.resolution_bits = CONTELEC_RESOLUTION;
                    position_feedback_config.contelec_config.offset = CONTELEC_OFFSET;
                    position_feedback_config.contelec_config.pole_pairs = POLE_PAIRS;
                    position_feedback_config.contelec_config.timeout = CONTELEC_TIMEOUT;
                    position_feedback_config.contelec_config.velocity_loop = CONTELEC_VELOCITY_LOOP;
                    position_feedback_config.contelec_config.enable_push_service = PushAll;

                    position_feedback_config.hall_config.pole_pairs = POLE_PAIRS;
                    position_feedback_config.hall_config.enable_push_service = PushAll;

                    position_feedback_config.qei_config.ticks_resolution = QEI_SENSOR_RESOLUTION;
                    position_feedback_config.qei_config.index_type = QEI_SENSOR_INDEX_TYPE;
                    position_feedback_config.qei_config.sensor_polarity = QEI_SENSOR_POLARITY;
                    position_feedback_config.qei_config.signal_type = QEI_SENSOR_SIGNAL_TYPE;
                    position_feedback_config.qei_config.enable_push_service = PushPosition;

                    position_feedback_service(hall_ports, qei_ports, spi_ports,
                                              position_feedback_config, i_shared_memory[0], i_position_feedback,
                                              null, null, null);
                }
            }
        }
    }

    return 0;
}
