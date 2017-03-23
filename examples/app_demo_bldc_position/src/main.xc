/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC1K-rev-c3.bsp>

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
#include <position_feedback_service.h>

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
ADCPorts adc_ports = SOMANET_IFM_ADC_PORTS;
QEIHallPort qei_hall_port_1 = SOMANET_IFM_HALL_PORTS;
QEIHallPort qei_hall_port_2 = SOMANET_IFM_QEI_PORTS;
HallEncSelectPort hall_enc_select_port = SOMANET_IFM_QEI_PORT_INPUT_MODE_SELECTION;
SPIPorts spi_ports = SOMANET_IFM_SPI_PORTS;
port ?gpio_port_0 = SOMANET_IFM_GPIO_D0;
port ?gpio_port_1 = SOMANET_IFM_GPIO_D1;
port ?gpio_port_2 = SOMANET_IFM_GPIO_D2;
port ?gpio_port_3 = SOMANET_IFM_GPIO_D3;


//Position control + profile libs
#include <motion_control_service.h>
#include <profile_control.h>


/* Test Profile Position function */
void position_profile_test(interface PositionVelocityCtrlInterface client i_position_control)
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
    profiler_config.max_position = MAX_POSITION_RANGE_LIMIT;
    profiler_config.min_position = MIN_POSITION_RANGE_LIMIT;
    //select resolution of sensor used for motion control
    if (SENSOR_2_FUNCTION == SENSOR_FUNCTION_COMMUTATION_AND_MOTION_CONTROL || SENSOR_2_FUNCTION == SENSOR_FUNCTION_MOTION_CONTROL) {
        profiler_config.ticks_per_turn  =                          SENSOR_2_RESOLUTION;
    } else {
        profiler_config.ticks_per_turn  =                          SENSOR_1_RESOLUTION;
    }
    profiler_config.max_velocity = MAX_MOTOR_SPEED;
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
    interface MotorControlInterface i_motorcontrol[2];
    interface update_pwm i_update_pwm;
    interface update_brake i_update_brake;
    interface shared_memory_interface i_shared_memory[3];
    interface PositionVelocityCtrlInterface i_position_control[3];
    interface PositionFeedbackInterface i_position_feedback_1[3];
    interface PositionFeedbackInterface i_position_feedback_2[3];

    par
    {
        /* Test Profile Position Client function*/
        on tile[APP_TILE]:
        {
            position_profile_test(i_position_control[0]);      // test PPM on slave side
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
            MotionControlConfig pos_velocity_ctrl_config;

            pos_velocity_ctrl_config.min_pos_range_limit =                  MIN_POSITION_RANGE_LIMIT;
            pos_velocity_ctrl_config.max_pos_range_limit =                  MAX_POSITION_RANGE_LIMIT;
            pos_velocity_ctrl_config.max_motor_speed =                      MAX_MOTOR_SPEED;
            pos_velocity_ctrl_config.polarity =                             POLARITY;

            pos_velocity_ctrl_config.enable_profiler =                      ENABLE_PROFILER;
            pos_velocity_ctrl_config.max_acceleration_profiler =            MAX_ACCELERATION_PROFILER;
            pos_velocity_ctrl_config.max_speed_profiler =                   MAX_SPEED_PROFILER;

            pos_velocity_ctrl_config.position_control_strategy =            NL_POSITION_CONTROLLER;

            pos_velocity_ctrl_config.position_kp =                                POSITION_Kp;
            pos_velocity_ctrl_config.position_ki =                                POSITION_Ki;
            pos_velocity_ctrl_config.position_kd =                                POSITION_Kd;
            pos_velocity_ctrl_config.position_integral_limit =                   POSITION_INTEGRAL_LIMIT;
            pos_velocity_ctrl_config.moment_of_inertia =                    MOMENT_OF_INERTIA;

            pos_velocity_ctrl_config.velocity_kp =                           VELOCITY_Kp;
            pos_velocity_ctrl_config.velocity_ki =                           VELOCITY_Ki;
            pos_velocity_ctrl_config.velocity_kd =                           VELOCITY_Kd;
            pos_velocity_ctrl_config.velocity_integral_limit =              VELOCITY_INTEGRAL_LIMIT;

            pos_velocity_ctrl_config.special_brake_release =                ENABLE_SHAKE_BRAKE;
            pos_velocity_ctrl_config.brake_shutdown_delay =                 BRAKE_SHUTDOWN_DELAY;

            //select resolution of sensor used for motion control
            if (SENSOR_2_FUNCTION == SENSOR_FUNCTION_COMMUTATION_AND_MOTION_CONTROL || SENSOR_2_FUNCTION == SENSOR_FUNCTION_MOTION_CONTROL) {
                pos_velocity_ctrl_config.resolution  =                          SENSOR_2_RESOLUTION;
            } else {
                pos_velocity_ctrl_config.resolution  =                          SENSOR_1_RESOLUTION;
            }

            pos_velocity_ctrl_config.dc_bus_voltage=                        DC_BUS_VOLTAGE;
            pos_velocity_ctrl_config.pull_brake_voltage=                    PULL_BRAKE_VOLTAGE;
            pos_velocity_ctrl_config.pull_brake_time =                      PULL_BRAKE_TIME;
            pos_velocity_ctrl_config.hold_brake_voltage =                   HOLD_BRAKE_VOLTAGE;

            motion_control_service(APP_TILE_USEC, pos_velocity_ctrl_config, i_motorcontrol[0], i_position_control, i_update_brake);
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

                    if (!isnull(fet_driver_ports.p_esf_rst_pwml_pwmh) && !isnull(fet_driver_ports.p_coast))
                        predriver(fet_driver_ports);

                    //pwm_check(pwm_ports);//checks if pulses can be generated on pwm ports or not
                    pwm_service_task(MOTOR_ID, pwm_ports, i_update_pwm,
                            i_update_brake, IFM_TILE_USEC);

                }

                /* ADC Service */
                {
                    adc_service(adc_ports, i_adc /*ADCInterface*/, i_watchdog[1], IFM_TILE_USEC, SINGLE_ENDED);
                }

                /* Watchdog Service */
                {
                    watchdog_service(wd_ports, i_watchdog, IFM_TILE_USEC);
                }

                /* Motor Control Service */
                {

                    MotorcontrolConfig motorcontrol_config;

                    motorcontrol_config.v_dc =  DC_BUS_VOLTAGE;
                    motorcontrol_config.phases_inverted = MOTOR_PHASES_NORMAL;
                    motorcontrol_config.torque_P_gain =  TORQUE_P_VALUE;
                    motorcontrol_config.torque_I_gain =  TORQUE_I_VALUE;
                    motorcontrol_config.torque_D_gain =  TORQUE_D_VALUE;
                    motorcontrol_config.pole_pairs =  MOTOR_POLE_PAIRS;
                    motorcontrol_config.commutation_sensor=SENSOR_1_TYPE;
                    motorcontrol_config.commutation_angle_offset=COMMUTATION_ANGLE_OFFSET;
                    motorcontrol_config.hall_state_angle[0]=HALL_STATE_1_ANGLE;
                    motorcontrol_config.hall_state_angle[1]=HALL_STATE_2_ANGLE;
                    motorcontrol_config.hall_state_angle[2]=HALL_STATE_3_ANGLE;
                    motorcontrol_config.hall_state_angle[3]=HALL_STATE_4_ANGLE;
                    motorcontrol_config.hall_state_angle[4]=HALL_STATE_5_ANGLE;
                    motorcontrol_config.hall_state_angle[5]=HALL_STATE_6_ANGLE;
                    motorcontrol_config.max_torque =  MOTOR_MAXIMUM_TORQUE;
                    motorcontrol_config.phase_resistance =  MOTOR_PHASE_RESISTANCE;
                    motorcontrol_config.phase_inductance =  MOTOR_PHASE_INDUCTANCE;
                    motorcontrol_config.torque_constant =  MOTOR_TORQUE_CONSTANT;
                    motorcontrol_config.current_ratio =  CURRENT_RATIO;
                    motorcontrol_config.voltage_ratio =  VOLTAGE_RATIO;
                    motorcontrol_config.rated_current =  MOTOR_RATED_CURRENT;
                    motorcontrol_config.rated_torque  =  MOTOR_RATED_TORQUE;
                    motorcontrol_config.percent_offset_torque =  APPLIED_TUNING_TORQUE_PERCENT;
                    motorcontrol_config.protection_limit_over_current =  PROTECTION_MAXIMUM_CURRENT;
                    motorcontrol_config.protection_limit_over_voltage =  PROTECTION_MAXIMUM_VOLTAGE;
                    motorcontrol_config.protection_limit_under_voltage = PROTECTION_MINIMUM_VOLTAGE;

                    motor_control_service(motorcontrol_config, i_adc[0], i_shared_memory[2],
                            i_watchdog[0], i_motorcontrol, i_update_pwm, IFM_TILE_USEC);
                }

                /* Shared memory Service */
                [[distribute]] shared_memory_service(i_shared_memory, 3);

                /* Position feedback service */
                {
                    PositionFeedbackConfig position_feedback_config;
                    position_feedback_config.sensor_type = SENSOR_1_TYPE;
                    position_feedback_config.resolution  = SENSOR_1_RESOLUTION;
                    position_feedback_config.polarity    = SENSOR_1_POLARITY;
                    position_feedback_config.velocity_compute_period = SENSOR_1_VELOCITY_COMPUTE_PERIOD;
                    position_feedback_config.pole_pairs  = MOTOR_POLE_PAIRS;
                    position_feedback_config.ifm_usec    = IFM_TILE_USEC;
                    position_feedback_config.max_ticks   = SENSOR_MAX_TICKS;
                    position_feedback_config.offset      = 0;
                    position_feedback_config.sensor_function = SENSOR_1_FUNCTION;

                    position_feedback_config.biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
                    position_feedback_config.biss_config.filling_bits = BISS_FILLING_BITS;
                    position_feedback_config.biss_config.crc_poly = BISS_CRC_POLY;
                    position_feedback_config.biss_config.clock_frequency = BISS_CLOCK_FREQUENCY;
                    position_feedback_config.biss_config.timeout = BISS_TIMEOUT;
                    position_feedback_config.biss_config.busy = BISS_BUSY;
                    position_feedback_config.biss_config.clock_port_config = BISS_CLOCK_PORT;
                    position_feedback_config.biss_config.data_port_number = BISS_DATA_PORT_NUMBER;

                    position_feedback_config.rem_16mt_config.filter = REM_16MT_FILTER;

                    position_feedback_config.rem_14_config.hysteresis     = REM_14_SENSOR_HYSTERESIS ;
                    position_feedback_config.rem_14_config.noise_setting  = REM_14_SENSOR_NOISE;
                    position_feedback_config.rem_14_config.dyn_angle_comp = REM_14_SENSOR_DAE;
                    position_feedback_config.rem_14_config.abi_resolution = REM_14_SENSOR_ABI_RES;

                    position_feedback_config.qei_config.index_type  = QEI_SENSOR_INDEX_TYPE;
                    position_feedback_config.qei_config.signal_type = QEI_SENSOR_SIGNAL_TYPE;
                    position_feedback_config.qei_config.port_number = QEI_SENSOR_PORT_NUMBER;

                    position_feedback_config.hall_config.port_number = HALL_SENSOR_PORT_NUMBER;

                    //setting second sensor
                    PositionFeedbackConfig position_feedback_config_2 = position_feedback_config;
                    position_feedback_config_2.sensor_type = 0;
                    if (SENSOR_2_FUNCTION != SENSOR_FUNCTION_DISABLED) //enable second sensor
                    {
                        position_feedback_config_2.sensor_type = SENSOR_2_TYPE;
                        position_feedback_config_2.polarity    = SENSOR_2_POLARITY;
                        position_feedback_config_2.resolution  = SENSOR_2_RESOLUTION;
                        position_feedback_config_2.velocity_compute_period = SENSOR_2_VELOCITY_COMPUTE_PERIOD;
                        position_feedback_config_2.sensor_function = SENSOR_2_FUNCTION;
                    }

                    position_feedback_service(qei_hall_port_1, qei_hall_port_2, hall_enc_select_port, spi_ports, gpio_port_0, gpio_port_1, gpio_port_2, gpio_port_3,
                            position_feedback_config, i_shared_memory[0], i_position_feedback_1,
                            position_feedback_config_2, i_shared_memory[1], i_position_feedback_2);
                }
            }
        }
    }

    return 0;
}
