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
#include <tuning.h>
#include <motor_control_interfaces.h>
#include <advanced_motor_control.h>
#include <position_feedback_service.h>
#include <torque_ripple_correction.h>

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

int main(void) {

    // Motor control interfaces
    interface WatchdogInterface i_watchdog[2];
    interface UpdatePWM i_update_pwm;
    interface UpdateBrake i_update_brake;
    interface ADCInterface i_adc[2];
    interface MotorControlInterface i_motorcontrol[2];
    interface PositionVelocityCtrlInterface i_position_control[3];
    interface PositionFeedbackInterface i_position_feedback_1[3];
    interface PositionFeedbackInterface i_position_feedback_2[3];
    interface shared_memory_interface i_shared_memory[3];
    interface TuningStepInterface i_tuning_step[3];

    par
    {
        /* WARNING: only one blocking task is possible per tile. */
        /* Waiting for a user input blocks other tasks on the same tile from execution. */
        on tile[APP_TILE]:
        {
            for (int generation_number = 5; generation_number < 15; generation_number+=3)
                //            {
                //                printf("///////////////////////////////\n"
                //                        "//// Test : %d Generations ////\n"
                //                        "///////////////////////////////\n", generation_number);
                //                struct individual tests [NUMBER_OF_TESTS];
                //                int result_parameters [MAX_COLUMNS][NUMBER_OF_TESTS];
                //
                //                for (int x = 0; x < NUMBER_OF_TESTS; x++)
                //                {
                //                    printf("\n---Test N°%d---\n", x);
                //                    tests[x] = autotune(i_position_control[0],generation_number,  i_tuning_step[1], i_position_feedback_1[0]);
                //                    result_parameters[0][x] =  tests[x].kp;
                //                    result_parameters[1][x] =  tests[x].ki;
                //                    result_parameters[2][x] =  tests[x].criterion;
                //                    result_parameters[3][x] =  tests[x].overshoot;
                //                    result_parameters[4][x] =  tests[x].oscillation;
                //                    result_parameters[5][x] =  NUMBER_OF_GENERATIONS - tests[x].age;
                //                }
                //
                //                char file_name[] = "Test_autotuning\0";
                //                char path[70];
                //                //            char headers[WRITE_BUFFER_SIZE] = "";
                //                char headers[] = "Kp, Ki, Criterion, Overshoot, Oscillation, Generation number";
                //                sprintf(path, "Data/Parameters/%s_%d_generations_%d_competitors.csv", file_name, generation_number, NUMBER_OF_COMPETITORS);
                //                write_array_to_csv_file(result_parameters, NUMBER_OF_TESTS, 6, path, headers);
                //
                //                //            int measurements_velocity[MAX_COLUMNS][NUMBER_OF_TESTS];
                //                i_tuning_step[1].set_reference_velocity(1000);
                //                i_tuning_step[1].set_time_zero(400);
                //                i_tuning_step[1].set_time_reference(500);
                //                i_tuning_step[1].start_steps(VELOCITY_CONTROL_ENABLE);
                //
                //                int time_min = INT_MAX;
                //                //            for (int i =0; i < NUMBER_OF_TESTS; i++)
                //                int i =0;
                //                for (int i = 0; i< NUMBER_OF_TESTS; i++)
                //                {
                //                    printf("test %d\n", i);
                //                    int time = 0;
                //                    compute_pid(tests, i, i_position_control[0]);
                //
                //
                //                    while (!i_tuning_step[1].get_reference_velocity_display());
                //
                //                    while (i_tuning_step[1].get_reference_velocity_display())
                //                    {
                //                        result_parameters[time][i] = i_position_feedback_1[0].get_velocity();
                //                        time++;
                //                        delay_microseconds(1000);
                //                    }
                //                    if (time < time_min)
                //                        time_min=time;
                //                    printf("Time : %d\n", time);
                //
                //                }
                //                i_tuning_step[1].stop_steps();
                //                strcpy(file_name, "Step_measurement");
                //                strcpy(headers ,"");
                //                sprintf(path, "Data/Curves/%s_%d_generations_%d_competitors.csv", file_name, generation_number, NUMBER_OF_COMPETITORS);
                //                write_array_to_csv_file(result_parameters, NUMBER_OF_TESTS, time_min, path, headers);
                //                printf("Done");
                //
                //            }
                //             exit(1);
                //                        autotune(i_position_control[0],10,  i_tuning_step[1], i_position_feedback_1[0]);
                user_interface(i_position_control[0], i_tuning_step[1]);

            //            map_torque_ripples(i_position_control[0], i_position_feedback_1[0]);
            //            demo_torque_position_velocity_control(i_position_control[0]);
        }

        on tile[APP_TILE_2]:
        /* Position Control Loop */
        {
            par
            {
                {
                    int count, status = 0;
                    int velocity_command = 0, velocity = 0;
                    int position_command = 0, position = 0;
                    int error_position = 0, error_velocity = 0;
                    int energy_error_position = 0, energy_error_velocity = 0;
                    int torque;
                    UpstreamControlData upstream_control_data;
                    while (1)
                    {

                        velocity_command = i_tuning_step[2].get_reference_velocity_display();
                        velocity = i_position_feedback_1[1].get_velocity();
                        error_velocity = velocity_command-velocity;
                        if (velocity_command)
                        {
                            energy_error_velocity += abs(error_velocity);
                        }
                        else energy_error_velocity = 0;

                        //                        velocity_command = i_tuning_step[2].get_reference_velocity();
                        {count, position, status} = i_position_feedback_1[1].get_position();
                        position_command = i_tuning_step[2].get_reference_position_display();
                        error_position = position_command-position;
                        if (position_command)
                        {
                            energy_error_position += abs(error_position);
                        }
                        else energy_error_position = 0;

                        upstream_control_data = i_motorcontrol[1].update_upstream_control_data();
                        //                        xscope_int(VELOCITY, velocity);
                        //                        xscope_int(VELOCITY_CMD, velocity_command);
                        //                        xscope_int(ERROR_VELOCITY, error_velocity);
                        //                        xscope_int(ENERGY_ERROR_VELOCITY, energy_error_velocity);
                        //                        xscope_int(POSITION, position);
                        //                        xscope_int(POSITION_CMD, position_command);
                        //                        xscope_int(ERROR_POSITION, error_position);
                        //                        xscope_int(ENERGY_ERROR_POSITION, energy_error_position);
                        //                        xscope_int(COUNT, count);
                        //                        xscope_int(TORQUE, upstream_control_data.computed_torque);
                        delay_microseconds(1000);
                    }

                }
                {
                    tuning_step_service(i_tuning_step);
                }
                {
                    make_steps(i_position_control[1], i_tuning_step[0], i_position_feedback_1[2]);
                }
                {
                    MotionControlConfig motion_ctrl_config;

                    motion_ctrl_config.min_pos_range_limit =                  MIN_POSITION_RANGE_LIMIT;
                    motion_ctrl_config.max_pos_range_limit =                  MAX_POSITION_RANGE_LIMIT;
                    motion_ctrl_config.max_motor_speed =                      MOTOR_MAX_SPEED;
                    motion_ctrl_config.polarity =                             POLARITY;

                    motion_ctrl_config.enable_profiler =                      ENABLE_PROFILER;
                    motion_ctrl_config.max_acceleration_profiler =            MAX_ACCELERATION_PROFILER;
                    motion_ctrl_config.max_deceleration_profiler =            MAX_DECELERATION_PROFILER;
                    motion_ctrl_config.max_speed_profiler =                   MAX_SPEED_PROFILER;

                    motion_ctrl_config.position_control_strategy =            NL_POSITION_CONTROLLER;

                    motion_ctrl_config.position_kp =                                POSITION_Kp;
                    motion_ctrl_config.position_ki =                                POSITION_Ki;
                    motion_ctrl_config.position_kd =                                POSITION_Kd;
                    motion_ctrl_config.position_integral_limit =                   POSITION_INTEGRAL_LIMIT;
                    motion_ctrl_config.moment_of_inertia =                    MOMENT_OF_INERTIA;

                    motion_ctrl_config.velocity_kp =                           VELOCITY_Kp;
                    motion_ctrl_config.velocity_ki =                           VELOCITY_Ki;
                    motion_ctrl_config.velocity_kd =                           VELOCITY_Kd;
                    motion_ctrl_config.velocity_integral_limit =              VELOCITY_INTEGRAL_LIMIT;

                    motion_ctrl_config.brake_release_strategy =                BRAKE_RELEASE_STRATEGY;
                    motion_ctrl_config.brake_release_delay =                 BRAKE_RELEASE_DELAY;

                    //select resolution of sensor used for motion control
                    if (SENSOR_2_FUNCTION == SENSOR_FUNCTION_COMMUTATION_AND_MOTION_CONTROL || SENSOR_2_FUNCTION == SENSOR_FUNCTION_MOTION_CONTROL) {
                        motion_ctrl_config.resolution  =                          SENSOR_2_RESOLUTION;
                    } else {
                        motion_ctrl_config.resolution  =                          SENSOR_1_RESOLUTION;
                    }

                    motion_ctrl_config.dc_bus_voltage=                        DC_BUS_VOLTAGE;
                    motion_ctrl_config.pull_brake_voltage=                    PULL_BRAKE_VOLTAGE;
                    motion_ctrl_config.pull_brake_time =                      PULL_BRAKE_TIME;
                    motion_ctrl_config.hold_brake_voltage =                   HOLD_BRAKE_VOLTAGE;

                    motion_control_service(APP_TILE_USEC, motion_ctrl_config, i_motorcontrol[0], i_position_control, i_update_brake);
                }
            }
        }

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

                    motorcontrol_config.dc_bus_voltage =  DC_BUS_VOLTAGE;
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

                    position_feedback_config.rem_14_config.hysteresis              = REM_14_SENSOR_HYSTERESIS;
                    position_feedback_config.rem_14_config.noise_settings          = REM_14_SENSOR_NOISE_SETTINGS;
                    position_feedback_config.rem_14_config.dyn_angle_error_comp    = REM_14_DYN_ANGLE_ERROR_COMPENSATION;
                    position_feedback_config.rem_14_config.abi_resolution_settings = REM_14_ABI_RESOLUTION_SETTINGS;

                    position_feedback_config.qei_config.number_of_channels = QEI_SENSOR_NUMBER_OF_CHANNELS;
                    position_feedback_config.qei_config.signal_type        = QEI_SENSOR_SIGNAL_TYPE;
                    position_feedback_config.qei_config.port_number        = QEI_SENSOR_PORT_NUMBER;

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
