/*
 * app_test_autotuning.xc
 *
 *  Created on: Mar 15, 2017
 *      Author: jgofre
 */
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC1K-rev-c4.bsp>

#include <pwm_server.h>
#include <adc_service.h>
#include <user_config.h>
#include <tuning.h>
#include <motor_control_interfaces.h>
#include <advanced_motor_control.h>
#include <advanced_motorcontrol_licence.h>
#include <position_feedback_service.h>
#include <print.h>
#include <stdio.h>
#include <stdlib.h>
#include <syscall.h>
#include <torque_ripple_correction.h>
#include <data_processing.h>
#include <string.h>

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
    interface update_pwm i_update_pwm;
    interface update_brake i_update_brake;
    interface ADCInterface i_adc[2];
    interface MotorcontrolInterface i_motorcontrol[2];
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
//            for (int generation_number = 5; generation_number < 15; generation_number+=3)
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
            autotune(i_position_control[0],10,  i_tuning_step[1], i_position_feedback_1[0]);
//            user_interface(i_position_control[0], i_tuning_step[1]);

//            map_torque_ripples(i_position_control[0], i_position_feedback_1[0]);
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
                        xscope_int(VELOCITY, velocity);
                        xscope_int(VELOCITY_CMD, velocity_command);
                        xscope_int(ERROR_VELOCITY, error_velocity);
                        xscope_int(ENERGY_ERROR_VELOCITY, energy_error_velocity);
                        xscope_int(POSITION, position);
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
                    PosVelocityControlConfig pos_velocity_ctrl_config;
                    /* Control Loop */
                    pos_velocity_ctrl_config.control_loop_period =                  CONTROL_LOOP_PERIOD; //us

                    pos_velocity_ctrl_config.min_pos =                              MIN_POSITION_LIMIT;
                    pos_velocity_ctrl_config.max_pos =                              MAX_POSITION_LIMIT;
                    pos_velocity_ctrl_config.pos_limit_threshold =                  POSITION_LIMIT_THRESHOLD;
                    pos_velocity_ctrl_config.max_speed =                            MAX_SPEED;
                    pos_velocity_ctrl_config.max_torque =                           TORQUE_CONTROL_LIMIT;
                    pos_velocity_ctrl_config.polarity =                             POLARITY;

                    pos_velocity_ctrl_config.enable_profiler =                      ENABLE_PROFILER;
                    pos_velocity_ctrl_config.max_acceleration_profiler =            MAX_ACCELERATION_PROFILER;
                    pos_velocity_ctrl_config.max_speed_profiler =                   MAX_SPEED_PROFILER;

                    pos_velocity_ctrl_config.control_mode =                         NL_POSITION_CONTROLLER;

                    pos_velocity_ctrl_config.P_pos =                                POSITION_Kp;
                    pos_velocity_ctrl_config.I_pos =                                POSITION_Ki;
                    pos_velocity_ctrl_config.D_pos =                                POSITION_Kd;
                    pos_velocity_ctrl_config.integral_limit_pos =                   POSITION_INTEGRAL_LIMIT;
                    pos_velocity_ctrl_config.j =                                    MOMENT_OF_INERTIA;

                    pos_velocity_ctrl_config.P_velocity =                           VELOCITY_Kp;
                    pos_velocity_ctrl_config.I_velocity =                           VELOCITY_Ki;
                    pos_velocity_ctrl_config.D_velocity =                           VELOCITY_Kd;
                    pos_velocity_ctrl_config.integral_limit_velocity =              VELOCITY_INTEGRAL_LIMIT;

                    pos_velocity_ctrl_config.position_fc =                          POSITION_FC;
                    pos_velocity_ctrl_config.velocity_fc =                          VELOCITY_FC;
                    pos_velocity_ctrl_config.resolution  =                          FEEDBACK_SENSOR_RESOLUTION;
                    pos_velocity_ctrl_config.special_brake_release =                ENABLE_SHAKE_BRAKE;
                    pos_velocity_ctrl_config.brake_shutdown_delay =                 BRAKE_SHUTDOWN_DELAY;

                    pos_velocity_ctrl_config.voltage_pull_brake=                    VOLTAGE_PULL_BRAKE;
                    pos_velocity_ctrl_config.time_pull_brake =                      TIME_PULL_BRAKE;
                    pos_velocity_ctrl_config.voltage_hold_brake =                   VOLTAGE_HOLD_BRAKE;

                    init_brake(i_update_brake, IFM_TILE_USEC, VDC,
                            pos_velocity_ctrl_config.voltage_pull_brake,
                            pos_velocity_ctrl_config.time_pull_brake,
                            pos_velocity_ctrl_config.voltage_hold_brake);

                    position_velocity_control_service(pos_velocity_ctrl_config, i_motorcontrol[0], i_position_control);
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
                    motorcontrol_config.hall_state_angle[0]=HALL_STATE_1_ANGLE;
                    motorcontrol_config.hall_state_angle[1]=HALL_STATE_2_ANGLE;
                    motorcontrol_config.hall_state_angle[2]=HALL_STATE_3_ANGLE;
                    motorcontrol_config.hall_state_angle[3]=HALL_STATE_4_ANGLE;
                    motorcontrol_config.hall_state_angle[4]=HALL_STATE_5_ANGLE;
                    motorcontrol_config.hall_state_angle[5]=HALL_STATE_6_ANGLE;
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

                    motor_control_service(motorcontrol_config, i_adc[0], i_shared_memory[2],
                            i_watchdog[0], i_motorcontrol, i_update_pwm, IFM_TILE_USEC);
                }

                /* Shared memory Service */
                [[distribute]] memory_manager(i_shared_memory, 3);

                /* Position feedback service */
                {
                    PositionFeedbackConfig position_feedback_config;
                    position_feedback_config.sensor_type = MOTOR_COMMUTATION_SENSOR;
                    position_feedback_config.polarity    = NORMAL_POLARITY;
                    position_feedback_config.pole_pairs  = POLE_PAIRS;
                    position_feedback_config.resolution  = COMMUTATION_SENSOR_RESOLUTION;
                    position_feedback_config.ifm_usec    = IFM_TILE_USEC;
                    position_feedback_config.max_ticks   = SENSOR_MAX_TICKS;
                    position_feedback_config.velocity_compute_period   = COMMUTATION_VELOCITY_COMPUTE_PERIOD;
                    position_feedback_config.offset      = 0;
                    position_feedback_config.enable_push_service = PushAll;

                    position_feedback_config.biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
                    position_feedback_config.biss_config.filling_bits = BISS_FILLING_BITS;
                    position_feedback_config.biss_config.crc_poly = BISS_CRC_POLY;
                    position_feedback_config.biss_config.clock_frequency = BISS_CLOCK_FREQUENCY;
                    position_feedback_config.biss_config.timeout = BISS_TIMEOUT;
                    position_feedback_config.biss_config.busy = BISS_BUSY;
                    position_feedback_config.biss_config.clock_port_config = BISS_CLOCK_PORT;
                    position_feedback_config.biss_config.data_port_config = BISS_DATA_PORT;

                    position_feedback_config.rem_16mt_config.filter = REM_16MT_FILTER;

                    position_feedback_config.rem_14_config.factory_settings = 1;
                    position_feedback_config.rem_14_config.hysteresis = 1;
                    position_feedback_config.rem_14_config.noise_setting = REM_14_NOISE_NORMAL;
                    position_feedback_config.rem_14_config.uvw_abi = 0;
                    position_feedback_config.rem_14_config.dyn_angle_comp = 0;
                    position_feedback_config.rem_14_config.data_select = 0;
                    position_feedback_config.rem_14_config.pwm_on = REM_14_PWM_OFF;
                    position_feedback_config.rem_14_config.abi_resolution = 0;

                    position_feedback_config.qei_config.index_type = QEI_SENSOR_INDEX_TYPE;
                    position_feedback_config.qei_config.signal_type = QEI_SENSOR_SIGNAL_TYPE;

                    //setting second sensor
                    PositionFeedbackConfig position_feedback_config_2 = position_feedback_config;
                    position_feedback_config_2.sensor_type = 0;
                    if (MOTOR_COMMUTATION_SENSOR != MOTOR_FEEDBACK_SENSOR) //enable second sensor when different from the first one
                    {
                        position_feedback_config_2.sensor_type = MOTOR_FEEDBACK_SENSOR;
                        position_feedback_config_2.polarity    = FEEDBACK_SENSOR_POLARITY;
                        position_feedback_config_2.resolution  = 1<<16;

                        position_feedback_config_2.velocity_compute_period = FEEDBACK_VELOCITY_COMPUTE_PERIOD;
                        //position_feedback_config_2.enable_push_service = PushPosition;
                        position_feedback_config_2.enable_push_service = NoPush;
                        //position_feedback_config_2.enable_push_service = PushAll;

                        position_feedback_config_2.rem_16mt_config.filter = REM_16MT_FILTER;
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


