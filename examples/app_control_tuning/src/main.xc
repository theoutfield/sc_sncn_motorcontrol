/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C21-DX_G2.bsp>
#include <IFM_DC1K-rev-d1.bsp>

/**
 * @file main.xc
 * @brief Demo application illustrates usage of module_pwm
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <pwm_server.h>
#include <adc_service.h>
#include <user_config.h>
#include <tuning_console.h>
#include <motor_control_interfaces.h>
#include <advanced_motor_control.h>
#include <position_feedback_service.h>

#include <xscope.h>
#include <timer.h>

PwmPortsGeneral pwm_ports = SOMANET_IFM_PWM_PORTS_GENERAL;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
ADCPorts adc_ports = SOMANET_IFM_ADC_PORTS;
SPIPorts spi_ports = SOMANET_IFM_SPI_PORTS;
HallEncSelectPort hall_enc_select_port = SOMANET_IFM_ENCODER_PORTS_INPUT_MODE_SELECTION;
port ? qei_hall_port_1 = SOMANET_IFM_ENCODER_1_PORT;
port ? qei_hall_port_2 = SOMANET_IFM_ENCODER_2_PORT;
port ?gpio_port_0 = SOMANET_IFM_GPIO_D0;
port ?gpio_port_1 = SOMANET_IFM_GPIO_D1;
port ?gpio_port_2 = SOMANET_IFM_GPIO_D2;
port ?gpio_port_3 = SOMANET_IFM_GPIO_D3;

void occupy_core(int foo)
{
    int x=foo;
    int y=2*foo;

    while(1)
    {
        x = x + foo;

        if(x>(y*1000000))
            x=foo;

        y=2*foo;
    }
}


int main(void) {

    // Motor control interfaces
    interface WatchdogInterface i_watchdog[2];
    interface UpdatePWMGeneral i_update_pwm;
    interface UpdateBrake i_update_brake;
    interface ADCInterface i_adc[2];
    interface TorqueControlInterface i_torque_control[2];
    interface MotionControlInterface i_motion_control[3];
    interface PositionFeedbackInterface i_position_feedback_1[3];
    interface PositionFeedbackInterface i_position_feedback_2[3];
    interface shared_memory_interface i_shared_memory[3];

    par
    {
        on tile[0]://APP TILE
        {
            par
            {
                {
                    occupy_core(10);
                }

                {
                    occupy_core(11);
                }

                {
                    occupy_core(12);
                }

                {
                    occupy_core(13);
                }

                {
                    occupy_core(14);
                }

                {
                    occupy_core(15);
                }

                {
                    occupy_core(16);
                }

                /* WARNING: only one blocking task is possible per tile. */
                /* Waiting for a user input blocks other tasks on the same tile from execution. */
                {
                    control_tuning_console(i_motion_control[0], i_position_feedback_1[0], i_position_feedback_2[0]);
                }
            }
        }

        on tile[1]://IFM TILE
        {
            par
            {
                {
                    occupy_core(20);
                }

                /* Shared memory Service */
                [[distribute]] shared_memory_service(i_shared_memory, 3);

                /* PWM Service */
                {
                    pwm_config_general(pwm_ports);

                    if (!isnull(fet_driver_ports.p_esf_rst_pwml_pwmh) && !isnull(fet_driver_ports.p_coast))
                        predriver(fet_driver_ports);

                    pwm_service_general(pwm_ports, i_update_pwm, GPWM_FRQ_15, DEADTIME_NS);
                }

                /* Watchdog Service */
                {
                    watchdog_service(wd_ports, i_watchdog, IFM_TILE_USEC);
                }

                /* ADC Service */
                {
                    adc_service(adc_ports, i_adc /*ADCInterface*/, i_watchdog[1], IFM_TILE_USEC, SINGLE_ENDED);
                }

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
                    position_feedback_config.offset      = HOME_OFFSET;
                    position_feedback_config.sensor_function = SENSOR_1_FUNCTION;

                    position_feedback_config.biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
                    position_feedback_config.biss_config.filling_bits = BISS_FILLING_BITS;
                    position_feedback_config.biss_config.crc_poly = BISS_CRC_POLY;
                    position_feedback_config.biss_config.clock_frequency = BISS_CLOCK_FREQUENCY;
                    position_feedback_config.biss_config.timeout = BISS_TIMEOUT;
                    position_feedback_config.biss_config.busy = BISS_BUSY;
                    position_feedback_config.biss_config.clock_port_config = BISS_CLOCK_PORT;
                    position_feedback_config.biss_config.data_port_number = BISS_DATA_PORT_NUMBER;
                    position_feedback_config.biss_config.data_port_signal_type = BISS_DATA_PORT_SIGNAL_TYPE;

                    position_feedback_config.rem_16mt_config.filter = REM_16MT_FILTER;

                    position_feedback_config.rem_14_config.hysteresis              = REM_14_SENSOR_HYSTERESIS;
                    position_feedback_config.rem_14_config.noise_settings          = REM_14_SENSOR_NOISE_SETTINGS;
                    position_feedback_config.rem_14_config.dyn_angle_error_comp    = REM_14_DYN_ANGLE_ERROR_COMPENSATION;
                    position_feedback_config.rem_14_config.abi_resolution_settings = REM_14_ABI_RESOLUTION_SETTINGS;

                    position_feedback_config.qei_config.number_of_channels = QEI_SENSOR_NUMBER_OF_CHANNELS;
                    position_feedback_config.qei_config.signal_type        = QEI_SENSOR_SIGNAL_TYPE;
                    position_feedback_config.qei_config.port_number        = QEI_SENSOR_PORT_NUMBER;
                    position_feedback_config.qei_config.ticks_lost_threshold = QEI_SENSOR_TICKS_LOST;

                    position_feedback_config.hall_config.port_number = HALL_SENSOR_PORT_NUMBER;
                    position_feedback_config.hall_config.hall_state_angle[0]=HALL_STATE_1_ANGLE;
                    position_feedback_config.hall_config.hall_state_angle[1]=HALL_STATE_2_ANGLE;
                    position_feedback_config.hall_config.hall_state_angle[2]=HALL_STATE_3_ANGLE;
                    position_feedback_config.hall_config.hall_state_angle[3]=HALL_STATE_4_ANGLE;
                    position_feedback_config.hall_config.hall_state_angle[4]=HALL_STATE_5_ANGLE;
                    position_feedback_config.hall_config.hall_state_angle[5]=HALL_STATE_6_ANGLE;

                    position_feedback_config.gpio_config[0] = GPIO_CONFIG_1;
                    position_feedback_config.gpio_config[1] = GPIO_CONFIG_2;
                    position_feedback_config.gpio_config[2] = GPIO_CONFIG_3;
                    position_feedback_config.gpio_config[3] = GPIO_CONFIG_4;

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

                /* Motor Control Service */
                {
                    MotorcontrolConfig motorcontrol_config;

                    motorcontrol_config.dc_bus_voltage =  DC_BUS_VOLTAGE;
                    motorcontrol_config.phases_inverted = MOTOR_PHASES_CONFIGURATION;

                    motorcontrol_config.torque_P_gain =  TORQUE_Kp;
                    motorcontrol_config.torque_I_gain =  TORQUE_Ki;
                    motorcontrol_config.torque_D_gain =  TORQUE_Kd;

                    motorcontrol_config.pole_pairs =  MOTOR_POLE_PAIRS;
                    motorcontrol_config.commutation_sensor=SENSOR_1_TYPE;
                    motorcontrol_config.commutation_angle_offset=COMMUTATION_ANGLE_OFFSET;
                    motorcontrol_config.max_torque =  MOTOR_MAXIMUM_TORQUE;
                    motorcontrol_config.phase_resistance =  MOTOR_PHASE_RESISTANCE;
                    motorcontrol_config.phase_inductance =  MOTOR_PHASE_INDUCTANCE;
                    motorcontrol_config.torque_constant1 =  MOTOR_TORQUE_CONSTANT;
                    motorcontrol_config.torque_constant2 =  MOTOR_TORQUE_CONSTANT;
                    motorcontrol_config.current_ratio =  CURRENT_RATIO;
                    motorcontrol_config.voltage_ratio =  VOLTAGE_RATIO;
                    motorcontrol_config.temperature_ratio =  TEMPERATURE_RATIO;
                    motorcontrol_config.rated_current =  MOTOR_RATED_CURRENT;
                    motorcontrol_config.rated_torque  =  MOTOR_RATED_TORQUE;
                    motorcontrol_config.percent_offset_torque =  APPLIED_TUNING_TORQUE_PERCENT;
                    motorcontrol_config.protection_limit_over_current =  PROTECTION_MAXIMUM_CURRENT;
                    motorcontrol_config.protection_limit_over_voltage =  PROTECTION_MAXIMUM_VOLTAGE;
                    motorcontrol_config.protection_limit_under_voltage = PROTECTION_MINIMUM_VOLTAGE;
                    motorcontrol_config.protection_limit_over_temperature = TEMP_BOARD_MAX;

                    for (int i = 0; i < 1024; i++)
                    {
                        motorcontrol_config.torque_offset[i] = 0;
                    }
                    torque_control_service(motorcontrol_config, i_adc[0], i_shared_memory[2],
                            i_watchdog[0], i_torque_control, i_update_pwm, IFM_TILE_USEC, /*gpio_port_0*/null);
                }

                /* Position Control Loop */
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

                    motion_ctrl_config.position_control_strategy =            POSITION_CONTROL_STRATEGY;

                    motion_ctrl_config.filter =                               FILTER_CUT_OFF_FREQ;

                    motion_ctrl_config.position_kp =                          POSITION_Kp;
                    motion_ctrl_config.position_ki =                          POSITION_Ki;
                    motion_ctrl_config.position_kd =                          POSITION_Kd;
                    motion_ctrl_config.position_integral_limit =              POSITION_INTEGRAL_LIMIT;
                    motion_ctrl_config.moment_of_inertia =                    MOMENT_OF_INERTIA;

                    motion_ctrl_config.velocity_kp =                          VELOCITY_Kp;
                    motion_ctrl_config.velocity_ki =                          VELOCITY_Ki;
                    motion_ctrl_config.velocity_kd =                          VELOCITY_Kd;
                    motion_ctrl_config.velocity_integral_limit =              VELOCITY_INTEGRAL_LIMIT;
                    motion_ctrl_config.enable_velocity_auto_tuner =           ENABLE_VELOCITY_AUTO_TUNER;
                    motion_ctrl_config.enable_compensation_recording =        ENABLE_COMPENSATION_RECORDING;
                    motion_ctrl_config.enable_open_phase_detection =          ENABLE_OPEN_PHASE_DETECTION;

                    motion_ctrl_config.position_kp_l =                        GAIN_SCHEDULING_POSITION_Kp_0;
                    motion_ctrl_config.position_ki_l =                        GAIN_SCHEDULING_POSITION_Ki_0;
                    motion_ctrl_config.position_kd_l =                        GAIN_SCHEDULING_POSITION_Kd_0;
                    motion_ctrl_config.position_kp_h =                        GAIN_SCHEDULING_POSITION_Kp_1;
                    motion_ctrl_config.position_ki_h =                        GAIN_SCHEDULING_POSITION_Ki_1;
                    motion_ctrl_config.position_kd_h =                        GAIN_SCHEDULING_POSITION_Kd_1;
                    motion_ctrl_config.velocity_kp_l =                        GAIN_SCHEDULING_VELOCITY_Kp_0;
                    motion_ctrl_config.velocity_ki_l =                        GAIN_SCHEDULING_VELOCITY_Ki_0;
                    motion_ctrl_config.velocity_kd_l =                        GAIN_SCHEDULING_VELOCITY_Kd_0;
                    motion_ctrl_config.velocity_kp_h =                        GAIN_SCHEDULING_VELOCITY_Kp_1;
                    motion_ctrl_config.velocity_ki_h =                        GAIN_SCHEDULING_VELOCITY_Ki_1;
                    motion_ctrl_config.velocity_kd_h =                        GAIN_SCHEDULING_VELOCITY_Kd_1;
                    motion_ctrl_config.velocity_lo_l =                        GAIN_SCHEDULING_VELOCITY_THRESHOLD_0;
                    motion_ctrl_config.velocity_hi_l =                        GAIN_SCHEDULING_VELOCITY_THRESHOLD_1;


                    motion_ctrl_config.brake_release_strategy =               BRAKE_RELEASE_STRATEGY;
                    motion_ctrl_config.brake_release_delay =                  BRAKE_RELEASE_DELAY;

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

                    motion_control_service(motion_ctrl_config, i_torque_control[0], i_motion_control, i_update_brake);
                }

            }
        }
    }

    return 0;
}

