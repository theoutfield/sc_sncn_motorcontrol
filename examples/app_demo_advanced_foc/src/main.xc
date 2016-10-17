/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC1K-rev-c3.bsp>


/**
 * @brief Test illustrates usage of module_commutation
 * @date 17/06/2014
 */

#include <tuning.h>
#include <pwm_server.h>
#include <pwm_ports.h>
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
//HallPorts hall_ports = SOMANET_IFM_HALL_PORTS;
SPIPorts spi_ports = SOMANET_IFM_AMS_PORTS;
//QEIPorts qei_ports = SOMANET_IFM_QEI_PORTS;

#define POSITION_LIMIT 5000000 //+/- 4095

int main(void) {

    // Motor control interfaces
    interface WatchdogInterface i_watchdog[2];
    interface ADCInterface i_adc[2];
    interface MotorcontrolInterface i_motorcontrol[1];
    interface shared_memory_interface i_shared_memory[2];
    interface PositionFeedbackInterface i_position_feedback[3];
    interface update_pwm i_update_pwm;
    interface TuningInterface i_tuning;

    par
    {
        /* WARNING: only one blocking task is possible per tile. */
        /* Waiting for a user input blocks other tasks on the same tile from execution. */
        //on tile[APP_TILE]: run_offset_tuning(POSITION_LIMIT, i_motorcontrol[0],i_tuning);

        on tile[APP_TILE]: demo_torque_control(i_motorcontrol[0]);

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
                    pwm_service_task(MOTOR_ID, pwm_ports, i_update_pwm,
                            DUTY_START_BRAKE, DUTY_MAINTAIN_BRAKE, PERIOD_START_BRAKE,
                            IFM_TILE_USEC);
                }

                /* ADC Service */
                {
                    delay_milliseconds(10);
                    adc_service(adc_ports, null/*c_trigger*/, i_adc /*ADCInterface*/, i_watchdog[1], IFM_TILE_USEC);
                }

                /* Watchdog Service */
                {
                    delay_milliseconds(5);
                    watchdog_service(wd_ports, i_watchdog, IFM_TILE_USEC);
                }

                /* Motor Control Service */
                {
                    delay_milliseconds(20);

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

                    Motor_Control_Service(motorcontrol_config, i_adc[0], i_shared_memory[1],
                            i_watchdog[0], i_motorcontrol, i_update_pwm, IFM_TILE_USEC);
                }

                /* Shared memory Service */
                [[distribute]] memory_manager(i_shared_memory, 2);

                /* Position feedback service */
                {
                    delay_milliseconds(10);
                    /*

                    PositionFeedbackConfig position_feedback_config;
                    position_feedback_config.sensor_type = HALL_SENSOR;
                    position_feedback_config.hall_config.pole_pairs = POLE_PAIRS;
                    position_feedback_config.hall_config.polarity = HALL_POLARITY;
                    position_feedback_config.hall_config.enable_push_service = PushAll;

                    position_feedback_service(hall_ports, null, null,
                                              position_feedback_config, i_shared_memory[0], i_position_feedback,
                                              null, null, null);
                     */

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


                    position_feedback_config.ams_config.factory_settings = 1;
                    position_feedback_config.ams_config.polarity = AMS_POLARITY;
                    position_feedback_config.ams_config.hysteresis = 1;
                    position_feedback_config.ams_config.noise_setting = AMS_NOISE_NORMAL;
                    position_feedback_config.ams_config.uvw_abi = 0;
                    position_feedback_config.ams_config.dyn_angle_comp = 0;
                    position_feedback_config.ams_config.data_select = 0;
                    position_feedback_config.ams_config.pwm_on = AMS_PWM_OFF;
                    position_feedback_config.ams_config.abi_resolution = 0;
                    position_feedback_config.ams_config.resolution_bits = AMS_RESOLUTION;
                    position_feedback_config.ams_config.offset = AMS_OFFSET;
                    position_feedback_config.ams_config.max_ticks = 0x7fffffff;
                    position_feedback_config.ams_config.pole_pairs = POLE_PAIRS;
                    position_feedback_config.ams_config.cache_time = AMS_CACHE_TIME;
                    position_feedback_config.ams_config.velocity_loop = AMS_VELOCITY_LOOP;
                    position_feedback_config.ams_config.enable_push_service = PushAll;

                    position_feedback_service(null, null, spi_ports,
                                              position_feedback_config, i_shared_memory[0], i_position_feedback,
                                              null, null, null);
                }
            }
        }
    }

    return 0;
}
