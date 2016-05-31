/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>

/**
 * @brief Test illustrates usage of module_commutation
 * @date 17/06/2014
 */

#include <pwm_service.h>
#include <position_feedback_service.h>
#include <adc_service.h>
#include <user_config.h>
#include <tuning.h>

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
ADCPorts adc_ports = SOMANET_IFM_ADC_PORTS;
PositionFeedbackPorts position_feedback_ports = SOMANET_IFM_POSITION_FEEDBACK_PORTS;

#define POSITION_LIMIT 0 //+/- 4095

int main(void) {

    // Motor control interfaces
    chan c_pwm_ctrl, c_adctrig; // pwm channels

    interface WatchdogInterface i_watchdog[2];
    interface ADCInterface i_adc[2];
    interface MotorcontrolInterface i_motorcontrol[4];
//    interface PositionControlInterface i_position_control[3];
    interface TuningInterface i_tuning;
    interface BrakeInterface i_brake;
    interface PositionFeedbackInterface i_position_feedback[3];
    interface shared_memory_interface i_shared_memory[2];

    par
    {
        /* WARNING: only one blocking task is possible per tile. */
        /* Waiting for a user input blocks other tasks on the same tile from execution. */
        on tile[APP_TILE]: run_offset_tuning(POSITION_LIMIT, i_motorcontrol[0], i_tuning);

        /* Tuning service */
        on tile[APP_TILE_2]: tuning_service(i_tuning, i_motorcontrol[1], i_adc[1], null, i_position_feedback[0]);

//        on tile[APP_TILE_2]:
//        /* Position Control Loop */
//        {
//            ControlConfig position_control_config;
//            position_control_config.feedback_sensor = MOTOR_FEEDBACK_SENSOR;
//            position_control_config.Kp_n = POSITION_Kp;    // Divided by 10000
//            position_control_config.Ki_n = POSITION_Ki;    // Divided by 10000
//            position_control_config.Kd_n = POSITION_Kd;    // Divided by 10000
//            position_control_config.control_loop_period = CONTROL_LOOP_PERIOD; //us
//            /* Control Loop */
//#if(MOTOR_COMMUTATION_SENSOR == BISS_SENSOR)
//            position_control_service(position_control_config, i_motorcontrol[3],
//                    i_position_control);
//#elif(MOTOR_COMMUTATION_SENSOR == AMS_SENSOR)
//            position_control_service(position_control_config, i_motorcontrol[3],
//                    i_position_control);
//#else
//            position_control_service(position_control_config, i_motorcontrol[3],
//                    i_position_control);
//#endif
//        }


        on tile[IFM_TILE]:
        {
            par
            {
                /* Triggered PWM Service */
                pwm_triggered_service(pwm_ports, c_adctrig, c_pwm_ctrl, i_brake);
                i_brake.set_brake(0);

                /* ADC Service */
                adc_service(adc_ports, c_adctrig, i_adc, i_watchdog[1]);

                /* Watchdog Service */
                watchdog_service(wd_ports, i_watchdog);

                /* Position service */
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

                    position_feedback_service(position_feedback_ports, position_feedback_config, i_shared_memory[1], i_position_feedback, null, null, null, null);
                }


                /* Shared memory Service */
                memory_manager(i_shared_memory, 2);

                /* Motor Control Service */
                {
                    MotorcontrolConfig motorcontrol_config;
                    motorcontrol_config.motor_type = BLDC_MOTOR;
                    motorcontrol_config.polarity_type = MOTOR_POLARITY;
                    motorcontrol_config.commutation_method = FOC;
                    motorcontrol_config.commutation_sensor = MOTOR_COMMUTATION_SENSOR;
                    motorcontrol_config.bldc_winding_type = BLDC_WINDING_TYPE;
                    motorcontrol_config.hall_offset[0] = COMMUTATION_OFFSET_CLK;
                    motorcontrol_config.hall_offset[1] = COMMUTATION_OFFSET_CCLK;
                    motorcontrol_config.commutation_loop_period =  COMMUTATION_LOOP_PERIOD;
                    motorcontrol_service(fet_driver_ports, motorcontrol_config,
                                         c_pwm_ctrl, i_adc[0], i_shared_memory[0], i_watchdog[0], null, i_motorcontrol);
                }
            }
        }
    }

    return 0;
}
