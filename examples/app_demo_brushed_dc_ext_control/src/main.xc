/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>

/**
 * @brief Test illustrates usage of external ADC inputs for controlling a brushed DC motor. Example can be easily adapted for BLDC.
 */

#include <pwm_service.h>
#include <adc_service.h>
#include <watchdog_service.h>
#include <motorcontrol_service.h>
#include <print.h>

#include <user_config.h>

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
ADCPorts adc_ports = SOMANET_IFM_ADC_PORTS;

int main(void) {

    // Motor control channels
    chan c_pwm_ctrl;

    interface WatchdogInterface i_watchdog[2];
    interface MotorcontrolInterface i_motorcontrol[5];
    interface ADCInterface i_adc[5];

    par
    {
        /************************************************************
         * USER_TILE
         ************************************************************/

        on tile[APP_TILE_1]:
        {
            {
                while (1) {
                    int a, b, AI0, AI1;

                    {AI0 , AI1} =  i_adc[0].get_external_inputs();
                    int normalized_value = AI1*PWM_MAX_VALUE/MAX_ADC_VALUE;
                    printf("Voltage SP: %i\n", normalized_value);

                    i_motorcontrol[0].set_voltage(normalized_value);
                }
            }
        }

        /************************************************************
         * IFM_TILE
         ************************************************************/
        on tile[IFM_TILE]:
        {
            par
            {
                /* ADC Loop */
                adc_service(adc_ports, null, i_adc);

                /* PWM Loop */
                pwm_service(pwm_ports, c_pwm_ctrl);

                /* Watchdog Server */
                watchdog_service(wd_ports, i_watchdog);

                /* Brushed Motor Drive loop */
                {
                    MotorcontrolConfig motorcontrol_config;
                    motorcontrol_config.motor_type = BDC_MOTOR;
                    motorcontrol_config.commutation_loop_period =  COMMUTATION_LOOP_PERIOD;

                    motorcontrol_service(fet_driver_ports, motorcontrol_config, c_pwm_ctrl, null, null, null, i_watchdog[0],
                                                i_motorcontrol);
                }
            }
        }
    }

    return 0;
}
