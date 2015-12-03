/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.inc>
#include <IFM_DC100-rev-b.inc>

/**
 * @brief Test illustrates usage of module_commutation
 * @date 17/06/2014
 */
#include <pwm_service.h>
#include <commutation_service.h>
#include <brushed_dc_server.h>
#include <brushed_dc_client.h>
#include <adc_service.h>

#include <internal_config.h> //FIXME: to use BDC motor, please change the parameter #define MOTOR_TYPE BDC
#include <brushed_dc_common.h>
#include <bldc_motor_config.h>

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
ADCPorts adc_ports = SOMANET_IFM_ADC_PORTS;

int main(void) {

    // Motor control channels
    chan c_pwm_ctrl;

    interface WatchdogInterface i_watchdog;
    interface CommutationInterface i_commutation;
    interface ADCInterface i_adc;

    par
    {
        /************************************************************
         * USER_TILE
         ************************************************************/

        on tile[APP_TILE_1]:
        {
            {
                //calib_data I_calib;
                //do_adc_calibration_ad7949(c_adc, I_calib);
                while (1) {
                    int a, b, AI0, AI1;
                   // {a, b} = i_adc.get_currents();//get_adc_calibrated_current_ad7949(c_adc, I_calib);
         //         printf("phase currents: %i, %i\n", a, b);
                    {AI0 , AI1} =  i_adc.get_external_inputs();
                    int normalized_value = AI1*13589/16383;
                    printf("Voltage SP: %i\n", normalized_value);
                    //int normalized_value = 1000;
                    i_commutation.setVoltage(-1000);
                   // set_BDC_motor_voltage(i_commutation, normalized_value);//maximum 13589
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
                adc_service(i_adc, adc_ports, null);

                /* PWM Loop */
                pwm_service(c_pwm_ctrl, pwm_ports);

                /* Watchdog Server */
                watchdog_service(i_watchdog, wd_ports);

                /* Brushed Motor Drive loop */
                bdc_loop(c_pwm_ctrl, i_watchdog, i_commutation, fet_driver_ports);

            }
        }

    }

    return 0;
}
