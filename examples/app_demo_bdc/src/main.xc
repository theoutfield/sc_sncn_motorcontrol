/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>

/**
 * @brief Test illustrates usage of module_commutation
 * @date 17/06/2014
 */
#include <xs1.h>
#include <platform.h>
#include <pwm_service_inv.h>
#include <commutation_server.h>
#include <brushed_dc_server.h>
#include <brushed_dc_client.h>
#include <refclk.h>
#include <adc_client_ad7949.h>
#include <adc_server_ad7949.h>
#include <internal_config.h> //FIXME: to use BDC motor, please change the parameter #define MOTOR_TYPE BDC
#include <brushed_dc_common.h>
#include <bldc_motor_config.h>
#include <xscope.h>
#include <stdio.h>

on tile[IFM_TILE]:clock clk_adc = XS1_CLKBLK_1;
on tile[IFM_TILE]:clock clk_pwm = XS1_CLKBLK_REF;

void set_BDC_motor_voltage(chanend c_commutation, int input_voltage){
    c_commutation <: BDC_CMD_SET_VOLTAGE;
    c_commutation <: input_voltage;
    return;
}

int main(void) {

    // Motor control channels
    chan c_pwm_ctrl, c_adctrig; // pwm channels
    chan c_watchdog;
    chan c_commutation;                     // motor drive channels
    chan c_adc;

    par
    {
        /************************************************************
         * USER_TILE
         ************************************************************/

        on tile[APP_TILE_1]:
        {
            {
                calib_data I_calib;
                do_adc_calibration_ad7949(c_adc, I_calib);
                while (1) {
                    int a, b, AI0, AI1;
                    {a, b} = get_adc_calibrated_current_ad7949(c_adc, I_calib);
         //         printf("phase currents: %i, %i\n", a, b);
                    {AI0 , AI1} = get_adc_external_ad7949(c_adc);
                    int normalized_value = AI1*13589/16383;
                    printf("Voltage SP: %i\n", normalized_value);

                    set_BDC_motor_voltage(c_commutation, normalized_value);//maximum 13589
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
                adc_ad7949_triggered(c_adc, c_adctrig, clk_adc,\
                        p_ifm_adc_sclk_conv_mosib_mosia, p_ifm_adc_misoa,\
                        p_ifm_adc_misob);

                /* PWM Loop */
                do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port,\
                        p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);

                /* Brushed Motor Drive loop */

                bdc_loop(c_watchdog, c_commutation, c_pwm_ctrl,\
                        p_ifm_esf_rstn_pwml_pwmh, p_ifm_coastn, p_ifm_ff1, p_ifm_ff2);


                /* Watchdog Server */
                run_watchdog(c_watchdog, p_ifm_wd_tick, p_ifm_shared_leds_wden);

            }
        }

    }

    return 0;
}
