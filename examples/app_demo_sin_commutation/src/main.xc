/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.inc>
#include <IFM_DC100-rev-b.inc>


/**
 * @brief Test illustrates usage of module_commutation
 * @date 17/06/2014
 */

#include <hall_service.h>
#include <adc_service.h>
#include <user_config.h>
#include <tuning.h>

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
ADCPorts adc_ports = SOMANET_IFM_ADC_PORTS;
HallPorts hall_ports = SOMANET_IFM_HALL_PORTS;

#define VOLTAGE -1000 //+/- 4095

void adc_client(interface ADCInterface client i_adc, interface HallInterface client i_hall){

    int b, c;
    unsigned state;

    while (1) {
        {b, c} = i_adc.get_currents();
        state = i_hall.get_hall_pinstate();
        xscope_int(PHASE_B, b);
        xscope_int(PHASE_C, c);
        xscope_int(HALL_PINS, state);
        delay_microseconds(10);
    }
}

int main(void) {

    // Motor control interfaces
    chan c_pwm_ctrl, c_adctrig; // pwm channels

    interface WatchdogInterface i_watchdog[2];
    interface ADCInterface i_adc[5];
    interface HallInterface i_hall[5];
    interface MotorcontrolInterface i_motorcontrol[5];

    par
    {

        on tile[APP_TILE_1]:
        {
            /* WARNING: only one blocking task is possible per tile. */
            /* Waiting for a user input blocks other tasks on the same tile from execution. */
            run_offset_tuning(VOLTAGE, i_motorcontrol[0]);
        }

            on tile[APP_TILE_2]: adc_client(i_adc[0], i_hall[1]);
   /*    { int a,b;
            while(1){
                a = i_adc[0].get_temperature();
                printf("%d %d\n",a,b);
            }
        }
*/
        on tile[IFM_TILE]:
        {
            par
            {
                /* ADC Loop */
                adc_service(adc_ports, c_adctrig, i_adc);

                /* Watchdog Server */
                watchdog_service(wd_ports, i_watchdog);

                /* PWM Loop */
                pwm_triggered_service( pwm_ports, c_adctrig, c_pwm_ctrl);

                /* Hall sensor Service */
                {
                    HallConfig hall_config;
                        hall_config.pole_pairs = POLE_PAIRS;

                    hall_service(hall_ports, hall_config, i_hall);
                }

                /* Motor Commutation Service */
                {
                    MotorcontrolConfig motorcontrol_config;
                        motorcontrol_config.motor_type = BLDC_MOTOR;
                        motorcontrol_config.bldc_winding_type = BLDC_WINDING_TYPE;
                        motorcontrol_config.hall_offset_clk =  COMMUTATION_OFFSET_CLK;
                        motorcontrol_config.hall_offset_cclk = COMMUTATION_OFFSET_CCLK;
                        motorcontrol_config.commutation_loop_period =  COMMUTATION_LOOP_PERIOD;

                    motorcontrol_service(fet_driver_ports, motorcontrol_config,
                                            c_pwm_ctrl, i_hall[0], null, i_watchdog[0], i_motorcontrol);
                }
            }
        }
    }

    return 0;
}
