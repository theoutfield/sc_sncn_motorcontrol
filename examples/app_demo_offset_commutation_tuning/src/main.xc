/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC100-rev-b.bsp>

/**
 * @brief Test illustrates usage of module_commutation
 * @date 17/06/2014
 */

#include <pwm_service.h>
#include <hall_service.h>
#include <adc_service.h>
#include <user_config.h>
#include <tuning.h>

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
ADCPorts adc_ports = SOMANET_IFM_ADC_PORTS;
HallPorts hall_ports = SOMANET_IFM_HALL_PORTS;
BISSPorts biss_ports = {QEI_PORT, SOMANET_IFM_GPIO_D0, IFM_TILE_CLOCK_2};

#define VOLTAGE 1000 //+/- 4095

void adc_client(interface ADCInterface client i_adc, interface HallInterface client ?i_hall){

    int b, c;
    unsigned state;

    while (1) {

        {b, c} = i_adc.get_currents();
        if (!isnull(i_hall)) {
            state = i_hall.get_hall_pinstate();
            xscope_int(HALL_PINS, state);
        }

        xscope_int(PHASE_B, b);
        xscope_int(PHASE_C, c);

        delay_milliseconds(1000);
    }
}

int main(void) {

    // Motor control interfaces
    chan c_pwm_ctrl, c_adctrig; // pwm channels

    interface WatchdogInterface i_watchdog[2];
    interface ADCInterface i_adc[5];
    interface MotorcontrolInterface i_motorcontrol[5];
#if(MOTOR_COMMUTATION_SENSOR == BISS_SENSOR)
    interface BISSInterface i_biss[5];
#else
    interface HallInterface i_hall[5];
#endif

    par
    {
        /* WARNING: only one blocking task is possible per tile. */
        /* Waiting for a user input blocks other tasks on the same tile from execution. */
#if(MOTOR_COMMUTATION_SENSOR == BISS_SENSOR)
        on tile[APP_TILE_2]: adc_client(i_adc[0], null);
        on tile[APP_TILE_1]: run_offset_tuning(VOLTAGE, i_motorcontrol[0], i_biss[1]);
#else
        on tile[APP_TILE_2]: adc_client(i_adc[0], i_hall[1]);
        on tile[APP_TILE_1]: run_offset_tuning(VOLTAGE, i_motorcontrol[0], null);
#endif

        on tile[IFM_TILE]:
        {
            par
            {
                /* Triggered PWM Service */
                pwm_triggered_service( pwm_ports, c_adctrig, c_pwm_ctrl);

                /* ADC Service */
                adc_service(adc_ports, c_adctrig, i_adc);

                /* Watchdog Service */
                watchdog_service(wd_ports, i_watchdog);

#if(MOTOR_COMMUTATION_SENSOR == BISS_SENSOR)
                /* BiSS service */
                {
                    BISSConfig biss_config;
                    biss_config.multiturn_length = BISS_MULTITURN_LENGTH;
                    biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
                    biss_config.singleturn_length = BISS_SINGLETURN_LENGTH;
                    biss_config.singleturn_resolution = BISS_SINGLETURN_RESOLUTION;
                    biss_config.status_length = BISS_STATUS_LENGTH;
                    biss_config.crc_poly = BISS_CRC_POLY;
                    biss_config.pole_pairs = POLE_PAIRS;
                    biss_config.polarity = BISS_POLARITY;
                    biss_config.clock_dividend = BISS_CLOCK_DIVIDEND;
                    biss_config.clock_divisor = BISS_CLOCK_DIVISOR;
                    biss_config.timeout = BISS_TIMEOUT;
                    biss_config.max_ticks = BISS_MAX_TICKS;
                    biss_config.velocity_loop = BISS_VELOCITY_LOOP;
                    biss_config.offset_electrical = BISS_OFFSET_ELECTRICAL;

                    biss_service(biss_ports, biss_config, i_biss);
                }
#else
                /* Hall sensor Service */
                {
                    HallConfig hall_config;
                    hall_config.pole_pairs = POLE_PAIRS;

                    hall_service(hall_ports, hall_config, i_hall);
                }
#endif

                /* Motor Control Service */
                {
                    MotorcontrolConfig motorcontrol_config;
                    motorcontrol_config.motor_type = BLDC_MOTOR;
                    motorcontrol_config.commutation_sensor = MOTOR_COMMUTATION_SENSOR;
                    motorcontrol_config.bldc_winding_type = BLDC_WINDING_TYPE;
                    motorcontrol_config.hall_offset[0] = COMMUTATION_OFFSET_CLK;
                    motorcontrol_config.hall_offset[1] = COMMUTATION_OFFSET_CCLK;
                    motorcontrol_config.commutation_loop_period =  COMMUTATION_LOOP_PERIOD;
#if(MOTOR_COMMUTATION_SENSOR == BISS_SENSOR)
                    motorcontrol_service(fet_driver_ports, motorcontrol_config,
                                         c_pwm_ctrl, null, null, i_biss[0], i_watchdog[0], i_motorcontrol);
#else
                    motorcontrol_service(fet_driver_ports, motorcontrol_config,
                                         c_pwm_ctrl, i_hall[0], null, null, i_watchdog[0], i_motorcontrol);
#endif
                }
            }
        }
    }

    return 0;
}
