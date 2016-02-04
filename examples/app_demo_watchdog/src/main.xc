/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC100-rev-b.bsp>

/**
 * @brief Test illustrates usage of module_watchdog
 * @date 5/01/2016
 */
#include <pwm_service.h>
#include <hall_service.h>
#include <watchdog_service.h>
#include <motorcontrol_service.h>

#include <user_config.h>

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
HallPorts hall_ports = SOMANET_IFM_HALL_PORTS;
BISSPorts biss_ports = {QEI_PORT, SOMANET_IFM_GPIO_D0, IFM_TILE_CLOCK_2};

int main(void) {

    // Motor control interfaces
    chan c_pwm_ctrl; // pwm channels

    interface WatchdogInterface i_watchdog[2];
    interface HallInterface i_hall[5];
    interface BISSInterface i_biss[5];
    interface MotorcontrolInterface i_motorcontrol[5];

    par
    {

        on tile[APP_TILE]:
        /* Client Side */
        {
            delay_seconds(1);
            printstrln("Motor starts spinning");
            i_motorcontrol[0].set_voltage(500); // Watchdog is first started by the
                                                // Motorcontrol Service

            delay_seconds(3);                   // Motor spins for 3 secs

            printstrln("Watchdog is disabled, motor stops");
            i_watchdog[1].stop();

            delay_seconds(2);                   // Motor still for 2 secs

            printstrln("Watchdog starts again, motor spins");
            i_motorcontrol[0].set_voltage(0);    // We set a 0 voltage before starting operation
            i_watchdog[1].start();
            delay_milliseconds(200);
            i_motorcontrol[0].set_voltage(500);
        }

        /***************************************************
         * IFM TILE
         ***************************************************/
        on tile[IFM_TILE]:
        {
            par
            {
                /* Watchdog Server */
                watchdog_service(wd_ports, i_watchdog);

                /* PWM Loop */
                pwm_service(pwm_ports, c_pwm_ctrl);

                /* Hall sensor Service */
                {
                    HallConfig hall_config;
                    hall_config.pole_pairs = POLE_PAIRS;

                    hall_service(hall_ports, hall_config, i_hall);
                }

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

                /* Motor Commutation Service */
                {
                    MotorcontrolConfig motorcontrol_config;
                    motorcontrol_config.motor_type = BLDC_MOTOR;
                    motorcontrol_config.commutation_sensor = MOTOR_COMMUTATION_SENSOR;
                    motorcontrol_config.bldc_winding_type = BLDC_WINDING_TYPE;
                    motorcontrol_config.hall_offset[0] =  COMMUTATION_OFFSET_CLK;
                    motorcontrol_config.hall_offset[1] = COMMUTATION_OFFSET_CCLK;
                    motorcontrol_config.commutation_loop_period =  COMMUTATION_LOOP_PERIOD;

                    motorcontrol_service(fet_driver_ports, motorcontrol_config,
                                            c_pwm_ctrl, i_hall[0], null, i_biss[0], i_watchdog[0], i_motorcontrol);
                }
            }
        }
    }

    return 0;
}
