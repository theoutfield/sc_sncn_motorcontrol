/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>

/**
 * @file main.xc
 * @brief Test illustrates usage of the watchdog
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <watchdog_service.h>
#include <refclk.h>
#include <motor_control_interfaces.h>
#include <pwm_server.h>
#include <user_config.h>
#include <ctype.h>

void watchdog_commands(client interface WatchdogInterface i_watchdog)
{
    delay_milliseconds(1250);
    printstrln("Start watchdog commands:\np [number]: to set a fault\n[enter]: to reset the watchdog\ne: to read the error code");

    while(1) {
        char mode = 0;
        char c;
        int value = 0;
        int sign = 1;
        //reading user input.
        while((c = getchar ()) != '\n'){
            if(isdigit(c)>0){
                value *= 10;
                value += c - '0';
            } else if (c == '-') {
                sign = -1;
            } else if (c != ' ')
                mode = c;
        }

        switch(mode) {
        case 'p':
            i_watchdog.protect(value);
            printf("protect %d\n", value);
            break;
        case 'e':
            WatchdogError watchdog_error = i_watchdog.read_fault_monitor();
            switch(watchdog_error) {
            case WATCHDOG_NO_ERROR:
                printstrln("WATCHDOG_NO_ERROR");
                break;
            case WATCHDOG_TICKS_ERROR:
                printstrln("WATCHDOG_TICKS_ERROR");
                break;
            case WATCHDOG_OVER_UNDER_VOLTAGE_OVER_TEMP_ERROR:
                printstrln("WATCHDOG_OVER_UNDER_VOLTAGE_OVER_TEMP_ERROR");
                break;
            default:
                printf("watchdog error: %d\n", watchdog_error);
                break;
            }
            break;
        default:
            i_watchdog.reset_faults();
            printstrln("reset watchdog");
            break;
        }
        delay_milliseconds(25);
    }
}

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;

int main() {
    interface WatchdogInterface i_watchdog[2];
    interface UpdatePWM i_update_pwm;
    interface UpdateBrake i_update_brake;

    par {
        /************************************************************
         * IFM_TILE
         ************************************************************/

        on tile[COM_TILE]: watchdog_commands(i_watchdog[0]);

        on tile[IFM_TILE]: par {

            /* PWM Service */
            {
                pwm_config(pwm_ports);

                if (!isnull(fet_driver_ports.p_esf_rst_pwml_pwmh) && !isnull(fet_driver_ports.p_coast))
                    predriver(fet_driver_ports);

                //pwm_check(pwm_ports);//checks if pulses can be generated on pwm ports or not
                pwm_service_task(MOTOR_ID, pwm_ports, i_update_pwm,
                        i_update_brake, IFM_TILE_USEC);

            }

            /* Watchdog Service */
            {
                printstrln("start watchdog");
                watchdog_service(wd_ports, i_watchdog, IFM_TILE_USEC);
            }
        }
    }
    return 0;
}
