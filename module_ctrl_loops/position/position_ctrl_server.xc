/**
 * @file  position_ctrl_server.xc
 * @brief Position Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/

#include <xscope.h>
#include <print.h>

#include <position_ctrl_server.h>
#include <position_ctrl_common.h>
#include <statemachine.h>
#include <drive_modes.h>
#include <a4935.h>
#include <internal_config.h>


#if(MOTOR_TYPE == BDC)
#include <brushed_dc_client.h>
#endif

//#define DEBUG
//#define debug_print

#define POSITION_CTRL_WRITE(x)  c_position_ctrl <: (x)
#define POSITION_CTRL_READ(x)   c_position_ctrl :> (x)


void position_control(ctrl_par &position_ctrl_params, hall_par &hall_params, qei_par &qei_params, int sensor_used,
                      chanend c_hall, chanend c_qei, client interface i_biss i_biss, chanend c_position_ctrl, chanend c_commutation)
{
    int actual_position = 0;
    int target_position = 0;

    int error_position = 0;
    int error_position_D = 0;
    int error_position_I = 0;
    int previous_error = 0;
    int position_control_out = 0;

    timer ts;
    unsigned int time;

    int command = 0;
    int activate = 0;
    int direction = 0;

    int fet_state = 0;
    int init_state = INIT_BUSY; /* check commutation init */

    printstr("*************************************\n    POSITION CONTROLLER STARTING\n*************************************\n");

    //printstrln("start pos");

    if (sensor_used == HALL) {
        { actual_position, direction } = get_hall_position_absolute(c_hall);
        target_position = actual_position;
    } else if (sensor_used == BISS) {
        { actual_position, void, void } = i_biss.get_position();
        target_position = actual_position;
    } else if (sensor_used == QEI) {
        {actual_position, direction} = get_qei_position_absolute(c_qei);
        target_position = actual_position;
    }

    /*
     * Or any other sensor interfaced to the IFM Module
     * place client functions here to acquire position
     */

    ts :> time;
    time += position_ctrl_params.Loop_time;

    while(1) {
#pragma ordered
        select {
        case ts when timerafter(time) :> void:
            time += position_ctrl_params.Loop_time;
            if (activate == 1) {
                /* acquire actual position hall/qei/sensor */
                switch (sensor_used) {
                case HALL:
                { actual_position, direction } = get_hall_position_absolute(c_hall);
                break;

                case BISS:
                { actual_position, void, void } = i_biss.get_position();
                break;

                case QEI:
                { actual_position, direction } =  get_qei_position_absolute(c_qei);
                break;

                /*
                 * Or any other sensor interfaced to the IFM Module
                 * place client functions here to acquire position
                 */
                }

                /* PID Controller */

                error_position = (target_position - actual_position);
                error_position_I = error_position_I + error_position;
                error_position_D = error_position - previous_error;

                if (error_position_I > position_ctrl_params.Integral_limit) {
                    error_position_I = position_ctrl_params.Integral_limit;
                } else if (error_position_I < -position_ctrl_params.Integral_limit) {
                    error_position_I = 0 - position_ctrl_params.Integral_limit;
                }

                position_control_out = ( (position_ctrl_params.Kp_n * error_position)/position_ctrl_params.Kp_d +
                                         (position_ctrl_params.Ki_n * error_position_I)/position_ctrl_params.Ki_d +
                                         (position_ctrl_params.Kd_n * error_position_D)/position_ctrl_params.Kd_d );

                if (position_control_out > position_ctrl_params.Control_limit) {
                    position_control_out = position_ctrl_params.Control_limit;
                } else if (position_control_out < -position_ctrl_params.Control_limit) {
                    position_control_out = 0 - position_ctrl_params.Control_limit;
                }

#if(MOTOR_TYPE == BDC)
                set_bdc_voltage(c_commutation, position_control_out);
#else
                set_commutation_sinusoidal(c_commutation, position_control_out);
#endif

#ifdef DEBUG
                xscope_int(ACTUAL_POSITION, actual_position);
                xscope_int(TARGET_POSITION, target_position);
#endif

                previous_error = error_position;
            }

            break;

        case POSITION_CTRL_READ(command):
            switch (command) {
            case PCTRL_CMD_SET_POSITION:
                POSITION_CTRL_READ(target_position);
                break;

            case PCTRL_CMD_GET_POSITION:
                POSITION_CTRL_WRITE(actual_position);
                break;

            case CHECK_BUSY:                                        /* Check init state */
                POSITION_CTRL_WRITE(activate);
                break;

            case PCTRL_CMD_SET_PARAMS:
                POSITION_CTRL_READ(position_ctrl_params.Kp_n);
                POSITION_CTRL_READ(position_ctrl_params.Kp_d);
                POSITION_CTRL_READ(position_ctrl_params.Ki_n);
                POSITION_CTRL_READ(position_ctrl_params.Ki_d);
                POSITION_CTRL_READ(position_ctrl_params.Kd_n);
                POSITION_CTRL_READ(position_ctrl_params.Kd_d);
                POSITION_CTRL_READ(position_ctrl_params.Integral_limit);
                break;

            case PCTRL_CMD_SET_HALL:
                c_position_ctrl :> hall_params.pole_pairs;
                break;

            case PCTRL_CMD_SET_QEI:
                c_position_ctrl :> qei_params.index;
                c_position_ctrl :> qei_params.real_counts;
                c_position_ctrl :> qei_params.max_ticks_per_turn;
                break;

            case PCTRL_CMD_SENSOR_SELECT:
                POSITION_CTRL_READ(sensor_used);
                if (sensor_used == HALL) {
                    { actual_position, direction }= get_hall_position_absolute(c_hall);
                } else if (sensor_used == BISS) {
                    { actual_position, void, void } = i_biss.get_position();
                } else if (sensor_used == QEI) {
                    { actual_position, direction } = get_qei_position_absolute(c_qei);
                }
                /*
                 * Or any other sensor interfaced to the IFM Module
                 * place client functions here to acquire position
                 */
                target_position = actual_position;
                break;

            case PCTRL_CMD_ENABLE:
                POSITION_CTRL_READ(activate);
                activate = SET;
                while (1) {
                    init_state = __check_commutation_init(c_commutation);
                    if(init_state == INIT) {
#ifdef debug_print
                        printstrln("commutation intialized");
#endif
#if(MOTOR_TYPE == BLDC)
                        fet_state = check_fet_state(c_commutation);
                        if (fet_state == 1) {
                            enable_motor(c_commutation);
                            wait_ms(2, 1, ts);
                        }
#endif
                        break;
                    }
                }
#ifdef debug_print
                printstrln("position control activated");
#endif
                break;

            case PCTRL_CMD_SHUTDOWN:
                POSITION_CTRL_READ(activate);
                set_commutation_sinusoidal(c_commutation, 0);
                error_position = 0;
                error_position_D = 0;
                error_position_I = 0;
                previous_error = 0;
                position_control_out = 0;
                disable_motor(c_commutation);
                wait_ms(30, 1, ts); //
#ifdef debug_print
                printstrln("position control disabled");
#endif
                break;

            case PCTRL_CMD_GET_STATUS: //check active state
                POSITION_CTRL_WRITE(activate);
                break;

            default:
                break;
            }
            break;
        }

    }
}
