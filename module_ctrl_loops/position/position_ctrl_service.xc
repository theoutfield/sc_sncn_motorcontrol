/**
 * @file  position_ctrl_server.xc
 * @brief Position Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/

#include <xscope.h>
#include <print.h>

#include <position_ctrl_service.h>
#include <a4935.h>
#include <internal_config.h>
#include <hall_service.h>
#include <qei_service.h>






int init_position_control(interface PositionControlInterface client i_position_control)
{
    int ctrl_state = INIT_BUSY;

    while (1) {
        ctrl_state = i_position_control.check_position_ctrl_state();
        if (ctrl_state == INIT_BUSY) {
            i_position_control.enable_position_ctrl();
        }

        if (ctrl_state == INIT) {
#ifdef debug_print
            printstrln("position control intialized");
#endif
            break;
        }
    }
    return ctrl_state;
}

int position_limit(int position, int max_position_limit, int min_position_limit)
{
    if (position > max_position_limit) {
        position = max_position_limit;
    } else if (position < min_position_limit) {
        position = min_position_limit;
    }
    return position;
}

void set_position_csp( CyclicSyncPositionConfig & csp_params, int target_position, int position_offset,
                       int velocity_offset, int torque_offset, interface PositionControlInterface client i_position_control )
{
    i_position_control.set_position( position_limit( (target_position + position_offset) * csp_params.velocity_config.polarity,
                                  csp_params.max_position_limit,
                                  csp_params.min_position_limit));
}


void position_control_service(ControlConfig &position_ctrl_params,
                                interface HallInterface client ?i_hall,
                                interface QEIInterface client ?i_qei,
                                interface MotorcontrolInterface client commutation_interface,
                                interface PositionControlInterface server i_position_control[3])
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

    int activate = 0;
    int direction = 0;

    int fet_state;
    int init_state = INIT_BUSY; /* check commutation init */

    HallConfig hall_config;
    QEIConfig qei_config;

    if(position_ctrl_params.sensor_used == HALL && !isnull(i_hall)){
        hall_config = i_hall.getHallConfig();
    } else if(position_ctrl_params.sensor_used >= QEI && !isnull(i_qei)){
        qei_config = i_qei.getQEIConfig();
    }

    printstr("*************************************\n    POSITION CONTROLLER STARTING\n*************************************\n");

    if (position_ctrl_params.sensor_used == HALL && !isnull(i_hall)) {
        { actual_position, direction } = i_hall.get_hall_position_absolute();// get_hall_position_absolute(c_hall);
        target_position = actual_position;
    } else if (position_ctrl_params.sensor_used >= QEI && !isnull(i_qei)) {
        {actual_position, direction} = i_qei.get_qei_position_absolute();
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
                switch (position_ctrl_params.sensor_used) {
                    case HALL:
                    { actual_position, direction } = i_hall.get_hall_position_absolute();//get_hall_position_absolute(c_hall);
                    break;

                    case QEI:
                    { actual_position, direction } =  i_qei.get_qei_position_absolute();
                    break;

                    case QEI_WITH_INDEX:
                    { actual_position, direction } =  i_qei.get_qei_position_absolute();
                    break;

                    case QEI_WITH_NO_INDEX:
                    { actual_position, direction } =  i_qei.get_qei_position_absolute();
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


               // set_commutation_sinusoidal(c_commutation, position_control_out);
                commutation_interface.setVoltage(position_control_out);

#ifdef DEBUG
                xscope_int(ACTUAL_POSITION, actual_position);
                xscope_int(TARGET_POSITION, target_position);
#endif
                //xscope_int(TARGET_POSITION, target_position);
                previous_error = error_position;
            }

            break;

        case i_position_control[int i].set_position(int in_target_position):

            target_position = in_target_position;

            break;

        case i_position_control[int i].get_position() -> int out_position:

                out_position = actual_position;
                break;

        case i_position_control[int i].get_target_position() -> int out_set_position:

                out_set_position = target_position;
                break;

        case i_position_control[int i].check_busy() -> int out_activate:

                out_activate = activate;
                break;

        case i_position_control[int i].set_position_ctrl_param(ControlConfig in_params):

            position_ctrl_params.Kp_n = in_params.Kp_n;
            position_ctrl_params.Kp_d = in_params.Kp_d;
            position_ctrl_params.Ki_n = in_params.Ki_n;
            position_ctrl_params.Ki_d = in_params.Ki_d;
            position_ctrl_params.Kd_n = in_params.Kd_n;
            position_ctrl_params.Kd_d = in_params.Kd_d;
            position_ctrl_params.Integral_limit = in_params.Integral_limit;

            break;

        case i_position_control[int i].set_position_ctrl_hall_param(HallConfig in_config):

            hall_config.pole_pairs = in_config.pole_pairs;
            break;

        case i_position_control[int i].set_position_ctrl_qei_param(QEIConfig in_qei_params):

            qei_config.index_type = in_qei_params.index_type;
            qei_config.ticks_resolution = in_qei_params.ticks_resolution;
            //qei_config.max_ticks_per_turn = in_qei_params.max_ticks_per_turn;
            break;

        case i_position_control[int i].set_position_sensor(int in_sensor_used):

            position_ctrl_params.sensor_used = in_sensor_used;

            if (in_sensor_used == HALL) {
                { actual_position, direction }= i_hall.get_hall_position_absolute();
            } else if (in_sensor_used >= QEI) {
                { actual_position, direction } = i_qei.get_qei_position_absolute();
            }
            /*
             * Or any other sensor interfaced to the IFM Module
             * place client functions here to acquire position
             */
            target_position = actual_position;

            break;

        case i_position_control[int i].enable_position_ctrl():
                        activate = SET;
                            while (1) {
                                init_state = commutation_interface.checkBusy(); //__check_commutation_init(c_commutation);
                                if(init_state == INIT) {
            #ifdef debug_print
                                    printstrln("commutation intialized");
            #endif
                                    fet_state = commutation_interface.getFetsState(); // check_fet_state(c_commutation);
                                    if (fet_state == 1) {
                                        commutation_interface.enableFets();
                                        delay_milliseconds(2);
                                    }

                                    break;
                                }
                            }
            #ifdef debug_print
                            printstrln("position control activated");
            #endif
                            break;

        case i_position_control[int i].shutdown_position_ctrl():
            activate = 0;
            commutation_interface.setVoltage(0);
            //set_commutation_sinusoidal(c_commutation, 0);
            error_position = 0;
            error_position_D = 0;
            error_position_I = 0;
            previous_error = 0;
            position_control_out = 0;
            commutation_interface.disableFets();
            // disable_motor(c_commutation);
            delay_milliseconds(30);
            //wait_ms(30, 1, ts); //
#ifdef debug_print
            printstrln("position control disabled");
#endif
            break;

        case i_position_control[int i].check_position_ctrl_state() -> int out_state:
                out_state = activate;

                break;

        case i_position_control[int i].getControlConfig() ->  ControlConfig out_config:

                out_config = position_ctrl_params;
                break;
        case i_position_control[int i].getHallConfig() -> HallConfig out_config:

                out_config = hall_config;
                break;
        case i_position_control[int i].getQEIConfig() -> QEIConfig out_config:

                out_config = qei_config;
                break;

        }
        }

    }

