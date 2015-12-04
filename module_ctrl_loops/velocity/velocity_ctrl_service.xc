/**
 * @file velocity_ctrl_server.xc
 * @brief Velocity Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <velocity_ctrl_service.h>
#include <refclk.h>
//#include <qei_client.h>
#include <filter_blocks.h>
#include <xscope.h>
#include <internal_config.h>
#include <statemachine.h>
#include <drive_modes.h>
#include <stdio.h>
#include <a4935.h>
#include <motorcontrol_service.h>


int init_velocity_control(interface VelocityControlInterface client i_velocity_control)
{
    int ctrl_state = INIT_BUSY;

    while (1) {
        ctrl_state = i_velocity_control.check_velocity_ctrl_state(); //check_velocity_ctrl_state(c_velocity_ctrl);
        if (ctrl_state == INIT_BUSY) {
            i_velocity_control.enable_velocity_ctrl();
        }

        if(ctrl_state == INIT) {
#ifdef debug_print
            printstrln("velocity control intialized");
#endif
            break;
        }
    }
    return ctrl_state;
}

int max_speed_limit(int velocity, int max_speed) {
    if (velocity > max_speed) {
        velocity = max_speed;
    } else if (velocity < -max_speed) {
        velocity = -max_speed;
    }
    return velocity;
}

//csv mode function
void set_velocity_csv(csv_par &csv_params, int target_velocity,
                      int velocity_offset, int torque_offset, interface VelocityControlInterface client i_velocity_control)
{
    i_velocity_control.set_velocity( max_speed_limit( (target_velocity + velocity_offset) * csv_params.polarity,
                                   csv_params.max_motor_speed ));
}

[[combinable]]
void velocity_control_service(ControlConfig &velocity_ctrl_params,
                       interface HallInterface client i_hall,
                       interface QEIInterface client ?i_qei,
                       interface VelocityControlInterface server i_velocity_control,
                       interface MotorcontrolInterface client commutation_interface )
{
    /* Controller declarations */
    int actual_velocity = 0;
    int target_velocity = 0;
    int error_velocity = 0;
    int error_velocity_D = 0;
    int error_velocity_I = 0;
    int previous_error = 0;
    int velocity_control_out = 0;

    timer ts;
    unsigned int time;

    /* Sensor filter declarations */
    int filter_length = DEFAULT_FILTER_LENGTH; //sensor_filter_params.filter_length;
    int filter_buffer[FILTER_SIZE_MAX] = {0};   //default size used at compile time (cant be changed further)
    int index = 0;

    /* speed calc declarations */
    int position;
    int init = 0;
    int previous_position = 0;
    int raw_speed = 0;                      // rpm
    int difference;
    int direction = 0;
    int old_difference;
    int rpm_constant = 1000*60; // constant
    int speed_factor_hall = 0;
    int speed_factor_qei = 0;
    int activate = 0;
    int init_state = INIT_BUSY;
    int qei_crossover = 0;
    int hall_crossover = 0;
    int compute_flag = 0;
    int fet_state = 0;

    HallConfig hall_config = i_hall.getHallConfig();
    QEIConfig qei_config;

    if(velocity_ctrl_params.sensor_used == QEI && !isnull(i_qei)){
        qei_config = i_qei.getQEIConfig();
    }

    init_filter(filter_buffer, index, FILTER_SIZE_MAX);
    if (velocity_ctrl_params.sensor_used == HALL){
        if(velocity_ctrl_params.Loop_time == MSEC_FAST){//FixMe: implement reference clock check
            speed_factor_hall = hall_config.pole_pairs*4096*(velocity_ctrl_params.Loop_time/MSEC_FAST); // variable pole_pairs
        }
        else {
            speed_factor_hall = hall_config.pole_pairs*4096*(velocity_ctrl_params.Loop_time/MSEC_STD); // variable pole_pairs
        }
        hall_crossover = hall_config.max_ticks - hall_config.max_ticks/10;
    }
    else if (velocity_ctrl_params.sensor_used == QEI){
        if(velocity_ctrl_params.Loop_time == MSEC_FAST){//FixMe: implement reference clock check
            speed_factor_qei = qei_config.real_counts*(velocity_ctrl_params.Loop_time/MSEC_FAST);       // variable qei_real_max
        }
        else {
            speed_factor_qei = qei_config.real_counts*(velocity_ctrl_params.Loop_time/MSEC_STD);       // variable qei_real_max
        }
        qei_crossover = qei_config.real_counts - qei_config.real_counts/10;
    }

    printf("*************************************\n    VELOCITY CONTROLLER STARTING\n*************************************\n");

    ts :> time;
    time += velocity_ctrl_params.Loop_time;

    init_state = INIT;

    while (1) {
//#pragma ordered
        select {
        case ts when timerafter (time) :> void:
            time += velocity_ctrl_params.Loop_time;
            if (compute_flag == 1) {
                /* calculate actual velocity from hall/qei with filter*/
                if (velocity_ctrl_params.sensor_used == HALL) {
                    if (init == 0) {
                        { position, direction } = i_hall.get_hall_position_absolute();//get_hall_position_absolute(c_hall);
                        if (position > 2049) {
                            init = 1;
                            previous_position = 2049;
                        } else if (position < -2049) {
                            init = 1;
                            previous_position = -2049;
                        }
                        raw_speed = 0;
                        //target_velocity = 0;
                    } else if (init == 1) {
                        { position, direction } = i_hall.get_hall_position_absolute();//get_hall_position_absolute(c_hall);
                        difference = position - previous_position;
                        if (difference > hall_crossover) {
                            difference = old_difference;
                        } else if (difference < -hall_crossover) {
                            difference = old_difference;
                        }
                        raw_speed = (difference*rpm_constant)/speed_factor_hall;
#ifdef Debug_velocity_ctrl
                        //xscope_int(RAW_SPEED, raw_speed);
#endif
                        previous_position = position;
                        old_difference = difference;
                    }
                } else if (velocity_ctrl_params.sensor_used == QEI) {
                    { position, direction } = i_qei.get_qei_position_absolute();
                    difference = position - previous_position;

                    if (difference > qei_crossover) {
                        difference = old_difference;
                    }

                    if (difference < -qei_crossover) {
                        difference = old_difference;
                    }

                    raw_speed = (difference*rpm_constant)/speed_factor_qei;

#ifdef Debug_velocity_ctrl
                    //xscope_int(RAW_SPEED, raw_speed);
#endif
                    previous_position = position;
                    old_difference = difference;
                }
                /**
                 * Or any other sensor interfaced to the IFM Module
                 * place client functions here to acquire velocity/position
                 */

                actual_velocity = filter(filter_buffer, index, filter_length, raw_speed);
            }

            if(activate == 1) {
#ifdef Debug_velocity_ctrl
                xscope_int(ACTUAL_VELOCITY, actual_velocity);
                xscope_int(TARGET_VELOCITY, target_velocity);
#endif
                compute_flag = 1;
                /* Controller */
                error_velocity   = (target_velocity - actual_velocity);
                error_velocity_I = error_velocity_I + error_velocity;
                error_velocity_D = error_velocity - previous_error;

                if (error_velocity_I > (velocity_ctrl_params.Integral_limit)) {
                    error_velocity_I = (velocity_ctrl_params.Integral_limit);
                } else if (error_velocity_I < -(velocity_ctrl_params.Integral_limit)) {
                    error_velocity_I = 0 -(velocity_ctrl_params.Integral_limit);
                }

                velocity_control_out = ( (velocity_ctrl_params.Kp_n*error_velocity) / (velocity_ctrl_params.Kp_d) +
                                         (velocity_ctrl_params.Ki_n*error_velocity_I) / (velocity_ctrl_params.Ki_d) +
                                         (velocity_ctrl_params.Kd_n*error_velocity_D) / (velocity_ctrl_params.Kd_d) );

                if (velocity_control_out > velocity_ctrl_params.Control_limit) {
                    velocity_control_out = velocity_ctrl_params.Control_limit;
                } else if (velocity_control_out < -velocity_ctrl_params.Control_limit) {
                    velocity_control_out = 0 - velocity_ctrl_params.Control_limit;
                }

                commutation_interface.setVoltage(velocity_control_out);//set_commutation_sinusoidal(c_commutation, velocity_control_out);//velocity_control_out

                previous_error = error_velocity;

            }
    //        printf("looping %d\n", velocity_ctrl_params.Loop_time);
            break;

        case i_velocity_control.set_velocity(int in_velocity):
            target_velocity = in_velocity;

            break;

        case i_velocity_control.get_velocity()-> int out_velocity:

            out_velocity = actual_velocity;

            break;

        case i_velocity_control.set_velocity_ctrl_param(ControlConfig in_params):

            velocity_ctrl_params.Kp_n = in_params.Kp_n;
            velocity_ctrl_params.Kp_d = in_params.Kp_d;
            velocity_ctrl_params.Ki_n = in_params.Ki_n;
            velocity_ctrl_params.Ki_d = in_params.Ki_d;
            velocity_ctrl_params.Kd_n = in_params.Kd_n;
            velocity_ctrl_params.Kd_d = in_params.Kd_d;
            velocity_ctrl_params.Integral_limit = in_params.Integral_limit;

            break;

        case i_velocity_control.set_velocity_filter(int in_length):

            filter_length = in_length;

            if(filter_length > FILTER_SIZE_MAX)
                filter_length = FILTER_SIZE_MAX;

            break;

        case i_velocity_control.set_velocity_ctrl_hall_param(HallConfig in_config):

            hall_config.pole_pairs = in_config.pole_pairs;
            hall_config.max_ticks = in_config.max_ticks;
            hall_config.max_ticks_per_turn = in_config.max_ticks_per_turn;
            break;

        case i_velocity_control.set_velocity_ctrl_qei_param(QEIConfig in_params):

            qei_config.max_ticks = in_params.max_ticks;
            qei_config.index = in_params.index;
            qei_config.real_counts = in_params.real_counts;
            qei_config.max_ticks_per_turn = in_params.max_ticks_per_turn;
            qei_config.poles = in_params.poles;

            break;

        case i_velocity_control.set_velocity_sensor(int in_sensor_used):

            velocity_ctrl_params.sensor_used = in_sensor_used;

            if(in_sensor_used == HALL) {
                speed_factor_hall = hall_config.pole_pairs * 4096 * (velocity_ctrl_params.Loop_time/MSEC_STD);
                hall_crossover = hall_config.max_ticks - hall_config.max_ticks/10;

            } else if(in_sensor_used == QEI) {
                speed_factor_qei = qei_config.real_counts * (velocity_ctrl_params.Loop_time/MSEC_STD);
                qei_crossover = qei_config.max_ticks - qei_config.max_ticks/10;
            }
            target_velocity = actual_velocity;
            break;


        case i_velocity_control.shutdown_velocity_ctrl():

            activate = 0;
            error_velocity = 0;
            error_velocity_D = 0;
            error_velocity_I = 0;
            previous_error = 0;
            velocity_control_out = 0;
            commutation_interface.setVoltage(0); //set_commutation_sinusoidal(c_commutation, 0);
            commutation_interface.disableFets();//disable_motor(c_commutation);
            wait_ms(30, 1, ts);
            break;

        case i_velocity_control.check_velocity_ctrl_state() -> int out_state:

            out_state = activate;
            break;

        case i_velocity_control.check_busy() -> int out_state:

                out_state = activate;
                break;

        case i_velocity_control.enable_velocity_ctrl():

            activate = SET;
            while (1) {
                init_state = commutation_interface.checkBusy();//__check_commutation_init(c_commutation);
                if (init_state == INIT) {
#ifdef debug_print
                    printf("commutation intialized\n");
#endif
                    fet_state = commutation_interface.getFetsState();//check_fet_state(c_commutation);
                    if (fet_state == 1) {
                        commutation_interface.enableFets();//enable_motor(c_commutation);
                        wait_ms(2, 1, ts);
                    }
                    break;
                }
            }

#ifdef debug_print
                printf("velocity control activated\n");
#endif
            break;

        case i_velocity_control.get_velocity_control_config() -> ControlConfig out_config:

                out_config = velocity_ctrl_params;
                break;

            }
           // break;
        }

}
