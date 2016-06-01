/**
 * @file velocity_ctrl_server.xc
 * @brief Velocity Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */
#include <xs1.h>
#include <velocity_ctrl_service.h>
#include <motorcontrol_service.h>
#include <mc_internal_constants.h>

#include <filter_blocks.h>
#include <limits.h>
#include <refclk.h>
#include <print.h>
#include <stdlib.h>

void init_velocity_control(interface VelocityControlInterface client i_velocity_control)
{
    int ctrl_state;

    while (1) {
        ctrl_state = i_velocity_control.check_busy();
        if (ctrl_state == INIT_BUSY) {
            i_velocity_control.enable_velocity_ctrl();
        }

        if (ctrl_state == INIT) {
#ifdef debug_print
            printstrln("velocity_ctrl_service: velocity control initialized");
#endif
            break;
        }
    }
}

int max_speed_limit(int velocity, int max_speed) {
    if (velocity > max_speed) {
        velocity = max_speed;
    } else if (velocity < -max_speed) {
        velocity = -max_speed;
    }
    return velocity;
}

[[combinable]]
void velocity_control_service(ControlConfig &velocity_control_config,
                       interface MotorcontrolInterface client i_motorcontrol,
                       interface VelocityControlInterface server i_velocity_control[3])
{
    /* Controller declarations */
    int actual_velocity = 0;
    int target_velocity = 0;
    int error_velocity = 0;
    int error_velocity_D = 0;
    int error_velocity_I = 0;
    int previous_error = 0;
    int velocity_control_out = 0;

    int velocity_control_out_limit = 0;
    int error_velocity_I_limit = 0;

    timer t;
    unsigned int ts;

    /* Sensor filter declarations */
    int filter_length = DEFAULT_FILTER_LENGTH; //sensor_filter_params.filter_length;
    int filter_buffer[FILTER_SIZE_MAX] = {0};   //default size used at compile ts (cant be changed further)
    int index = 0;

    /* speed calc declarations */
    int position;
    int init = 0;
    int previous_position = 0;
    int raw_speed = 0;                      // rpm
    int difference;
    int old_difference;
    int rpm_constant = 1000*60; // constant
    int speed_factor;
    int crossover;
    int activate = 0;
    int compute_flag = 0;

    int config_update_flag = 1;
    MotorcontrolConfig motorcontrol_config;

    printstr(">>   SOMANET VELOCITY CONTROL SERVICE STARTING...\n");

    t :> ts;

    while (1) {
//#pragma ordered
        select {
            case t when timerafter (ts + USEC_STD * velocity_control_config.control_loop_period) :> ts:

                if (config_update_flag) {
                    motorcontrol_config = i_motorcontrol.get_config();

                    //Limits
                    if (motorcontrol_config.motor_type == BLDC_MOTOR) {
                        if(motorcontrol_config.commutation_method == FOC){
                            velocity_control_out_limit = 4096;//FOC control range [-4096:4096]
                        } else {
                            velocity_control_out_limit = BLDC_PWM_CONTROL_LIMIT;
                        }
                    } else if (motorcontrol_config.motor_type == BDC_MOTOR) {
                        velocity_control_out_limit = BDC_PWM_CONTROL_LIMIT;
                    }

                    if (velocity_control_config.Ki_n != 0) {
                        error_velocity_I_limit = velocity_control_out_limit * PID_DENOMINATOR / velocity_control_config.Ki_n;
                    }

                    init_filter(filter_buffer, index, FILTER_SIZE_MAX);

                    config_update_flag = 0;
                }

                if (compute_flag == 1) {
                    actual_velocity = i_motorcontrol.get_velocity_actual();
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

                    if (error_velocity_I > error_velocity_I_limit) {
                        error_velocity_I = error_velocity_I_limit;
                    } else if (error_velocity_I < -error_velocity_I_limit) {
                        error_velocity_I = 0 -error_velocity_I_limit;
                    }

                    velocity_control_out = (velocity_control_config.Kp_n*error_velocity)  +
                                           (velocity_control_config.Ki_n*error_velocity_I) +
                                           (velocity_control_config.Kd_n*error_velocity_D);

                    velocity_control_out /= PID_DENOMINATOR;

                    if (velocity_control_out > velocity_control_out_limit) {
                        velocity_control_out = velocity_control_out_limit;
                    } else if (velocity_control_out < -velocity_control_out_limit) {
                        velocity_control_out = -velocity_control_out_limit;
                    }

                    if(velocity_control_config.cascade_with_torque == 1 && motorcontrol_config.commutation_method == FOC){
                        i_motorcontrol.set_torque(velocity_control_out);  //cascades with FOC's torque controller
                    }
                    else {
                        i_motorcontrol.set_voltage(velocity_control_out); //in case of FOC sets Q value, torque controller in FOC is disabled
                    }

                    previous_error = error_velocity;
                }
                //printf("looping %d\n", velocity_control_config.Loop_time);
                break;

            case i_motorcontrol.notification():

                switch (i_motorcontrol.get_notification()) {
                    case MOTCTRL_NTF_CONFIG_CHANGED:
                        config_update_flag = 1;
                        break;
                    default:
                        break;
                }
                break;

            case i_velocity_control[int i].set_velocity(int in_velocity):

                target_velocity = in_velocity;
                break;

            case i_velocity_control[int i].get_velocity()-> int out_velocity:

                out_velocity = actual_velocity;
                break;

            case i_velocity_control[int i].get_target_velocity() -> int out_target_velocity:

                out_target_velocity = target_velocity;
                break;

            case i_velocity_control[int i].get_velocity_control_config() -> ControlConfig out_config:

                out_config = velocity_control_config;
                break;

            case i_velocity_control[int i].set_velocity_control_config(ControlConfig in_params):

                velocity_control_config = in_params;
                config_update_flag = 1;
                break;

            case i_velocity_control[int i].set_velocity_filter(int in_length):

                filter_length = in_length;

                if (filter_length > FILTER_SIZE_MAX) {
                    filter_length = FILTER_SIZE_MAX;
                }

                config_update_flag = 1;

                break;

            case i_velocity_control[int i].set_velocity_sensor(int in_sensor_used):

                velocity_control_config.feedback_sensor = in_sensor_used;
                target_velocity = actual_velocity;
                config_update_flag = 1;
                break;

            case i_velocity_control[int i].disable_velocity_ctrl():

                activate = 0;
                error_velocity = 0;
                error_velocity_D = 0;
                error_velocity_I = 0;
                previous_error = 0;
                velocity_control_out = 0;
                i_motorcontrol.set_voltage(0);
                i_motorcontrol.set_fets_state(0); //disable_motor;
                delay_milliseconds(30); //wait_ms(30, 1, t);
                break;

            case i_velocity_control[int i].check_busy() -> int out_state:

                out_state = activate;
                break;

            case i_velocity_control[int i].enable_velocity_ctrl():

                activate = 1;
                while (1) {
                    if (i_motorcontrol.check_busy() == INIT) { //__check_commutation_init(c_commutation);
#ifdef debug_print
                        printstrln("velocity_ctrl_service: commutation initialized");
#endif
                        if (i_motorcontrol.get_fets_state() == 0) { //check_fet_state(c_commutation);
                            i_motorcontrol.set_fets_state(1); //enable_motor(c_commutation);
                            delay_milliseconds(2); //wait_ms(2, 1, t);
                        }
                        break;
                    }
                }

#ifdef debug_print
                printstrln("velocity_ctrl_service: velocity control activated");
#endif
                break;

        }
    }
}
