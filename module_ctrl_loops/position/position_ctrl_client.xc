/**
 * @file  position_ctrl_client.xc
 * @brief Position control Loop Client functions
 * @author Synapticon GmbH <support@synapticon.com>
*/

#include <position_ctrl_client.h>
#include <position_ctrl_common.h>
#include <print.h>
#include <statemachine.h>
#include <drive_modes.h>
#include <internal_config.h>
#include <refclk.h>
#include <hall_server.h>

/* TODO: remove dependency (at the moment required for PID params) */
#include <bldc_motor_config.h>

//#define debug_print

#define POSITION_CTRL_WRITE(x)  c_position_ctrl <: (x)
#define POSITION_CTRL_READ(x)   c_position_ctrl :> (x)

void init_position_control_param(ctrl_par &position_ctrl_params)
{
    position_ctrl_params.Kp_n = POSITION_Kp_NUMERATOR;
    position_ctrl_params.Kp_d = POSITION_Kp_DENOMINATOR;
    position_ctrl_params.Ki_n = POSITION_Ki_NUMERATOR;
    position_ctrl_params.Ki_d = POSITION_Ki_DENOMINATOR;
    position_ctrl_params.Kd_n = POSITION_Kd_NUMERATOR;
    position_ctrl_params.Kd_d = POSITION_Kd_DENOMINATOR;
    position_ctrl_params.Loop_time = 1 * MSEC_STD; // units - for CORE 2/1/0 only default

    position_ctrl_params.Control_limit = BLDC_PWM_CONTROL_LIMIT; // PWM resolution

    if(position_ctrl_params.Ki_n != 0) // auto calculated using control_limit
    {
        position_ctrl_params.Integral_limit = position_ctrl_params.Control_limit * (position_ctrl_params.Ki_d/position_ctrl_params.Ki_n);
    } else {
        position_ctrl_params.Integral_limit = 0;
    }

    return;
}

int init_position_control(chanend c_position_ctrl)
{
    int ctrl_state = INIT_BUSY;

    while (1) {
        ctrl_state = check_position_ctrl_state(c_position_ctrl);
        if (ctrl_state == INIT_BUSY) {
            enable_position_ctrl(c_position_ctrl);
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

//internal functions
void set_position(int target_position, chanend c_position_ctrl)
{
    POSITION_CTRL_WRITE(PCTRL_CMD_SET_POSITION);
    POSITION_CTRL_WRITE(target_position);
}


int get_position(chanend c_position_ctrl)
{
    int position;
    POSITION_CTRL_WRITE(PCTRL_CMD_GET_POSITION);
    POSITION_CTRL_READ(position);
    return position;
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

//csp mode function
void set_position_csp( csp_par & csp_params, int target_position, int position_offset,
                       int velocity_offset, int torque_offset, chanend c_position_ctrl )
{
    set_position( position_limit( (target_position + position_offset) * csp_params.base.polarity,
                                  csp_params.max_position_limit,
                                  csp_params.min_position_limit),
                  c_position_ctrl);
}


void set_position_ctrl_param(ctrl_par & position_ctrl_params, chanend c_position_ctrl)
{
    POSITION_CTRL_WRITE(PCTRL_CMD_SET_PARAMS);
    POSITION_CTRL_WRITE(position_ctrl_params.Kp_n);
    POSITION_CTRL_WRITE(position_ctrl_params.Kp_d);
    POSITION_CTRL_WRITE(position_ctrl_params.Ki_n);
    POSITION_CTRL_WRITE(position_ctrl_params.Ki_d);
    POSITION_CTRL_WRITE(position_ctrl_params.Kd_n);
    POSITION_CTRL_WRITE(position_ctrl_params.Kd_d);
    POSITION_CTRL_WRITE(position_ctrl_params.Integral_limit);
}

void set_position_ctrl_hall_param(hall_par & hall_params, chanend c_position_ctrl)
{
    c_position_ctrl <: PCTRL_CMD_SET_HALL;
    c_position_ctrl <: hall_params.pole_pairs;
}

void set_position_ctrl_qei_param(qei_par & qei_params, chanend c_position_ctrl)
{
    c_position_ctrl <: PCTRL_CMD_SET_QEI;
    c_position_ctrl <: qei_params.index;
    c_position_ctrl <: qei_params.real_counts;
    c_position_ctrl <: qei_params.max_ticks_per_turn;
}


void set_position_sensor(int sensor_used, chanend c_position_ctrl)
{
    POSITION_CTRL_WRITE(SENSOR_SELECT);
    POSITION_CTRL_WRITE(sensor_used);
}

void enable_position_ctrl(chanend c_position_ctrl)
{
    POSITION_CTRL_WRITE(PCTRL_CMD_ENABLE);
    POSITION_CTRL_WRITE(1);
}

void shutdown_position_ctrl(chanend c_position_ctrl)
{
    POSITION_CTRL_WRITE(PCTRL_CMD_SHUTDOWN);
    POSITION_CTRL_WRITE(0);
}

int check_position_ctrl_state(chanend c_position_ctrl)
{
    int state;
    POSITION_CTRL_WRITE(PCTRL_CMD_GET_STATUS);
    POSITION_CTRL_READ(state);
    return state;
}
