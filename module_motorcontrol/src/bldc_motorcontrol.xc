/**
 * @file comm_loop_server.xc
 * @brief Commutation Loop based on sinusoidal commutation method
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <bldc_motorcontrol.h>
#include <pwm_service_client.h>
#include <a4935.h>
#include <sine_table_big.h>

static void commutation_init_to_zero(chanend c_pwm_ctrl, t_pwm_control & pwm_ctrl)
{
    unsigned int pwm[3] = {0, 0, 0};  // PWM OFF (break mode; short all phases)
    pwm_share_control_buffer_address_with_server(c_pwm_ctrl, pwm_ctrl);
    update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);
}

[[combinable]]
void bldc_loop(HallConfig hall_config, QEIConfig qei_config,
                            interface HallInterface client ?i_hall,
                            interface QEIInterface client ?i_qei,
                            interface BISSInterface client ?i_biss,
                            interface WatchdogInterface client i_watchdog,
                            interface MotorcontrolInterface server i_motorcontrol[4],
                            chanend c_pwm_ctrl,
                            FetDriverPorts &fet_driver_ports,
                            MotorcontrolConfig &motorcontrol_config)
{
    const unsigned t_delay = 300*USEC_FAST;
    timer t;
    unsigned int ts;
    t_pwm_control pwm_ctrl;
    int check_fet;
    int calib_flag = 0;
    int init_state = INIT_BUSY;

    unsigned int pwm[3] = { 0, 0, 0 };
    int angle_pwm = 0;
    int angle = 0;
    int voltage = 0;
    int pwm_half = PWM_MAX_VALUE>>1;
    int max_count_per_hall, angle_offset;

    if (!isnull(i_hall)) {
        if(!isnull(i_qei))
            max_count_per_hall = qei_config.ticks_resolution * QEI_CHANGES_PER_TICK /hall_config.pole_pairs;
        angle_offset = (4096 / 6) / (2 * hall_config.pole_pairs);
    }

    int fw_flag = 0;
    int bw_flag = 0;

    int shutdown = 0; //Disable FETS
    int sensor_select = motorcontrol_config.commutation_sensor;

    int notification = MOTCTRL_NTF_EMPTY;

    commutation_init_to_zero(c_pwm_ctrl, pwm_ctrl);

    // enable watchdog
    t :> ts;
    t when timerafter (ts + 250000*4):> ts; /* FIXME: replace with constant */
    i_watchdog.start();

    t :> ts;
    t when timerafter (ts + t_delay) :> ts;

    if (!isnull(fet_driver_ports.p_esf_rst_pwml_pwmh) && !isnull(fet_driver_ports.p_coast)){
        a4935_initialize(fet_driver_ports.p_esf_rst_pwml_pwmh, fet_driver_ports.p_coast, A4935_BIT_PWML | A4935_BIT_PWMH);
        t when timerafter (ts + t_delay) :> ts;
    }

    if(!isnull(fet_driver_ports.p_coast)){
        fet_driver_ports.p_coast :> check_fet;
        init_state = check_fet;
    }
    else {
        init_state = 1;
    }

    while (1) {

  //      t_loop :> start_time;

        select {

            case t when timerafter(ts + USEC_FAST * motorcontrol_config.commutation_loop_period) :> ts: //XX kHz commutation loop
                if (calib_flag != 0) {
                    angle = 0;
                } else if (sensor_select == HALL_SENSOR) {
                    //hall only
                    angle = i_hall.get_hall_position();
                } else if (sensor_select == QEI_SENSOR && !isnull(i_qei)) {
                    { angle, fw_flag, bw_flag } = i_qei.get_qei_sync_position();
                    angle = (angle << 12) / max_count_per_hall;
                    if ((voltage >= 0 && fw_flag == 0) || (voltage < 0 && bw_flag == 0)) {
                        angle = i_hall.get_hall_position();
                    }
                } else if (sensor_select == BISS_SENSOR) {
                    angle = i_biss.get_biss_angle();
                }
                if (motorcontrol_config.polarity_type == INVERTED_POLARITY)
                    angle = 4096 - angle;

                if (shutdown == 1) {    /* stop PWM */
                    pwm[0] = -1;
                    pwm[1] = -1;
                    pwm[2] = -1;
                } else {
                    if (voltage >= 0) {
                        if (sensor_select == QEI_SENSOR ) {
                            angle_pwm = (angle >> 2) & 0x3ff; //512
                        } else {
                            angle_pwm = ((angle + motorcontrol_config.hall_offset[0]) >> 2) & 0x3ff;
                        }
                        pwm[0] = ((sine_third_expanded(angle_pwm)) * voltage) / pwm_half + pwm_half; // 6944 -- 6867range
                        angle_pwm = (angle_pwm + 341) & 0x3ff; /* +120 degrees (sine LUT size divided by 3) */
                        pwm[1] = ((sine_third_expanded(angle_pwm)) * voltage) / pwm_half + pwm_half;
                        angle_pwm = (angle_pwm + 342) & 0x3ff;
                        pwm[2] = ((sine_third_expanded(angle_pwm)) * voltage) / pwm_half + pwm_half;
                    } else { /* voltage < 0 */
                        if (sensor_select == QEI_SENSOR ) {
                            angle_pwm = (angle >> 2) & 0x3ff; //3100
                        } else {
                            angle_pwm = ((angle + motorcontrol_config.hall_offset[1]) >> 2) & 0x3ff;
                        }
                        pwm[0] = ((sine_third_expanded(angle_pwm)) * -voltage) / pwm_half + pwm_half;
                        angle_pwm = (angle_pwm + 341) & 0x3ff;
                        pwm[1] = ((sine_third_expanded(angle_pwm)) * -voltage) / pwm_half + pwm_half;
                        angle_pwm = (angle_pwm + 342) & 0x3ff;
                        pwm[2] = ((sine_third_expanded(angle_pwm)) * -voltage) / pwm_half + pwm_half;
                    }
                }

                /* Limiting PWM values (and suppression of short pulses) is done in
                 * update_pwm_inv() */
                update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);
                break;

            case i_motorcontrol[int i].get_notification() -> int out_notification:

                out_notification = notification;
                break;

            case i_motorcontrol[int i].set_voltage(int new_voltage):
                    if (motorcontrol_config.bldc_winding_type == DELTA_WINDING)
                        voltage = -new_voltage;
                    else
                        voltage = new_voltage;
                    break;

            case i_motorcontrol[int i].set_config(MotorcontrolConfig new_parameters):
                    motorcontrol_config = new_parameters;

                    notification = MOTCTRL_NTF_CONFIG_CHANGED;
                    // TODO: Use a constant for the number of interfaces
                    for (int i = 0; i < 4; i++) {
                        i_motorcontrol[i].notification();
                    }

                    sensor_select = motorcontrol_config.commutation_sensor;
                    break;

            case i_motorcontrol[int i].get_config() -> MotorcontrolConfig out_config:

                    out_config = motorcontrol_config;
                    break;

            case i_motorcontrol[int i].set_sensor(int new_sensor):
                    sensor_select = new_sensor;
                    motorcontrol_config.commutation_sensor = sensor_select;
                    break;

            case i_motorcontrol[int i].set_sensor_offset(int in_offset):
                    if (sensor_select == BISS_SENSOR ) {
                        BISSConfig out_biss_config = i_biss.get_biss_config();
                        out_biss_config.offset_electrical = in_offset;
                        i_biss.set_biss_config(out_biss_config);
                    }
                    break;

            case i_motorcontrol[int i].set_fets_state(int new_state):

                    if(new_state == 0){
                        shutdown = 1;
                    }else{
                        shutdown = 0;
                        voltage = 0;
                    }

                    break;

            case i_motorcontrol[int i].get_fets_state() -> int fets_state:
                    fets_state = !shutdown;
                    break;

            case i_motorcontrol[int i].check_busy() -> int state_return:
                    state_return = init_state;
                    break;

            case i_motorcontrol[int i].set_calib(int in_flag) -> int out_offset:
                    calib_flag = in_flag;
                    if (calib_flag == 1) {
                        motorcontrol_config.hall_offset[0] = 0;
                        motorcontrol_config.hall_offset[1] = 2048;
                    } else {
                        int calib_angle;
                        if (motorcontrol_config.bldc_winding_type == STAR_WINDING)
                            calib_angle = 1024;
                        else
                            calib_angle = 3072;
                        if (sensor_select == HALL_SENSOR) {
                            out_offset = (1024 - i_hall.get_hall_position()) & 4095;
                            //Hall has a low resolution so the offsets could need to be shifted by +/- 1/6 turn
                            if (motorcontrol_config.polarity_type == INVERTED_POLARITY) {
                                if (motorcontrol_config.bldc_winding_type == STAR_WINDING) {
                                    motorcontrol_config.hall_offset[0] = (out_offset - 682) & 4095; // -1/6 turn
                                    motorcontrol_config.hall_offset[1] = (out_offset + 682) & 4095; // + half turn - 1/6 turn
                                } else {
                                    motorcontrol_config.hall_offset[1] = (out_offset - 682) & 4095;
                                    motorcontrol_config.hall_offset[0] = (out_offset + 682) & 4095;
                                }
                            } else {
                                if (motorcontrol_config.bldc_winding_type == STAR_WINDING) {
                                    motorcontrol_config.hall_offset[0] = out_offset;
                                    motorcontrol_config.hall_offset[1] = (out_offset + 2731) & 4095; // + half turn + 1/6 turn
                                } else {
                                    motorcontrol_config.hall_offset[1] = out_offset;
                                    motorcontrol_config.hall_offset[0] = (out_offset + 2731) & 4095;
                                }
                            }
                        } else if (sensor_select == BISS_SENSOR) {
                            out_offset = i_biss.reset_biss_angle_electrical(calib_angle);// quarter turn
                        }
                    }
                    break;

            case i_motorcontrol[int i].set_all_parameters(HallConfig in_hall_config,
                                                                QEIConfig in_qei_config,
                                                                MotorcontrolConfig in_commutation_config):

                 //hall_config.pole_pairs = in_hall_config.pole_pairs;
                 qei_config.index_type = in_qei_config.index_type;
                 //qei_config.max_ticks_per_turn = in_qei_config.max_ticks_per_turn;
                 qei_config.ticks_resolution = in_qei_config.ticks_resolution;

                 motorcontrol_config.hall_offset[0] = in_commutation_config.hall_offset[0];
                 motorcontrol_config.hall_offset[1] = in_commutation_config.hall_offset[1];
                 motorcontrol_config.bldc_winding_type = in_commutation_config.bldc_winding_type;
                 //motorcontrol_config.angle_variance = (60 * 4096) / (hall_config.pole_pairs * 2 * 360);

                // if (hall_config.pole_pairs < 4) {
                //      motorcontrol_config.nominal_speed = nominal_speed * 4;
                //  } else if (hall_config.pole_pairs >= 4) {
                //      motorcontrol_config.nominal_speed = nominal_speed;
                //  }

                  voltage = 0;
                  if (!isnull(i_hall)) {
                      if(!isnull(i_qei))
                          max_count_per_hall = qei_config.ticks_resolution  * QEI_CHANGES_PER_TICK / hall_config.pole_pairs;
                      angle_offset = (4096 / 6) / (2 * hall_config.pole_pairs);
                  }
                  fw_flag = 0;
                  bw_flag = 0;

                    break;

            }

 //       t_loop :> end_time;
 //       printf("%i kHz\n", 250000/(end_time - start_time));
    }

}
