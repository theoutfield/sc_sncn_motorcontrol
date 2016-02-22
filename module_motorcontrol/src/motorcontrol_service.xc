/**
 * @file comm_loop_server.xc
 * @brief Commutation Loop based on sinusoidal commutation method
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <xs1.h>
#include <print.h>

#include <motorcontrol_service.h>
#include <bldc_motorcontrol.h>
#include <bdc_motorcontrol.h>

int check_motorcontrol_config(MotorcontrolConfig &commutation_params)
{
    if(commutation_params.motor_type != BLDC_MOTOR && commutation_params.motor_type != BDC_MOTOR ){
        printstrln("motorcontrol_service: ERROR: Wrong configuration: motor type");
        return ERROR;
    }

    if(commutation_params.motor_type == BLDC_MOTOR){
        if(commutation_params.bldc_winding_type < 0 || commutation_params.bldc_winding_type > 2){
            printstrln("motorcontrol_service: ERROR: Wrong configuration: wrong winding");
            return ERROR;
        }

        if(commutation_params.commutation_sensor != HALL_SENSOR && commutation_params.commutation_sensor != BISS_SENSOR){
            printstrln("motorcontrol_service: ERROR: Wrong configuration: just HALL and BiSS sensors are supported as commutation sensor");
            return ERROR;
        }
    }

    return SUCCESS;
}

[[combinable]]
void motorcontrol_service(FetDriverPorts &fet_driver_ports, MotorcontrolConfig &motorcontrol_config,
                            chanend c_pwm_ctrl,
                            interface HallInterface client ?i_hall,
                            interface QEIInterface client ?i_qei,
                            interface BISSInterface client ?i_biss,
                            interface WatchdogInterface client i_watchdog,
                            interface MotorcontrolInterface server i_motorcontrol[4])
{
    //Set freq to 250MHz (always needed for proper timing)
    write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz

    HallConfig hall_config;
    QEIConfig qei_config;
    timer t;
    unsigned ts = 0;

    if(!isnull(i_hall))
        hall_config = i_hall.get_hall_config();

    if(!isnull(i_qei)){
        qei_config = i_qei.get_qei_config();
    }

    if (check_motorcontrol_config(motorcontrol_config) == ERROR){
        return;
    }

    printstr(">>   SOMANET MOTORCONTROL SERVICE STARTING...\n");

    //This while + select is just to make it combinable
    while(1){

        select{

            case t when timerafter(ts+0) :> void:

                    if(motorcontrol_config.motor_type == BLDC_MOTOR){

                        bldc_loop(hall_config, qei_config, i_hall, i_qei, i_biss, i_watchdog, i_motorcontrol,
                                c_pwm_ctrl, fet_driver_ports, motorcontrol_config);

                    }else if(motorcontrol_config.motor_type == BDC_MOTOR){

                        bdc_loop(c_pwm_ctrl, i_watchdog, i_motorcontrol, fet_driver_ports, motorcontrol_config);
                    }

                    break;
        }
    }
}


