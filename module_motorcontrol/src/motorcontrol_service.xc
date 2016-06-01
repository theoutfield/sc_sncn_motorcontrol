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
#include <stdlib.h>

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

        //FIXME check commutation sensor
//        if(commutation_params.commutation_sensor != HALL_SENSOR &&
//           commutation_params.commutation_sensor != BISS_SENSOR &&
//           commutation_params.commutation_sensor != AMS_SENSOR) {
//            printstrln("motorcontrol_service: ERROR: Wrong configuration: just HALL, BiSS and AMS sensors are supported as commutation sensor");
//            return ERROR;
//        }
    }

    return SUCCESS;
}

//[[combinable]]
void motorcontrol_service(FetDriverPorts &fet_driver_ports, MotorcontrolConfig &motorcontrol_config,
                            chanend c_pwm_ctrl,
                            interface ADCInterface client ?i_adc,
                            client interface shared_memory_interface ?i_shared_memory,
                            interface WatchdogInterface client i_watchdog,
                            interface BrakeInterface client ?i_brake,
                            interface MotorcontrolInterface server i_motorcontrol[4])
{
    //Set freq to 250MHz (always needed for proper timing)
    write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz

    timer t;
    unsigned ts = 0;

    if (check_motorcontrol_config(motorcontrol_config) == ERROR){
        return;
    }

    printstr(">>   SOMANET MOTORCONTROL SERVICE STARTING...\n");

    //This while + select is just to make it combinable
    while(1){

        select{

            case t when timerafter(ts+0) :> void:

                    if(motorcontrol_config.motor_type == BLDC_MOTOR){

                        if(!isnull(i_adc)){

                            bldc_loop( fet_driver_ports, motorcontrol_config,
                                    i_motorcontrol, c_pwm_ctrl, i_adc,
                                    i_shared_memory, i_watchdog, i_brake);

                        }
                        else{
                            printstr(">! ADC interface is not provided to the motorcontrol service...\n");
                            printstr(">! stopping the service...\n");
                            exit(-1);
                        }


                    }else if(motorcontrol_config.motor_type == BDC_MOTOR){

                        bdc_loop(c_pwm_ctrl, i_watchdog, i_motorcontrol, fet_driver_ports, motorcontrol_config);
                    }

                    break;
        }
    }
}


