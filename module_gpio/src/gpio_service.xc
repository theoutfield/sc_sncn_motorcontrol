/**
 * @file gpio_server.xc
 * @brief General Purpose IO Digital Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/

#include <gpio_service.h>

//input types

#define INPUT_PORT           1
#define OUTPUT_PORT          2
#define NUMBER_OF_IO_PORTS   4    /**< Defines number of Digital IOs available. */

void gpio_service(port gpio_ports[4], interface GPIOInterface server i_gpio[1])
{
    //Set freq to 250MHz (always needed for proper timing)
    write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz

    int port_state[NUMBER_OF_IO_PORTS];
    int port_function[NUMBER_OF_IO_PORTS];
    int port_switch_states[NUMBER_OF_IO_PORTS];
    int active_states[NUMBER_OF_IO_PORTS];
    int input_types[NUMBER_OF_IO_PORTS];
    int all_ports_input = 0;
    timer t;
    unsigned int time;
    int new_pin;
    int config_state = 0;
    int j;

    for(j = 0; j < NUMBER_OF_IO_PORTS; j++) {
        port_state[j] = 0;
        port_switch_states[j] = 0;
        port_function[j] = OUTPUT_PORT;         /**< By default all IO ports are configured as outputs. */
    }

    t:>time;


    while(1){

        select
        {
        case i_gpio[int i].config_dio_input(int port_number, int input_type, SwitchType switch_type) -> int out_config_state:

                if (input_type != SWITCH_INPUT_TYPE && input_type != GP_INPUT_TYPE) {
                    out_config_state = 0;
                    break;
                }

                if (port_number > 4) {
                    out_config_state = 0;
                    break;
                }

                if (port_number < NUMBER_OF_IO_PORTS) {
                    port_function[port_number] = INPUT_PORT;
                    if (input_type == SWITCH_INPUT_TYPE) {
                        if (switch_type == ACTIVE_HIGH) {
                            set_port_pull_down(gpio_ports[port_number]);
                            active_states[port_number] = 1;
                        } else if (switch_type == ACTIVE_LOW) {
                            active_states[port_number] = 0;
                            /**< external pull ups required for active low digital input */
                        }
                        input_types[port_number] = input_type;
                    } else if (input_type == GP_INPUT_TYPE) {
                        input_types[port_number] = input_type;
                        active_states[port_number] = 0; // invalid
                    }
                } else if (port_number == NUMBER_OF_IO_PORTS) {
                    all_ports_input = 1;
                    if (input_type == SWITCH_INPUT_TYPE) {
                        if (switch_type == ACTIVE_HIGH) {
                            for(j = 0; j < NUMBER_OF_IO_PORTS; j++) {
                                set_port_pull_down(gpio_ports[j]);
                                port_function[j] = INPUT_PORT;
                                input_types[j] = input_type;
                                active_states[j] = 1;
                            }
                        } else if (switch_type == ACTIVE_LOW) {
                            for (j = 0; j < NUMBER_OF_IO_PORTS; j++) {
                                port_function[j] = INPUT_PORT;
                                input_types[j] = input_type;
                                active_states[j] = 0;
                            }
                            /**< external pull ups required for active low digital input */
                        }
                    } else if (input_type == GP_INPUT_TYPE) {
                        for (j = 0; j < NUMBER_OF_IO_PORTS; j++) {
                            port_function[j] = INPUT_PORT;
                            input_types[j] = input_type;
                            active_states[j] = 0; // invalid
                        }
                    }
                }

                out_config_state = 1;


                break;

        case i_gpio[int i].config_dio_done():

                config_state = 1;

                for (j = 0; j < NUMBER_OF_IO_PORTS; j++) {
                    if (port_function[j] == INPUT_PORT) {
                        if (input_types[j] == SWITCH_INPUT_TYPE) {
                            gpio_ports[j] :> port_state[j];
                            if (port_state[j] == active_states[j]) {
                                port_switch_states[j] = 1;
                            } else {
                                port_switch_states[j] = 0;
                            }
                        }
                    }
                }

                break;

        case i_gpio[int i].read_gpio(int port_number) -> int out_value:

                if(config_state==1){

                    if (port_number < NUMBER_OF_IO_PORTS) {
                        if (input_types[port_number] == SWITCH_INPUT_TYPE) {
                            out_value = port_switch_states[port_number];
                        } else {
                            gpio_ports[port_number] :> out_value;
                        }
                    }
                }

                break;

        case i_gpio[int i].write_gpio(int port_number, int out_value):

                if(config_state==1){

                    if (port_number < NUMBER_OF_IO_PORTS) {

                        gpio_ports[port_number] <: out_value;
                    }
                }

                break;

        case gpio_ports[0] when pinsneq(port_state[0]) :> port_state[0]:
                   gpio_ports[0] :> new_pin;
                   if (new_pin ==  port_state[0]) {
                       gpio_ports[0] :> new_pin;
                       if (new_pin == port_state[0]) {
                           if (port_state[0] == active_states[0]) {
                               port_switch_states[0] = 1;
                           } else {
                               port_switch_states[0] = 0;
                           }
                       }
                   }

                   break;

           case gpio_ports[1] when pinsneq(port_state[1]) :> port_state[1]:
               gpio_ports[1] :> new_pin;
               if (new_pin ==  port_state[1]) {
                   gpio_ports[1] :> new_pin;
                   if (new_pin == port_state[1]) {
                       if (port_state[1] == active_states[1]) {
                           port_switch_states[1] = 1;
                       } else {
                           port_switch_states[1] = 0;
                       }
                   }
               }
               break;

           case gpio_ports[2] when pinsneq(port_state[2]) :> port_state[2]:
               gpio_ports[2] :> new_pin;
               if (new_pin ==  port_state[2]) {
                   gpio_ports[2] :> new_pin;
                   if (new_pin == port_state[2]) {
                       if (port_state[2] == active_states[2]) {
                           port_switch_states[2] = 1;
                       } else {
                           port_switch_states[2] = 0;
                       }
                   }
               }
               break;

           case gpio_ports[3] when pinsneq(port_state[3]) :> port_state[3]:
               gpio_ports[3] :> new_pin;
               if(new_pin ==  port_state[3]) {
                   gpio_ports[3] :> new_pin;
                   if (new_pin == port_state[3]) {
                       if(port_state[3] == active_states[3]) {
                           port_switch_states[3] = 1;
                       } else {
                           port_switch_states[3] = 0;
                       }
                   }
               }
               break;
        }
    }

}
