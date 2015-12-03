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

/*
void gpio_config_handler(chanend c_gpio, port gpio_ports[], int port_function[],
                         int active_states[], int input_types[], int &all_ports_input)
{
    int input_type;
    int switch_type;
    int port_number;
    int i;

    c_gpio :> port_number;
    c_gpio :> input_type;
    c_gpio :> switch_type;

    if (port_number < NUMBER_OF_IO_PORTS) {
        port_function[port_number] = INPUT_PORT;
        if (input_type == SWITCH_INPUT_TYPE) {
            if (switch_type == ACTIVE_HIGH) {
                set_port_pull_down(gpio_ports[port_number]);
                active_states[port_number] = 1;
            } else if (switch_type == ACTIVE_LOW) {
                active_states[port_number] = 0;
                // external pull ups required for active low digital input
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
                for(i = 0; i < NUMBER_OF_IO_PORTS; i++) {
                    set_port_pull_down(gpio_ports[i]);
                    port_function[i] = INPUT_PORT;
                    input_types[i] = input_type;
                    active_states[i] = 1;
                }
            } else if (switch_type == ACTIVE_LOW) {
                for (i = 0; i < NUMBER_OF_IO_PORTS; i++) {
                    port_function[i] = INPUT_PORT;
                    input_types[i] = input_type;
                    active_states[i] = 0;
                }
                // external pull ups required for active low digital input
            }
        } else if (input_type == GP_INPUT_TYPE) {
            for (i = 0; i < NUMBER_OF_IO_PORTS; i++) {
                port_function[i] = INPUT_PORT;
                input_types[i] = input_type;
                active_states[i] = 0; // invalid
            }
        }
    }
}*/
/*
void gpio_client_handler(chanend c_gpio, port gpio_ports[],
                         int port_number, int type, int input_types[], int port_switch_states[], int input, int output, int i)
{
    if (port_number < NUMBER_OF_IO_PORTS) {
        c_gpio :> type;
        if (type == GPIO_INPUT) {
            if (input_types[port_number] == SWITCH_INPUT_TYPE) {
                c_gpio <: port_switch_states[port_number];
            } else {
                gpio_ports[port_number] :> input;
                c_gpio <: input;
            }
        } else if (type == GPIO_OUTPUT) {
            c_gpio :> output;
            gpio_ports[port_number] <: output;
        }
    } else if (port_number == NUMBER_OF_IO_PORTS) {
        c_gpio :> type;
        if (type == GPIO_INPUT) {
            for (i = 0 ; i < NUMBER_OF_IO_PORTS; i++) {
                if (input_types[i] == SWITCH_INPUT_TYPE) {
                    c_gpio <: port_switch_states[i];
                } else {
                    if (i < NUMBER_OF_IO_PORTS) {
                        gpio_ports[i] :> input;

                        c_gpio <: input;
                    }
                }
            }
        } else if(type == GPIO_OUTPUT) {
            c_gpio :> output;
            for(i = 0 ; i < NUMBER_OF_IO_PORTS; i++) {
                gpio_ports[i] <: output;
            }
        }
    }
}
*/

void gpio_service(port gpio_ports[4], interface GPIOInterface server i_gpio[2])
{
    int port_state[NUMBER_OF_IO_PORTS];
    int port_function[NUMBER_OF_IO_PORTS];
    int port_switch_states[NUMBER_OF_IO_PORTS];
    int active_states[NUMBER_OF_IO_PORTS];
    int input_types[NUMBER_OF_IO_PORTS];
    int all_ports_input = 0;
    timer t;
    unsigned int time;
    int command;
    int type;
    int new_pin;
    int input;
    int output;
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
        case i_gpio[int i].config_dio_input(int port_number, int input_type, int switch_type) -> int out_config_state:

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
/*
    while(1) {
        select {
        case c_gpio_0 :> command:
            switch (command) {
            case CONFIG_DIO_INPUT:
                gpio_config_handler(c_gpio_0, p_ifm_ext_d, port_function, active_states, input_types, all_ports_input);
                break;

            case CONFIG_DIO_DONE:
                config_state = 1;
                break;

            default:
                break;
            }
            break;

        case c_gpio_1 :> command:
            switch (command) {
            case CONFIG_DIO_INPUT:
                gpio_config_handler(c_gpio_1, p_ifm_ext_d, port_function, active_states, input_types, all_ports_input);
                break;

            case CONFIG_DIO_DONE:
                config_state = 1;
                break;

            default:
                break;
            }
            break;
        }

        if(config_state == 1) {
            break;
        }
    }

    for (i = 0; i < NUMBER_OF_IO_PORTS; i++) {
        if (port_function[i] == INPUT_PORT) {
            if (input_types[i] == SWITCH_INPUT_TYPE) {
                p_ifm_ext_d[i] :> port_state[i];
                if (port_state[i] == active_states[i]) {
                    port_switch_states[i] = 1;
                } else {
                    port_switch_states[i] = 0;
                }
            }
        }
    }
*//*
    while(1) {
        //      for(i = 0 ; i< NUMBER_OF_IO_PORTS; i++)
        //              xscope_probe_data(i, port_switch_states[i]);
#pragma ordered
        select {

        case c_gpio_0 :> command:
            gpio_client_handler(c_gpio_0, p_ifm_ext_d,
                                command, type, input_types, port_switch_states, input, output, i);
            break;

        case c_gpio_1 :> command:
            gpio_client_handler(c_gpio_1, p_ifm_ext_d,
                                command, type, input_types, port_switch_states, input, output, i);
            break;

        }*/

}
