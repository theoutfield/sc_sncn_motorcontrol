#include <xs1.h>
#include <internal_config.h>
#include <platform.h>

int config_gpio_digital_input(chanend c_gpio, int port_number, int input_type, int switch_type);

int end_config_gpio(chanend c_gpio);

int read_gpio_digital_input(chanend c_gpio, int port_number);

void write_gpio_digital_output(chanend c_gpio, int port_number, int port_value);

