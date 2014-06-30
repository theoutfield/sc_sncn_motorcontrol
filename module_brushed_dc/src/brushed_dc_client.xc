/**
 * @file
 * @brief Brushed Motor Drive Client function
 * @author Pavan Kanajar <pkanajar@synapticon.com>
 * @author Martin Schwarz <mschwarz@synapticon.com>
*/

#include <xs1.h>
#include <brushed_dc_common.h>

/* MAX Input value BDC_PWM_CONTROL_LIMIT */
void set_bdc_voltage(chanend c_voltage, int input_voltage)
{
    c_voltage <: BDC_CMD_SET_VOLTAGE;
    c_voltage <: input_voltage;
    return;
}
