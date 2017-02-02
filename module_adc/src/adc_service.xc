#include <xs1.h>
#include <adc_service.h>
#include <adc_7265.h>
#include <adc_ad7949.h>

void adc_service(ADCPorts &adc_ports, interface ADCInterface server i_adc[2], interface WatchdogInterface client ?i_watchdog, int ifm_tile_usec, int operational_mode)
{

    if(ifm_tile_usec==250)
    {
        //Set freq to 250MHz (always needed for proper timing)
        write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz
    }

    if(!isnull(adc_ports.ad7949_ports.clk))
    { // Check which ADC is configured
        if (operational_mode==NORMAL_MODE)
            adc_ad7949(i_adc, adc_ports.ad7949_ports, adc_ports.current_sensor_config, i_watchdog);
        else if (operational_mode==STD_MOTOR_CTRL_MODE)
            adc_ad7949_fixed_channel(i_adc, adc_ports.ad7949_ports, adc_ports.current_sensor_config, i_watchdog);
    }
    else if(!isnull(adc_ports.ad7265_ports.xclk))
    {
        if (operational_mode==NORMAL_MODE)
            adc_ad7256(i_adc, adc_ports.ad7265_ports, adc_ports.current_sensor_config, i_watchdog);
        else if(operational_mode==STD_MOTOR_CTRL_MODE)
            adc_ad7256_fixed_channel(i_adc, adc_ports.ad7265_ports, adc_ports.current_sensor_config, i_watchdog);
    }
}
