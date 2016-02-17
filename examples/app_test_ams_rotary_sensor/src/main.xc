/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC300-rev-a.bsp>
//#include <IFM_DC1K-rev-c1.bsp>

/**
 * @file app_test_ams_rotary_sensor.xc
 * @brief Test illustrates usage of the AMS rotary sensor to get position, velocity, and electrical angle information
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <ams_service.h>
#include <ctype.h>

AMSPorts ams_ports = { {
        IFM_TILE_CLOCK_2,
        IFM_TILE_CLOCK_3,
        SOMANET_IFM_GPIO_D3, //D3,    //mosi
        SOMANET_IFM_GPIO_D1, //D1,    //sclk
        SOMANET_IFM_GPIO_D2  },//D2     //miso
        SOMANET_IFM_GPIO_D0 //D0         //slave select
};


/* Test AMS Sensor Client */
void ams_rotary_sensor_test(client interface AMSInterface i_ams)
{
    int count = 0;
    int velocity = 0;
    int position = 0;
    int electrical_angle = 0;

    while(1) {
        /* get position from AMS Sensor */
        {count, position} = i_ams.get_ams_position();
        electrical_angle = i_ams.get_ams_angle();

        /* get velocity from AMS Sensor */
        velocity = i_ams.get_ams_velocity();

        xscope_int(COUNT, count);
        xscope_int(POSITION, position);
        xscope_int(ANGLE, electrical_angle);
        xscope_int(VELOCITY, velocity);

        delay_milliseconds(1);
    }
}


int main(void)
{
    interface AMSInterface i_ams[5];

    par
    {
        on tile[APP_TILE]: ams_rotary_sensor_test(i_ams[0]);

        /************************************************************
         * IFM_TILE
         ************************************************************/
        on tile[IFM_TILE]:
            {
                /* AMS Rotary Sensor Service */
                AMSConfig ams_config;
                ams_config.factory_settings = 1;
                ams_config.polarity = AMS_POLARITY;
                ams_config.hysteresis = 1;
                ams_config.noise_setting = AMS_NOISE_NORMAL;
                ams_config.uvw_abi = 0;
                ams_config.dyn_angle_comp = 0;
                ams_config.data_select = 0;
                ams_config.pwm_on = AMS_PWM_OFF;
                ams_config.abi_resolution = 0;
                ams_config.resolution_bits = AMS_RESOLUTION;
                ams_config.offset = AMS_OFFSET;
                ams_config.max_ticks = 0x7fffffff;
                ams_config.pole_pairs = 3;
                ams_config.cache_time = AMS_CACHE_TIME;
                ams_config.velocity_loop = AMS_VELOCITY_LOOP;

                ams_service(ams_ports, ams_config, i_ams);
            }
    }

    return 0;
}
