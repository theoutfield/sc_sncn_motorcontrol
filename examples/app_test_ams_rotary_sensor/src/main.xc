/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>

/**
 * @file app_test_ams_rotary_sensor.xc
 * @brief Test illustrates usage of the AMS rotary sensor to get position, velocity, and electrical angle information
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <ams_service.h>
#include <ctype.h>


AMSPorts ams_ports = SOMANET_IFM_AMS_PORTS;

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

        /* get angle and velocity from AMS Sensor */
        { electrical_angle, velocity, void } = i_ams.get_ams_angle_velocity_position();

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
                ams_config.pole_pairs = 5;
                ams_config.cache_time = AMS_CACHE_TIME;
                ams_config.velocity_loop = AMS_VELOCITY_LOOP;

                ams_service(ams_ports, ams_config, i_ams);
            }
    }

    return 0;
}
