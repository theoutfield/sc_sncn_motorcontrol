/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
//#include <CORE_C22-rev-a.inc>
#include <CORE_C21-rev-a.inc>
//#include <IFM_DC1K-rev-c2.inc>
#include <IFM_DC100-rev-b.inc>

/**
 * @file app_test_ams_rotary_sensor.xc
 * @brief Test illustrates usage of the AMS rotary sensor to get position, velocity, and electrical angle information
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <rotary_sensor.h>
#include <xscope.h>
#include <bldc_motor_config.h>

#define NUM_OF_AMS_INTERFACES 1

//DC1K
on tile[IFM_TILE]: sensor_spi_interface pRotarySensor =
{
        {
            XS1_CLKBLK_2,
            XS1_CLKBLK_4,
            GPIO_D3, //D3,    //mosi
            GPIO_D1, //D1,    //sclk
            GPIO_D2  //D2     //miso
        },

        GPIO_D0 //D0         //slave select
};





/* Test Hall Sensor Client */
void ams_rotary_sensor_test(client interface AMS iAMS)
{

    int position = 0;
    int velocity = 0;
    int direction = 0;
    int electrical_angle = 0;

    while(1)
    {

        electrical_angle = iAMS.get_angle_electrical();

        /* get position from Hall Sensor */
        {position, direction} = iAMS.get_absolute_position_multiturn();
    //    position = iAMS.get_absolute_position_singleturn();

        /* get velocity from Hall Sensor */
   //     velocity = iAMS.get_velocity();

        xscope_int(POSITION, position);
        xscope_int(ANGLE, electrical_angle);
   //     printf("%i\n", position);

    }
}

void ams_rotary_sensor_direct_method(sensor_spi_interface &sensor_if, unsigned short settings1, unsigned short settings2, unsigned short offset){
    int result = initRotarySensor(sensor_if,  settings1,  settings2, offset);
    int abs_position = 0;

    printf("result init: %d\n",result);
    printf("pole pairs init: %d\n", readNumberPolePairs(sensor_if));

    while(1){
        abs_position = readRotarySensorAngleWithoutCompensation(sensor_if);
        xscope_int(POSITION, abs_position);
        printf("%i\n", abs_position);
    }


}




int main(void)
{
    interface AMS iAMS[NUM_OF_AMS_INTERFACES];

    par
    {
        on tile[APP_TILE]:
        {
            iAMS[0].configure(set_configuration());
            ams_rotary_sensor_test(iAMS[0]);

        }

        /************************************************************
         * IFM_TILE
         ************************************************************/
        on tile[IFM_TILE]:
        {
//            p_ifm_motor_hi[0] <: 0;
//            p_ifm_motor_hi[1] <: 0;
//            p_ifm_motor_hi[2] <: 0;
//
//            p_ifm_motor_lo[0] <: 0;
//            p_ifm_motor_lo[1] <: 0;
//            p_ifm_motor_lo[2] <: 0;
//
//            p_ifm_motor_hi[3] <: 0;
//            p_ifm_motor_lo[3] <: 0;

            /* AMS Rotary Sensor Server */

            ams_sensor_server(iAMS, NUM_OF_AMS_INTERFACES, pRotarySensor);

  //          ams_rotary_sensor_direct_method(pRotarySensor, AMS_INIT_SETTINGS1, AMS_INIT_SETTINGS2, 2555);


        }

    }

    return 0;
}
