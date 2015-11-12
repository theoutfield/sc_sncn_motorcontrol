/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.inc>
#include <IFM_DC100-rev-b.inc>

/**
 * @file app_test_ams_rotary_sensor.xc
 * @brief Test illustrates usage of the AMS rotary sensor to get position, velocity, and electrical angle information
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <rotary_sensor.h>

#define NUM_OF_AMS_INTERFACES 1

//DC1K
//on tile[IFM_TILE]: sensor_spi_interface pRotarySensor =
//{
//        {
//            XS1_CLKBLK_2,
//            XS1_CLKBLK_4,
//            XS1_PORT_1J, //D3,    //mosi
//            XS1_PORT_1I, //D1,    //sclk
//            XS1_PORT_1E  //D2     //miso
//        },
//
//        XS1_PORT_1F //D0         //slave select
//};

//DC100
on tile[IFM_TILE]: sensor_spi_interface pRotarySensor =
{
        {
            XS1_CLKBLK_2,
            XS1_CLKBLK_4,
            XS1_PORT_1B, //D3,    //mosi
            XS1_PORT_1E, //D1,    //sclk
            XS1_PORT_1J  //D2     //miso
        },

        XS1_PORT_1A //D0         //slave select
};


#define AMS_INIT_SETTINGS1  1//5    // Factory Setting 1
                                // NOISESET 0
                                // DIR      0   (CW)
                                // UVW_ABI  0
                                // DAECDIS  0
                                // ABIBIN   0
                                // Dataselect 0
                                // PWMon    0

#define AMS_INIT_SETTINGS2  4    //UVWPP     001 (5)
                                //HYS       0
                                //ABIRES    0

/* Test Hall Sensor Client */
void ams_rotary_sensor_test(client interface AMS iAMS)
{
    int position = 0;
    int velocity = 0;
    int direction = 0;

    while(1)
    {
        /* get position from Hall Sensor */
        {position, direction} = iAMS.get_absolute_position_multiturn();

        /* get velocity from Hall Sensor */
        velocity = iAMS.get_velocity();

    }
}



int main(void)
{
    interface AMS iAMS[NUM_OF_AMS_INTERFACES];

    par
    {
        on tile[APP_TILE]:
        {
            /* Test Hall Sensor Client */
            par
            {
                ams_rotary_sensor_test(iAMS[0]);
            }
        }

        /************************************************************
         * IFM_TILE
         ************************************************************/
        on tile[IFM_TILE]:
        {
            /* AMS Rotary Sensor Server */
            run_ams_sensor(iAMS, NUM_OF_AMS_INTERFACES, ROTARY_SENSOR_MAX_ANGLE, pRotarySensor, AMS_INIT_SETTINGS1, AMS_INIT_SETTINGS2, 2555);//MOT 3 R3

        }

    }

    return 0;
}
