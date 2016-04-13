/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
//#include <CORE_BOARD_REQUIRED>
//#include <IFM_BOARD_REQUIRED>
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC1K-rev-c3.bsp>

/**
 * @file test_biss.xc
 * @brief Test illustrates usage of biss encoder to get position and velocity information
 * @author Synapticon GmbH <support@synapticon.com>
 */
//BiSS libs
#include <biss_service.h>
#include <watchdog_service.h>
#include <pwm_service.h>
#include <adc_service.h>

/* Test BiSS Encoder Client */
void biss_test(client interface BISSInterface i_biss, client interface ADCInterface i_adc) {
    timer t;
    unsigned int start_time, end_time;
    int count = 0;
    int real_count = 0;
    int velocity = 0;
    unsigned int position = 0;
    unsigned int status = 0;
    int adc_a = 0, adc_b = 0;

    while(1) {

        /* get position from BiSS Encoder */
        { count, position, void } = i_biss.get_biss_position();
        t :> start_time;
        { real_count, void, status } = i_biss.get_biss_real_position();
        t :> end_time;

        /* get velocity from BiSS Encoder */
        velocity = i_biss.get_biss_velocity();

        //get adc
        { adc_a, adc_b } = i_adc.get_external_inputs();

        xscope_int(COUNT, count);                           //absolute count
        xscope_int(REAL_COUNT, real_count);                 //real internal absolute count
        xscope_int(POSITION, position);                     //singleturn position
        xscope_int(VELOCITY, velocity);                     //velocity in rpm
        xscope_int(ERROR_BIT, (status&0b10) * 500);         //error bit, should be 0
        xscope_int(WARNING_BIT, (status&0b01) * 1000);      //warning bit, should be 0
        xscope_int(TIME, (end_time-start_time)/USEC_STD);   //time to get the data in microseconds

        xscope_int(ADC_A, adc_a);
        xscope_int(ADC_B, adc_b);
        xscope_int(ADC_AB, (adc_b-adc_a)/4);

        delay_milliseconds(1);
    }
}

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
ADCPorts adc_ports = SOMANET_IFM_ADC_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
BISSPorts biss_ports = SOMANET_IFM_BISS_PORTS;

int main() {
    chan c_pwm_ctrl, c_adctrig; // pwm channels
    interface ADCInterface i_adc[2];
    interface WatchdogInterface i_watchdog[2];
    interface BISSInterface i_biss[5]; //array of interfaces for biss server
    interface BrakeInterface i_brake;

    par {
        /* Test BiSS Encoder Client */
        on tile[APP_TILE]: biss_test(i_biss[0], i_adc[0]);


        /************************************************************
         * IFM_TILE
         ************************************************************/
        on tile[IFM_TILE]: par {
            /* Triggered PWM Service */
            pwm_triggered_service( pwm_ports, c_adctrig, c_pwm_ctrl, i_brake);
            i_brake.set_brake(1);

            /* ADC Service */
            adc_service(adc_ports, c_adctrig, i_adc, i_watchdog[1]);

            /* Watchdog Service */
            watchdog_service(wd_ports, i_watchdog);

            // enable watchdog
            {
                delay_milliseconds(1);
                i_watchdog[0].start();
            }

            /* BiSS server */
            {
                BISSConfig biss_config;
                biss_config.multiturn_length = BISS_MULTITURN_LENGTH;
                biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
                biss_config.singleturn_length = BISS_SINGLETURN_LENGTH;
                biss_config.singleturn_resolution = BISS_SINGLETURN_RESOLUTION;
                biss_config.status_length = BISS_STATUS_LENGTH;
                biss_config.crc_poly = BISS_CRC_POLY;
                biss_config.pole_pairs = 15;
                biss_config.polarity = BISS_POLARITY;
                biss_config.clock_dividend = BISS_CLOCK_DIVIDEND;
                biss_config.clock_divisor = BISS_CLOCK_DIVISOR;
                biss_config.timeout = BISS_TIMEOUT;
                biss_config.max_ticks = BISS_MAX_TICKS;
                biss_config.velocity_loop = BISS_VELOCITY_LOOP;
                biss_config.offset_electrical = BISS_OFFSET_ELECTRICAL;

                biss_service(biss_ports, biss_config, i_biss);
            }
        }
    }
    return 0;
}
