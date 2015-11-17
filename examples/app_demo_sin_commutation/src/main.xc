/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.inc>
#include <IFM_DC100-rev-b.inc>


/**
 * @brief Test illustrates usage of module_commutation
 * @date 17/06/2014
 */

#include <tuning.h>

#ifdef AD7265
#include <adc_7265.h>
#else
//#include <adc_client_ad7949.h>
//#include <adc_server_ad7949.h>
    #include <adc.h>
#endif

PwmPorts pwm_ports = PWM_PORTS;
WatchdogPorts wd_ports = WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = FET_DRIVER_PORTS;
ADCPorts adc_ports = ADC_PORTS;
HallPorts hall_ports= HALL_PORTS;


#define VOLTAGE 2000 //+/- 4095

#ifdef AD7265
on tile[IFM_TILE]: adc_ports_t adc_ports =
{
        {ADC_DATA_A, ADC_DATA_B},
        ADC_INT_CLK,
        ADC_SCLK,
        ADC_READY,
        ADC_MUX
};

void adc_client(client interface ADC i_adc, chanend c_hall_){
    int sampling_time, phaseB, phaseC;
    unsigned hall_state = 0;
    while(1){
        {phaseB, phaseC, sampling_time} = i_adc.get_adc_measurements(1, 1);//port_id, config
        hall_state = get_hall_pinstate(c_hall_);
        xscope_int(PHASE_B, phaseB);
        xscope_int(PHASE_C, phaseC);
        xscope_int(HALL_PINS, hall_state);
        delay_microseconds(50);
    }
}
#endif

void adc_client(interface ADCInterface client adc_interface, chanend c_hall_p2){

    while (1) {
        int b, c;
        unsigned state;
        {b, c} = adc_interface.get_currents(); //get_adc_calibrated_current_ad7949(c_adc, I_calib);
        state = get_hall_pinstate(c_hall_p2);
        xscope_int(PHASE_B, b);
        xscope_int(PHASE_C, c);
        xscope_int(HALL_PINS, state);
        delay_microseconds(10);
    }
}

int main(void) {

    // Motor control channels
    chan c_qei_p1; // qei channels
    chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6; // hall channels
    chan c_signal; // commutation channels
    chan c_pwm_ctrl, c_adctrig; // pwm channels

    interface WatchdogInterface watchdog_interface;
    interface CommutationInterface commutation_interface[3];
    interface ADCInterface adc_interface;

    #ifdef AD7265
        interface ADC i_adc;
    #endif

    par
    {

        on tile[APP_TILE_1]:
        {   delay_seconds(1);
            /* WARNING: only one blocking task is possible per tile. */
            /* Waiting for a user input blocks other tasks on the same tile from execution. */
            run_offset_tuning(VOLTAGE, commutation_interface[0]);
        }

        on tile[IFM_TILE]: adc_client(adc_interface, c_hall_p2);

        on tile[IFM_TILE]:
        {
            par
            {
                /* ADC Loop */
#ifdef AD7265
                foc_adc_7265_continuous_loop(i_adc, adc_ports);
#else
                run_adc_service(adc_interface, adc_ports, c_adctrig);
#endif

                /* Watchdog Server */
#ifdef DC1K
                run_watchdog(c_watchdog, null, p_ifm_led_moton_wdtick_wden);
#else
                run_watchdog(watchdog_interface, wd_ports);
#endif

                /* PWM Loop */
               // {delay_milliseconds(100);
                do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, pwm_ports);


                /* Motor Commutation loop */
                {
                    hall_par hall_params;
                    qei_par qei_params;
                    commutation_par commutation_params;
                    init_hall_param(hall_params);

                    commutation_sinusoidal(c_hall_p1, c_qei_p1, c_signal,
                            watchdog_interface, commutation_interface, c_pwm_ctrl,
                            fet_driver_ports,
                            hall_params, qei_params,
                            commutation_params);
                }

                /* Hall Server */
                {
                    hall_par hall_params;
                    #ifdef DC1K
                    //connector 1
                    p_ifm_encoder_hall_select_ext_d4to5 <: SET_ALL_AS_HALL;
                    #endif
                    run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6,
                            p_ifm_hall, hall_params); // channel priority 1,2..6

                }

                /*Current sampling*/
                // It is placed here only for an educational purpose. Sampling with XSCOPE can also be done inside the adc server.
                #ifdef AD7265
                    adc_client(i_adc, c_hall_p2);
                #endif
            }
        }

    }

    return 0;
}
