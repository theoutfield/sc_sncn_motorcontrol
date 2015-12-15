/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.inc>
#include <IFM_DC1K-rev-c.inc>


/**
 * @brief Test illustrates usage of module_commutation
 * @date 17/06/2014
 */

#include <tuning.h>

#include <hall_service.h>
#include <hall_config.h>
#include <motorcontrol_config.h>

#ifdef AD7265
#include <adc_7265.h>
#else
    #include <adc_service.h>
#endif

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
ADCPorts adc_ports = SOMANET_IFM_ADC_PORTS;
HallPorts hall_ports = SOMANET_IFM_HALL_PORTS;

#define VOLTAGE 200 //+/- 4095

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

void adc_client(interface ADCInterface client i_adc, interface HallInterface client i_hall){

    while (1) {
        int b, c;
        unsigned state;
        {b, c} = i_adc.get_currents();
        state = i_hall.get_hall_pinstate();
        xscope_int(PHASE_B, b);
        xscope_int(PHASE_C, c);
        xscope_int(HALL_PINS, state);
        delay_microseconds(10);
    }
}

int main(void) {

    // Motor control interfaces
    chan c_pwm_ctrl, c_adctrig; // pwm channels

    interface WatchdogInterface i_watchdog[3];
    interface ADCInterface i_adc[5];
    interface HallInterface i_hall[5];
    interface MotorcontrolInterface i_motorcontrol[5];

    par
    {

        on tile[APP_TILE]:
        {
            /* WARNING: only one blocking task is possible per tile. */
            /* Waiting for a user input blocks other tasks on the same tile from execution. */
           // run_offset_tuning(VOLTAGE, i_motorcontrol[0]);
        }

        on tile[APP_TILE]: //adc_client(i_adc[0], i_hall[1]);
        { int a,b;
            while(1){
                a = i_adc[0].get_temperature();
                printf("%d %d\n",a,b);
            }
        }

        on tile[IFM_TILE]:
        {
            par
            {
                /* ADC Loop */
#ifdef AD7265
                foc_adc_7265_continuous_loop(i_adc, adc_ports);
#else
                adc_service(adc_ports, c_adctrig, i_adc);
#endif

                /* Watchdog Server */
                watchdog_service(wd_ports, i_watchdog);


                /* PWM Loop */
                pwm_triggered_service( pwm_ports, c_adctrig, c_pwm_ctrl);

                /* Hall Server */
                {
                    HallConfig hall_config;
                    init_hall_config(hall_config);

                    hall_service(hall_ports, hall_config, i_hall);
                }

                /* Motor Commutation loop */
                {
                    MotorcontrolConfig motorcontrol_config;
                    init_motorcontrol_config(motorcontrol_config);

                    motorcontrol_service(fet_driver_ports, motorcontrol_config,
                                            c_pwm_ctrl, i_hall[0], null, i_watchdog[0], i_motorcontrol);
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
