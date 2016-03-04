/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C21-rev-a.bsp>
//#include <IFM_DC100-rev-b.bsp>
#include <IFM_DC1K-rev-c2.bsp>

/**
 * @brief Test illustrates usage of module_commutation
 * @date 17/06/2014
 */

#include <platform.h>
#include <hall_service.h>
#include <pwm_service.h>
#include <xscope.h>
#include <adc_service.h>
#include <watchdog_service.h>
//#include <foc_base.h>
#include <motorcontrol_service.h>
//#include <torque_control.h>

#include <user_config.h>

ADCPorts adc_ports = SOMANET_IFM_ADC_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
#if(MOTOR_COMMUTATION_SENSOR == BISS_SENSOR)
BISSPorts biss_ports = SOMANET_IFM_BISS_PORTS;
#elif(MOTOR_COMMUTATION_SENSOR == AMS_SENSOR)
AMSPorts ams_ports = SOMANET_IFM_AMS_PORTS;
#else
HallPorts hall_ports = SOMANET_IFM_HALL_PORTS;
#endif

#define TORQUE 1000 //+/- 4095
#define Q_MAX 3000

void simple_torque_controller(client interface foc_base i_foc_base){
    delay_seconds(5);
    int torque_actual = 0, error = 0, setpoint = 0, feedforward = 200;
    while(1){
        error = TORQUE - i_foc_base.get_torque_actual();
        if (error > 0) setpoint++;
        else setpoint--;

        if (setpoint > Q_MAX) setpoint = Q_MAX;
        if (setpoint < -Q_MAX) setpoint = -Q_MAX;

        if((setpoint > 0)  && (setpoint < 50)) setpoint = 50;
        else if ((setpoint < 0)  && (setpoint > -50)) setpoint = -50;

        if (setpoint < feedforward) setpoint = feedforward;

        xscope_int(DEBUG_VALUE, setpoint);
        xscope_int(CONTROL_ERROR, error);

        i_foc_base.set_q(setpoint);
        delay_milliseconds(1);
    }
}


int main(void) {

    // Motor control channels
    chan c_pwm_ctrl, c_adctrig;  // pwm channels

    interface ADCInterface i_adc[2];
    interface foc_base i_foc;
    interface WatchdogInterface i_watchdog[2];
    interface MotorcontrolInterface i_motorcontrol[4];
#if(MOTOR_COMMUTATION_SENSOR == BISS_SENSOR)
    interface BISSInterface i_biss[5];
#elif(MOTOR_COMMUTATION_SENSOR == AMS_SENSOR)
    interface AMSInterface i_ams[5];
#else
    interface HallInterface i_hall[5];
#endif

    par
    {

        on tile[APP_TILE]:
        {
            i_foc.set_q(TORQUE);
        }

        on tile[IFM_TILE]:
        {
            par
            {
               // simple_torque_controller(i_foc_base);

                /* Triggered PWM Service */
          //      pwm_triggered_service( pwm_ports, c_adctrig, c_pwm_ctrl);
                pwm_service(pwm_ports, c_pwm_ctrl);

                /* ADC Service */
                adc_service(adc_ports, c_adctrig, i_adc);
       //         adc_service(adc_ports, null, i_adc);

                /* Watchdog Service */
                watchdog_service(wd_ports, i_watchdog);

                /* Motor Commutation Service */
                {
                    MotorcontrolConfig motorcontrol_config;
                    motorcontrol_config.motor_type = BLDC_MOTOR;
                    motorcontrol_config.commutation_method = FOC;
                    motorcontrol_config.commutation_sensor = MOTOR_COMMUTATION_SENSOR;
                    motorcontrol_config.bldc_winding_type = BLDC_WINDING_TYPE;
                    motorcontrol_config.hall_offset[0] =  COMMUTATION_OFFSET_CLK;
                    motorcontrol_config.hall_offset[1] = COMMUTATION_OFFSET_CCLK;
                    motorcontrol_config.commutation_loop_period =  COMMUTATION_LOOP_PERIOD;

#if(MOTOR_COMMUTATION_SENSOR == BISS_SENSOR)
                    motorcontrol_service(fet_driver_ports, motorcontrol_config,
                                         c_pwm_ctrl, i_adc[0], null, null, i_biss[0], null, i_watchdog[0], i_motorcontrol, i_foc);
#elif(MOTOR_FEEDBACK_SENSOR == AMS_SENSOR)
                    motorcontrol_service(fet_driver_ports, motorcontrol_config,
                                         c_pwm_ctrl, i_adc[0], null, null, null, i_ams[0], i_watchdog[0], i_motorcontrol, i_foc);
#else
                    motorcontrol_service(fet_driver_ports, motorcontrol_config,
                                         c_pwm_ctrl, i_adc[0], i_hall[0], null, null, null, i_watchdog[0], i_motorcontrol, i_foc);
#endif
                }


#if(MOTOR_COMMUTATION_SENSOR == BISS_SENSOR)
                /* BiSS service */
                {
                    BISSConfig biss_config;
                    biss_config.multiturn_length = BISS_MULTITURN_LENGTH;
                    biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
                    biss_config.singleturn_length = BISS_SINGLETURN_LENGTH;
                    biss_config.singleturn_resolution = BISS_SINGLETURN_RESOLUTION;
                    biss_config.status_length = BISS_STATUS_LENGTH;
                    biss_config.crc_poly = BISS_CRC_POLY;
                    biss_config.pole_pairs = POLE_PAIRS;
                    biss_config.polarity = BISS_POLARITY;
                    biss_config.clock_dividend = BISS_CLOCK_DIVIDEND;
                    biss_config.clock_divisor = BISS_CLOCK_DIVISOR;
                    biss_config.timeout = BISS_TIMEOUT;
                    biss_config.max_ticks = BISS_MAX_TICKS;
                    biss_config.velocity_loop = BISS_VELOCITY_LOOP;
                    biss_config.offset_electrical = BISS_OFFSET_ELECTRICAL;

                    biss_service(biss_ports, biss_config, i_biss);
                }
#elif(MOTOR_COMMUTATION_SENSOR == AMS_SENSOR)
                /* AMS Rotary Sensor Service */
                {
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
                    ams_config.pole_pairs = POLE_PAIRS;
                    ams_config.max_ticks = 0x7fffffff;
                    ams_config.cache_time = AMS_CACHE_TIME;
                    ams_config.velocity_loop = AMS_VELOCITY_LOOP;

                    ams_service(ams_ports, ams_config, i_ams);
                }
#else
                /* Hall sensor Service */
                {
                    HallConfig hall_config;
                    hall_config.pole_pairs = POLE_PAIRS;

                    hall_service(hall_ports, hall_config, i_hall);
                }
#endif


            }
        }

    }

    return 0;
}
