/*
 * motor_control_interfaces.h
 *
 *  Created on: March, 23, 2017
 *      Author: Synapticon
 */


#ifndef MOTOR_CONTROL_INTERFACES_H_
#define MOTOR_CONTROL_INTERFACES_H_

#include <motor_control_structures.h>


/**
 * @brief Interface type to communicate with Motor Control Service.
 */
interface TorqueControlInterface
{
    /**
     * @brief Sets brake status to ON/OFF
     *
     * @param brake_status      integer value to activate/deactivate brake (0 => deactive, 1=> active)
     *
     * @return void
     */
    void set_brake_status(int brake_status);

    /**
     * @brief configures the brake settings including its voltages and timing
     *
     * @param pull_brake_voltage  voltage applied to electric brake at startup of brake
     * @param pull_brake_time     period of applying high voltage to electric brake at startup (in milliseconds)
     * @param hold_brake_voltage  voltage applied to electric brake after the brake is pulled
     */
    void configure_brake(int pull_brake_voltage, int pull_brake_time, int hold_brake_voltage);

    /**
     * @brief Enables the torque control
     *
     * @return void
     */
    void set_torque_control_enabled();

    /**
     * @brief Disables the torque control
     *
     * @return void
     */
    void set_torque_control_disabled();

    /**
     * @brief Enables the offset detection process
     *
     * @return void
     */
    void set_offset_detection_enabled();

    /**
     * @brief Sends the status of the sensor to the motorcontrol
     *
     * @return void
     */
    void set_sensor_status(int error_sensor);

    /**
     * @brief Enables the safe-torque-off mode
     *
     * @return void
     */
    void set_safe_torque_off_enabled();

    /**
     * @brief Shows if sensor polarity is true or wrong.
     *
     * @return Integer value (if 0, then sensor polarity is wrong)
     */
    int get_sensor_polarity_state();

    /**
     * @brief Sets offset value
     *
     * @param offset_value  integer value which will be used as commutation angle offset
     *
     * @return void
     */
    void set_offset_value(int offset_value);

    /**
     * @brief Sets torque target value.
     *
     * @param integer value torque_sp in milli-Nm
     *
     * @return void
     */
    void set_torque(int torque_sp);

    /**
     * @brief   Sets MotorcontrolConfig parameters which are used by motor_control_service.
     *          Note that not all configuration parameters can be changed on runtime.
     *
     * @param   in_config MotorcontroConfig structure which will be passed to motor_control_service
     *
     * @return  void
     */
    void set_config(MotorcontrolConfig in_config);

    /**
     * @brief Gets MotorcontrolConfig parameters which are used by motor_control_service.
     *
     * @return MotorcontroConfig structure which will be passed to motor_control_service
     */
    MotorcontrolConfig get_config();

    /**
     * @brief   Gets the offset value from motor_control_service
     *
     * @return  calculated offset vlaue (in case offset value is not calculated yet, the returned value will be -1).
     */
    int get_offset();

    /**
     * @brief   resets the state of motor controller from faulty to normal so that the application can again be restarted.
     *
     * @return  void
     */
    void reset_faults();

    /**
     * @brief   starts global system evaluation (fault detection)
     *
     * @return  void
     */
    void start_system_eval();

    /**
     * @brief   sets reference of phase voltages directly on the output of inverter
     *
     * @return  void
     */
    void set_evaluation_references(int phase_a, int phase_b, int phase_c);

   /**
    * @brief   Send upstream control data and receive GPIO output values
    *
    * @param   gpio_output value to ouput to the GPIO pins (rightmost bit is GPIO 1)
    *
    * @return  upstream control data
    */
   UpstreamControlData update_upstream_control_data(unsigned int gpio_output);

    /**
     * @brief   Enables the cogging torque compensation
     *
     * @return  void
     */
    void enable_cogging_compensation();

    /**
     * @brief   Disables the cogging torque compensation
     *
     * @return  void
     */
    void disable_cogging_compensation();

    /**
     * @brief   Enables the rotation of the motor to find the index of the incremental encoder
     *
     * @return  void
     */
    void enable_index_detection();

    /**
     * @brief   Disables the rotation of the motor when the index of the incremental encoder would be found
     *
     * @return  void
     */
    void disable_index_detection();
};

/**
 * @brief Interface type to communicate with PWM service and update brake parameters
 */
interface UpdateBrake
{
    /**
     * @brief send the brake settings to pwm server
     *
     * @param   duty_start_brake    pwm duty which will be used to pull the brake out (activate the brake at startup)
     * @param   duty_maintain_brake pwm duty which will be used to hold the brake after it is released
     * @param   period_start_brake  period (in milliseconds) in which the brake is pulled for being released
     *
     * @return  void
     */
    void update_brake_control_data(int duty_start_brake, int duty_maintain_brake, int period_start_brake);
};

/**
 * @brief Interface type to communicate with the ADC Service.
 */
interface ADCInterface
{

    /**
     * @brief sends the required channel (to be sampled by ADC), and recieves its corresponding analogue input values
     *
     * @param   adc channel (to be sampled by ADC)
     *
     * @return  two integer values corresponding to the voltage of selected channel
     */
    {int, int}  get_channel(unsigned short);

    /**
     * @brief send the status of adc service to the client (ACTIVE/INACTIVE)
     *
     * @return  integer value corresponding to ACTIVE/INACTIVE enumeration
     */
    int status(void);

    /**
     * @brief gets all adc measured parameters at once. In its most complete form, these parameters will be:
     *
     * @return seven integer values including:
     *  - phase current B
     *  - phase current C
     *  - v_dc
     *  - i_dc
     *  - temperature
     *  - analogue input a1
     *  - analogue input a2
     *  - analogue input b1
     *  - analogue input b2
     *  - fault code
     */
    {int, int, int, int, int, int, int, int, int, int} get_all_measurements();

    /**
     * @brief Sets the protection limits including:
     *
     * @param   limit_oc -> high limit for adc output value in adc channels which are connected to current sensors
     * @param   limit_ov -> high limit for adc output value in adc channels which are connected to dc link
     * @param   limit_uv -> low  limit for adc output value in adc channels which are connected to dc link
     * @param   limit_ot -> high limit for adc output value in adc channels which are connected to temperature sensor
     *
     * @return  void
     */
    void set_protection_limits(int limit_oc, int limit_ov, int limit_uv, int limit_ot);

    /**
     * @brief   Resets the fault state in adc service
     *
     * @return  void
     */
    void reset_faults();
};


/**
 * @brief Interface exchange data between tasks OF THE SAME TILE without blocking tasks execution.
 */
interface shared_memory_interface
{
    /**
    * @brief Getter for UpstreamControlData in shared memory.
    *
    * @return  UpstreamControlData in shared memory.
    */
    UpstreamControlData read();

    /**
    * @brief Getter for UpstreamControlData in shared memory. Also write GPIO output to shared memory.
    *
    * @param   gpio_output value to ouput to the GPIO pins (rightmost bit is GPIO 1)
    *
    * @return  UpstreamControlData in shared memory.
    */
    UpstreamControlData read_upstream_data_and_write_gpio_output(unsigned int gpio_output);

    /**
    * @brief Write electrical angle and primary position feedback (used for motion control) to shared memory.
    *
    * @param Electrical angle.
    * @param Hall state (in case HALL sensor is used).
    * @param Qei_index_found flag that indicates if the position data is absolute or not
    * @param Position.
    * @param Velocity.
    * @param sensor_error the sensor error status.
    * @param last_sensor_error the last non zero sensor error status.
    * @param timestamp timestamp in microseconds of when the position data was read.
    */
    void write_angle_and_primary_feedback(unsigned int angle, unsigned int hall_state, unsigned int qei_index_found, int position, int singleturn, int velocity, SensorError sensor_error, SensorError last_sensor_error, unsigned int timestamp);

    /**
    * @brief Write electrical angle to shared memory.
    *
    * @param Electrical angle.
    * @param Hall state (in case HALL sensor is used).
    * @param Qei_index_found flag that indicates if the position data is absolute or not
    * @param Velocity.
    * @param sensor_error the sensor error status.
    * @param last_sensor_error the last non zero sensor error status.
    */
    void write_angle(unsigned int angle, unsigned int hall_state, unsigned int qei_index_found, int velocity, SensorError sensor_error, SensorError last_sensor_error);

    /**
    * @brief Write electrical angle and secondary position feedback (display only) to shared memory.
    *
    * @param Electrical angle.
    * @param Hall state (in case HALL sensor is used).
    * @param Qei_index_found flag that indicates if the position data is absolute or not
    * @param Position.
    * @param Velocity.
    * @param sensor_error the sensor error status
    * @param last_sensor_error the last non zero sensor error status.
    * @param timestamp timestamp in microseconds of when the position data was read.
    */
    void write_angle_and_secondary_feedback(unsigned int angle, unsigned int hall_state, unsigned int qei_index_found, int position, int singleturn, int velocity, SensorError sensor_error, SensorError last_sensor_error, unsigned int timestamp);

    /**
    * @brief Write primary position feedback (used for motion control) to shared memory.
    *
    * @param Position.
    * @param Velocity.
    * @param sensor_error the sensor error status
    * @param last_sensor_error the last non zero sensor error status.
    * @param timestamp timestamp in microseconds of when the position data was read.
    */
    void write_primary_feedback(int position, int singleturn, int velocity, SensorError sensor_error, SensorError last_sensor_error, unsigned int timestamp);

    /**
    * @brief Write secondary position feedback (display only) to shared memory.
    *
    * @param Position.
    * @param Velocity.
    * @param sensor_error the sensor error status
    * @param last_sensor_error the last non zero sensor error status.
    * @param timestamp timestamp in microseconds of when the position data was read.
    */
    void write_secondary_feedback(int position, int singleturn, int velocity, SensorError sensor_error, SensorError last_sensor_error, unsigned int timestamp);

    /**
     * @brief Write write gpio input data to shared memory, return the gpio output data.
     *
     * @param  gpio input data.
     *
     * @return  gpio output data.
     */
    unsigned int gpio_write_input_read_output(unsigned int in_gpio);

    /**
     * @brief Write gpio output data to shared memory.
     *
     * @param  gpio output data.
     */
    void write_gpio_output(unsigned int out_gpio);

    /**
     * @brief Write hall state angle to shared memory.
     *
     * @param in_hall_state_angle Hall state angle array
     */
    void write_hall_state_angle(int in_hall_state_angle[6]);

    /**
     * @brief Read hall state angle from shared memory.
     *
     * @param out_hall_state_angle Hall state angle array
     *
     * @return 1 if the data is valid
     */
    int read_hall_state_angle(int out_hall_state_angle[6]);
};

/**
 * @brief Interface type to communicate with the PWM Service
 */
interface UpdatePWM
{
    /**
     * @brief send the status of adc service to the client (ACTIVE/INACTIVE)
     *
     * @return state of PWM service ACTIVE/DEACTIVE
     */
    int status(void);

    /**
     * @brief send the pwm values and pwm controling commands to pwm service
     *
     * @param   pwm_a pwm value for phase a
     * @param   pwm_b pwm value for phase b
     * @param   pwm_c pwm value for phase c
     * @param   pwm_on determines whether pwm service generates the pulses or not
     * @param   brake_active activates/deactivates the brake functionality
     * @param   safe_torque_off_mode if set to 1 then pwm will not work in normal mode
     *
     * @return  void
     */
    void update_server_control_data(int pwm_a, int pwm_b, int pwm_c, int pwm_on, int brake_active, int safe_torque_off_mode);

    /**
     * @brief send safe_torque_off_mode command to pwm service
     *
     * @return  void
     */
    void safe_torque_off_enabled();
};

/**
 * @brief Interface type to communicate with the general PWM Service.
 */
interface UpdatePWMGeneral
{
    /**
     * @brief send the status of adc service to the client (ACTIVE/INACTIVE)
     *
     * @return state of PWM service ACTIVE/DEACTIVE
     */
    int status(void);

    /**
     * @brief send the frequency of pwm service to the client side (kHz)
     *
     * @return frequency of pwm service (kHz)
     */
    int frequency(void);

    /**
     * @brief send the pwm values and pwm controling commands to pwm service
     *
     * @param   pwm_a pwm value for phase a
     * @param   pwm_b pwm value for phase b
     * @param   pwm_c pwm value for phase c
     * @param   pwm_u pwm value for phase u
     * @param   pwm_v pwm value for phase v
     * @param   pwm_w pwm value for phase w
     * @param   pwm_on determines whether pwm service generates the pulses or not
     * @param   safe_torque_off_mode if set to 1 then pwm will not work in normal mode
     *
     * @return  void
     */
    void update_server_control_data(unsigned short pwm_a, unsigned short pwm_b, unsigned short pwm_c, unsigned short pwm_u, unsigned short pwm_v, unsigned short pwm_w,unsigned short pwm_on);

    /**
     * @brief send safe_torque_off_mode command to pwm service
     *
     * @return  void
     */
    void safe_torque_off_enabled();
};

/**
 * @brief Interface type to communicate with the Watchdog Service.
 */
interface WatchdogInterface
{
    /**
     * @brief reacts on any detected fault. Any output through the phases will be disabled.
     */
    void protect(int fault_id);

    /**
     * @brief resets the state of fault in watchdog service, and starts the watchdog from the beginning
     */
    void reset_faults();

    /**
     * @brief read the fault monitor led
     */
    WatchdogError read_fault_monitor();
};

#endif /* MOTOR_CONTROL_INTERFACES_H_ */
