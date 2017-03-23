/*
 * motor_control_interfaces.h
 *
 *  Created on: Aug 2, 2016
 *      Author: ramin
 */


#ifndef MOTOR_CONTROL_INTERFACES_H_
#define MOTOR_CONTROL_INTERFACES_H_

#include <motor_control_structures.h>

interface BrakeInterface {
    void set_brake(int enable);
    int get_brake();
};

/**
 * @brief Interface type to communicate with the Motor Control Service.
 */
interface MotorControlInterface
{
    /**
     * @brief Sets brake status to ON (no movement) or OFF (possible to move)
     */
    void set_brake_status(int brake_status);

    /**
     * @brief Enables the torque control
     */
    void set_torque_control_enabled();

    /**
     * @brief Disables the torque control
     */
    void set_torque_control_disabled();

    /**
     * @brief Enables the offset detection process
     */
    void set_offset_detection_enabled();

    /**
     * @brief Enables the safe-torque-off mode
     */
    void set_safe_torque_off_enabled();

    /**
     * @brief Shows if sensor polarity is true or wrong.
     * If the returned value is 0, then sensor polarity is wrong (sensor polarity should be changed, or motor phases should be flipped)
     * If the returned value is 1, then sensor polarity is true.
     */
    int get_sensor_polarity_state();

    /**
     * @brief Sets offset value
     */
    void set_offset_value(int offset_value);

    /**
     * @brief Sets torque target value when FOC is used.
     *
     * @param torque_sp Torque [-4096:4096].
     */
    void set_torque(int torque_sp);

    /**
     * @brief Setter for the configuration used by the Service.
     *        Note that not all configuration parameters can be changed on runtime.
     *
     * @param in_config New Service configuration.
     */
    void set_config(MotorcontrolConfig in_config);

    /**
     * @brief Getter for current configuration used by the Service.
     *
     * @return Current configuration.
     */
    MotorcontrolConfig get_config();

    /**
     * @brief Getter for actual velocity.
     *
     * @return Velocity actual.
     */
    int get_velocity_actual();

    /**
     * @brief Set calib flag in the Motorcontrol service so it will alway set 0 as electrical angle
     *
     * @param flag 1 to activate, 0 to deactivate calibration
     */
    int get_offset();

    /**
     * @brief resets the state of motor controller from faulty to normal so that
     *        the application can again be restarted.
     */
    void reset_faults();

    UpstreamControlData update_upstream_control_data ();
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
     * @param
     *      - I_max
     *      - V_dc_max
     *      - V_dc_min
     */
    void set_protection_limits(int, int, int, int, int);

    /**
     * @brief Resets the fault state in adc service
     */
    void reset_faults();
};

interface shared_memory_interface
{
    /**
     * @brief send the status of memory manager task (ACTIVE or INACTIVE) to client side
     */
    int status(void);

    /**
    * @brief Getter for UpstreamControlData in shared memory.
    *
    * @return  UpstreamControlData in shared memory.
    */
    UpstreamControlData read();

    /**
    * @brief Write electrical angle and primary position feedback (used for motion control) to shared memory.
    *
    * @param Electrical angle.
    * @param Hall state (in case HALL sensor is used).
    * @param Position.
    * @param Velocity.
    * @param sensor_error the sensor error status
    */
    void write_angle_and_primary_feedback(unsigned int angle, unsigned int hall_state, int position, int velocity, SensorError sensor_error);

    /**
    * @brief Write electrical angle and secondary position feedback (display only) to shared memory.
    *
    * @param Electrical angle.
    * @param Hall state (in case HALL sensor is used).
    * @param Position.
    * @param Velocity.
    * @param sensor_error the sensor error status
    */
    void write_angle_and_secondary_feedback(unsigned int angle, unsigned int hall_state, int position, int velocity, SensorError sensor_error);

    /**
    * @brief Write primary position feedback (used for motion control) to shared memory.
    *
    * @param Position.
    * @param Velocity.
    * @param sensor_error the sensor error status
    */
    void write_primary_feedback(int position, int velocity, SensorError sensor_error);

    /**
    * @brief Write secondary position feedback (display only) to shared memory.
    *
    * @param Position.
    * @param Velocity.
    * @param sensor_error the sensor error status
    */
    void write_secondary_feedback(int position, int velocity, SensorError sensor_error);

    /**
     * @brief Write write gpio input data to shared memory, return the gpio output data.
     *
     * @param  gpio input data.
     *
     * @return  gpio output data.
     */
    unsigned int gpio_write_input_read_output(unsigned int in_gpio);

    /**
     * @brief Write write gpio output data to shared memory.
     *
     * @param  gpio output data.
     */
    void write_gpio_output(unsigned int out_gpio);
};

interface update_pwm
{
    /**
     * @brief send the status of adc service to the client (ACTIVE/INACTIVE)
     */
    int status(void);

    void update_server_control_data(int pwm_a, int pwm_b, int pwm_c, int pwm_on, int brake_active, int recieved_safe_torque_off_mode);
    void safe_torque_off_enabled();
};

interface update_pwm_general
{
    /**
     * @brief send the status of adc service to the client (ACTIVE/INACTIVE)
     */
    int status(void);

    void update_server_control_data(unsigned short pwm_a, unsigned short pwm_b, unsigned short pwm_c, unsigned short pwm_u, unsigned short pwm_v, unsigned short pwm_w, int received_pwm_on, int recieved_safe_torque_off_mode);
    void safe_torque_off_enabled();
};

/**
 * @brief Interface type to communicate with the Watchdog Service.
 */
interface WatchdogInterface
{
    /**
     * @brief send the status of WD service to the client (ACTIVE/INACTIVE)
     */
    int status(void);

    /**
     * @brief Initialize and starts ticking the watchdog.
     */
    void start(void);

    /**
     * @brief Stops ticking the watchdog. Therefore, any output through the phases is disabled.
     */
    void stop(void);

    /**
     * @reacts on any detected fault. Any output through the phases will be disabled.
     */
    void protect(int fault_id);

    /**
     * @resets the state of fault in watchdog service, and starts the watchdog from the beginning
     */
    void reset_faults();
};

#endif /* MOTOR_CONTROL_INTERFACES_H_ */
