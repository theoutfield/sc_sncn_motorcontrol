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
interface MotorcontrolInterface
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
     * @brief Getter for actual position.
     *
     * @return Position actual.
     */
    int get_position_actual();

    /**
     * @brief Set calib flag in the Motorcontrol service so it will alway set 0 as electrical angle
     *
     * @param flag 1 to activate, 0 to deactivate calibration
     */
    int set_calib(int flag);

    /**
     * @brief resets the state of motor controller from faulty to normal so that
     *        the application can again be restarted.
     */
    void reset_faults();

    UpstreamControlData update_upstream_control_data ();
};


interface update_brake
{
    /**
     * @brief send the brake settings to pwm server
     */
    void update_brake_control_data(int duty_start_brake, int duty_maintain_brake, int period_start_brake);
};


/**
 * @brief Interface type to communicate with the ADC Service.
 */
interface ADCInterface
{
    {int, int}  get_channel(unsigned short);

    /**
     * @brief send the status of adc service to the client (ACTIVE/INACTIVE)
     */
    int status(void);

    /**
     * @brief Get all measured parameters at once
     * The parameters include:
     *  - Current on Phase B
     *  - Current on Phase C
     *  - Vdc
     *  - Torque
     *  - fault code
     */
    {int, int, int, int, int, int, int, int, int, int} get_all_measurements();


    /**
     * @brief Sets the protection limits including:
     *      - I_max
     *      - V_dc_max
     *      - V_dc_min
     */
    void set_protection_limits_and_analogue_input_configs(int, int, int, int, int);

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
    */
    void write_angle_and_primary_feedback(unsigned int angle, unsigned int hall_state, int position, int velocity);

    /**
    * @brief Write electrical angle and secondary position feedback (display only) to shared memory.
    *
    * @param Electrical angle.
    * @param Hall state (in case HALL sensor is used).
    * @param Position.
    * @param Velocity.
    */
    void write_angle_and_secondary_feedback(unsigned int angle, unsigned int hall_state, int position, int velocity);

    /**
    * @brief Write primary position feedback (used for motion control) to shared memory.
    *
    * @param Position.
    * @param Velocity.
    */
    void write_primary_feedback(int position, int velocity);

    /**
    * @brief Write secondary position feedback (display only) to shared memory.
    *
    * @param Position.
    * @param Velocity.
    */
    void write_secondary_feedback(int position, int velocity);

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
