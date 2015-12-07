/**
 * @file commutation_server.h
 * @brief Commutation Loop based on sinusoidal commutation method
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <pwm_cli_inv.h>
#include <watchdog_service.h>
#include <hall_service.h>
#include <qei_service.h>

#include <a4935.h>
#include <internal_config.h>

#define ERROR 0
#define SUCCESS 1

typedef enum {BDC_MOTOR = 10, BLDC_MOTOR = 11} MotorType;

typedef struct {
    port ?p_coast;
    out port ?p_esf_rst_pwml_pwmh;
    port ?p_ff1;
    port ?p_ff2;
} FetDriverPorts;

typedef struct {
    MotorType motor_type;
    int angle_variance;         /* max allowed variance depending on speed */
    int nominal_speed;
    int qei_forward_offset;
    int qei_backward_offset;
    int hall_offset_clk;
    int hall_offset_cclk;
    int bldc_winding_type;
    int commutation_loop_freq;
} MotorcontrolConfig;

interface MotorcontrolInterface{
    int checkBusy();
    void setVoltage(int voltage);
    void setParameters(MotorcontrolConfig parameters);
    void setSensor(int sensor);
    void disableFets();
    void enableFets();
    int getFetsState();
    void setAllParameters(HallConfig hall_config, QEIConfig qei_config, MotorcontrolConfig commutation_config, int in_nominal_speed);
};

/**
 * @brief Sinusoidal based Commutation Loop
 *
 * @param[in] c_hall A chanend connected to the hall server
 * @param[in] c_qei A chanend connected to the qei server (QEI)
 * @param[in] c_signal A chanend for signaling after initialization of commutation loop
 * @param[in] c_watchdog A chanend connected to the watchdog
 * @param[in] c_commutation_p1 channel for receiving motor voltage input value - priority 1 (highest) 1 ... (lowest) 3
 * @param[in] c_commutation_p2 channel for receiving motor voltage input value - priority 2
 * @param[in] c_commutation_p3 channel for receiving motor voltage input value - priority 3
 * @param[out] c_pwm_ctrl channel to set PWM level output to motor phases
 * @param[out] p_ifm_esf_rstn_pwml_pwmh port to configure motor FET driver
 * @param[out] p_ifm_coastn port to enable motor FET driver
 * @param[out] p_ifm_ff1
 * @param[out] p_ifm_ff2
 * @param[in] hall_config struct defines the pole-pairs and gear ratio
 * @param[in] qei_params the struct defines sensor type and resolution parameters for QEI
 * @param[in] commutation_params struct defines the commutation angle parameters
 *
 */
[[combinable]]
void motorcontrol_service(FetDriverPorts &fet_driver_ports, MotorcontrolConfig &motorcontrol_config,
                            chanend c_pwm_ctrl,
                            interface HallInterface client ?i_hall,
                            interface QEIInterface client ?i_qei,
                            interface WatchdogInterface client i_watchdog,
                            interface MotorcontrolInterface server i_motorcontrol[5]);
