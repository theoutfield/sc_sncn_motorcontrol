Torque Control Test
===============
**test_torque-ctrl.xc** illustrates usage of [module_ctrl_loops]() to do torque control of a motor. Position loop is closed with information from the current on the motor phases, measured by ADC sensors.

<table class="core_usage" align="center" cellpadding="5" width="20%">
<tr>
    <th colspan="2">CORE use</th>
</tr>
<tr>
    <td>Parallel THREADS</td>
    <td width="30px" align="center"> 8 </td>
</tr>
<tr>
    <td> TILES used</td>
    <td width="30px" align="center"> 2 </td>
 </tr>
</table>

<table  class="hw_comp" align="center" cellpadding="2" width="50%">
<tr align="center">
    <th colspan="3">HW compatibility</th>
  <tr align="center">
    <th>COM</th>
    <th>CORE</th>
    <th>IFM</th>
  </tr>
  <tr align="center">
    <td>*</td>
    <td>C21-DX</td>
   <td>Drive DC 100</td>
 </tr>
  <tr align="center">
    <td></td>
    <td>C22</td>
    <td>Drive DC 300</td>
  </tr>
</table>

- **THREADS**: Profile Torque Client, Torque Control Loop, PWM Server, ADC Server, Commutation Server, Hall Server, QEI Server, Watchdog Server.

- **TILES**:
```
	#define TILE_ONE 0
	#define IFM_TILE 3
```

> **Do not forget to set properly your node and motor configuration when using this application**.

- [Configure your node]() 
- [How to configure your motors](https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/howto/HOW_TO_CONFIGURE_MOTORS.md)


### **TILE_ONE**
This tile (0 by default) takes care of the client side functions and control loop. Since these functions do not require any port access, any free TILE could run them.
```
	on stdcore[TILE_ONE]:
```
- **Thread**: Profile Torque Client
```
	profile_torque_test(c_torque_ctrl); // Test Torque Profile Mode on slave side
```
Set new target torque for the controller. Read more at [module_profile]().


- **Thread**: Torque Control Loop

```
	 ctrl_par position_ctrl_params; //Var definition
	 hall_par hall_params;
	 qei_par qei_params;

	init_torque_control_param(torque_ctrl_params); //Initialization
	init_qei_param(qei_params);
	init_hall_param(hall_params);

	torque_control(torque_ctrl_params, hall_params, qei_params, SENSOR_USED, 
			c_adc, c_commutation_p1,  c_hall_p3,  c_qei_p3, c_torque_ctrl); //Control loop
```		
Read back actual torque of the motor. Read more at [module_ctrl_loops]().

### **IFM_TILE** 
(3 by default)
It executes the server side functions, controlling the interfaces. These functions need access to the Interface Module (IFM), just the tile that provides access to the \b IFM ports can run these functions.  

```
	on stdcore[IFM_TILE]: 
```
- **Thread**: PWM Server.
```
	do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port, p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);
```
Responsible for generating a Pulse-Width Modulation signal that drives the motors. Provided by the [module_pwm_symmetrical]() at PWM software component \b sc_pwm.

- **Thread**: ADC Server.
```	
	adc_ad7949_triggered(c_adc, c_adctrig, clk_adc, p_ifm_adc_sclk_conv_mosib_mosia,
				p_ifm_adc_misoa, p_ifm_adc_misob);
```
It captures current values in the motor phases. Read more at [module_adc]().

- **Thread**: Commutation Server 
```
	hall_par hall_params; //Var definition
	qei_par qei_params;
	commutation_par commutation_params;

	init_hall_param(hall_params); //Initialization
	init_qei_param(qei_params);
	init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED); 

	commutation_sinusoidal(c_hall_p1,  c_qei_p2, c_signal, c_watchdog, c_commutation_p1,
				c_commutation_p2, c_commutation_p3, c_pwm_ctrl, hall_params,
				qei_params, commutation_params); //Read feedback
```

Responsible for proper BLDC motor drive. Read more at [module_commutation]().

- **Thread**: Hall Server
```
	hall_par hall_params;
	init_hall_param(hall_params);
	run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5,
			p_ifm_hall, hall_params); //Channel priority 1,2..5
```
To obtain information about motor position for position control loop, its use is mandatory since the motor commutation is Hall-based. Read more at [module_hall]().

- **Thread**: QEI Server
```
	qei_par qei_params;
	init_qei_param(qei_params);
	run_qei(c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5,
			p_ifm_encoder, qei_params);  	// channel priority 1,2..5
```
To obtain high precision information about motor position. Read more at [module_qei]().

- **Thread**: Watchdog Server
```
	run_watchdog(c_watchdog, p_ifm_wd_tick, p_ifm_shared_leds_wden);
```
A watchdog server is used to monitor IFM_TILE and disables motor in case of emergency. Read more at [module_watchdog]().


More information about Position Control Server/Client can be found at [module_ctrl_loops]() documentation.

Other dependencies:  [sc_somanet-base/module_nodeconfig]() [module_blocks]() [module_common]() [module_sm]() 

**See also**:

- [Getting started with SOMANET](http://doc.synapticon.com/wiki/index.php/Category:Getting_Started_with_SOMANET)  
