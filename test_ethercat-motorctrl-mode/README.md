Ethercat Mode Test
======================

test_ethercat-mode.xc illustrates the usage of Motor Control over EtherCAT with advanced features.

This example must be run..

(SPEAK ABOUT THE MASTER SIDE)


<table class="core_usage" align="center" cellpadding="5" width="20%">
<tr>
    <th colspan="2">CORE use</th>
</tr>
<tr>
    <td>Parallel THREADS</td>
    <td width="30px" align="center"> 12 </td>
</tr>
<tr>
    <td>TILES used</td>
    <td width="30px" align="center"> 4 </td>
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
    <td>EtherCAT</td>
    <td>C22</td>
   <td>Drive DC 100</td>
 </tr>
  <tr align="center">
    <td></td>
    <td></td>
    <td>Drive DC 300</td>
  </tr>
</table>


- **THREADS**: PWM Server, Commutation Server, Watchdog Server, ADC Server, Digital GPIO server, Hall Server, QEI Server, EtherCAT Communication Handling, EtherCAT-Motor Drive, Position Control Loop, Velocity Control Loop, and Torque Control Loop.

- **TILES**:
```	
	#define COM_TILE 0
	#define TILE_ONE 1
	#define TILE_TWO 2
	#define IFM_TILE 3
```

> **Do not forget to set properly your node and motor configuration when using this application**.

- [Configure your node]() 
- [How to configure your motors](https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/howto/HOW_TO_CONFIGURE_MOTORS.md)

###COM_TILE 
This tile (0 by default) runs threads responsible for handling communication over EtherCAT. They need to access the Communication Module (COM), just a tile that provides access to the COM ports can run these functions.
```
    on stdcore[COM_TILE] :
```
- **Thread**: Ethercat Communication Handling
```
	ecat_init();

	ecat_handler(coe_out, coe_in, eoe_out, eoe_in, eoe_sig, foe_out,
		      foe_in, pdo_out, pdo_in);
```
Ethercat communication handler loop. Read more at sc_sncn_ethercat/module_ethercat.

###TILE_ONE 
This tile (1 by default) executes a communication bridge between RX EtherCAT and motor drive. Since these functions do not require any port access, any free TILE could run them.
```
	on stdcore[TILE_ONE]:
```
- **Thread**: EtherCAT-Motor Drive
```	
	ecat_motor_drive(pdo_out, pdo_in, coe_out, c_signal, c_hall_p5, c_qei_p5,
	c_torque_ctrl, c_velocity_ctrl, c_position_ctrl, c_gpio_p1);
```
It forms a communication bridge between EtherCAT and motor drive. Read more at \ref module_ecat_drive.

###TILE_TWO
This tile (1 by default) will execute the different mode control loops. Since these functions do not require any port access, any free TILE could run them.
```
	on stdcore[TILE_TWO]:
```
- **Thread**: Position Control Loop
```
	ctrl_par position_ctrl_params; //Var definition
	hall_par hall_params;
	qei_par qei_params;

	init_position_control_param(position_ctrl_params); //Initialization
	init_hall_param(hall_params);
	init_qei_param(qei_params);

	position_control(position_ctrl_params, hall_params, qei_params,
		              SENSOR_USED, c_hall_p2, c_qei_p1,       
		              c_position_ctrl, c_commutation_p3); //Control loop
```
Read back actual position of the motor. Read more at \ref module_ctrl_loops.

- **Thread**: Velocity Control Loop
```
	ctrl_par velocity_ctrl_params; //Var definition
	filter_par sensor_filter_params;
	hall_par hall_params;
	qei_par qei_params;

	init_velocity_control_param(velocity_ctrl_params); //Initialization
	init_sensor_filter_param(sensor_filter_params);
	init_hall_param(hall_params);
	init_qei_param(qei_params);

	velocity_control(velocity_ctrl_params, sensor_filter_params, hall_params,
			qei_params, SENSOR_USED, c_hall_p3, c_qei_p3, 
			c_velocity_ctrl, c_commutation_p2); //Control loop
```
Read back actual velocity of the motor. Read more at \ref module_ctrl_loops or \ref module_blocks.

- **Thread**: Torque Control Loop
```
	ctrl_par torque_ctrl_params; //Var definition
	hall_par hall_params;
	qei_par qei_params;

	init_qei_param(qei_params); //Initialization
	init_hall_param(hall_params);
	init_torque_control_param(torque_ctrl_params);

	torque_control( torque_ctrl_params, hall_params, qei_params,
			SENSOR_USED, c_adc, c_commutation_p1, 
			c_hall_p2, c_qei_p2, c_torque_ctrl); //Control loop
```
Read back actual torque of the motor. Read more at \ref module_ctrl_loops.

###IFM_TILE 
This tile (3 by default) executes the server side functions, controlling the interfaces. These functions need access to the Interface Module (IFM), just the tile that provides access to the IFM ports can run these functions.  
```
    on stdcore[IFM_CORE]: 
```                   
We need to run this threads over IFM_TILE since this is the only tile accessing the interface module (IFM). And these threads, responsible for motor control, require for this access.

- **Thread**: PWM Server.
```
      do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port, p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);
```
Responsible for generating a Pulse-Width Modulation signal that drives the motors. Provided by the **module_pwm_symmetrical at PWM software component **sc_pwm.

- **Thread**: Commutation Server
```
	hall_par hall_params; //Var definition
	qei_par qei_params;
	commutation_par commutation_params;

	init_hall_param(hall_params); //Initialization
	init_qei_param(qei_params);
	init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED);     

	commutation_sinusoidal(c_hall_p1,  c_qei_p2, c_signal, c_watchdog,  //Commutation       
	c_commutation_p1, c_commutation_p2, c_commutation_p3, 
	c_pwm_ctrl, hall_params, qei_params, commutation_params);
```
Responsible for proper BLDC motor drive. Read more at \ref module_commutation.

- **Thread**: Hall Server
```
	hall_par hall_params;
	init_hall_param(hall_params);
	run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, 
		      p_ifm_hall, hall_params); // channel priority 1,2..5
```
To obtain information about motor position for position control loop, its use is mandatory since the motor commutation is Hall-based. Read more at \ref module_hall.

- **Thread**: QEI Server
```
	qei_par qei_params;
	init_qei_param(qei_params);
	run_qei(c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5, 
		      p_ifm_encoder, qei_params);     // channel priority 1,2..5
```
To obtain high precision information about motor position. Read more at \ref module_qei.

- **Thread**: Watchdog server
```
      run_watchdog(c_watchdog, p_ifm_wd_tick, p_ifm_shared_leds_wden);
```
A watchdog server is used to monitor IFM_TILE and disables motor in case of emergency. Read more at \ref module_watchdog.

- **Thread**: ADC Server
```
	adc_ad7949_triggered(c_adc, c_adctrig, clk_adc, 173  p_ifm_adc_sclk_conv_mosib_mosia, 
      				p_ifm_adc_misoa, p_ifm_adc_misob); 
```
It captures current values in the motor phases. Read more at \ref module_adc.

- **Thread**: Digital GPIO Server
```
      gpio_digital_server(p_ifm_ext_d, c_gpio_p1, c_gpio_p2);
```
It access the GPIO ports at the IFM module. See more at \ref module_gpio.

More information about Server/Client Control can be found on \ref module_ctrl_loops. And more information about motor control over EtherCAT on \ref module_ecat_drive.

Dependencies: module_blocks module_comm module_common module_pwm_common module_sm sc_sncn_ctrlproto/module_ctrlproto 
sc_somanet-base/module_nodeconfig

See also:

- [Getting started with SOMANET](http://doc.synapticon.com/wiki/index.php/Category:Getting_Started_with_SOMANET)  

