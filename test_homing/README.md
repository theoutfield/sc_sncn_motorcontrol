test_homing.xc illustrates the usage of \ref module_homing. A homing implementation based on limit switches and a home switch.
In this application we have home switch on port 0 of GPIO D and positive limit switch on port 1 of GPIO D. Here we use velocity controller to search for the home switch and reset the position counters with offset found.

<table class="core_usage" align="center" cellpadding="5" width="20%">
<tr>
    <th colspan="2">CORE use</th>
</tr>
<tr>
    <td>Parallel \b THREADS</td>
    <td width="30px" align="center"> 9 </td>
</tr>
<tr>
    <td>\b TILES used</td>
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

<br/>
<br/>
<br/>
<br/>
<br/>
<br/>

- \b THREADS: Homing Test, Velocity Control Loop, PWM Server, ADC Server, Commutation Server, Hall Server, QEI Server, Watchdog Server, GPIO Server . 

- \b TILES:

	#define TILE_ONE 0
	#define IFM_TILE 3 

\b TILE_ONE (0 by default): It takes care of the test function and control loop. Since these functions do not require any port access, any free TILE could run them.

	on stdcore[TILE_ONE]:

- \b Thread: Homing Test

	homing(c_qei_p3, c_gpio_p1, c_velocity_ctrl);

It monitors the home and limit switches; and upon encountering the home switch, it stores the position, stops the motor and updates the position counter with the offset from stop position.  Read more at \ref module_homing.

- \b Thread: Velocity Control Loop

	ctrl_par velocity_ctrl_params; //Var definition
	filter_par sensor_filter_params;
	hall_par hall_params;
	qei_par qei_params;

	init_velocity_control_param(velocity_ctrl_params); //Initialization
	init_qei_param(qei_params);
	init_hall_param(hall_params);
	init_sensor_filter_param(sensor_filter_params);

	velocity_control(velocity_ctrl_params, sensor_filter_params, hall_params, 
			qei_params, SENSOR_USED, c_hall_p2, c_qei_p2, 
			c_velocity_ctrl, c_commutation_p2); //Control loop
		
Read back actual velocity of the motor. Read more at \ref module_ctrl_loops or \ref module_blocks.

\b IFM_TILE (3 by default): It executes the server side functions, controlling the interfaces. These functions need access to the Interface Module (IFM), just the tile that provides access to the \b IFM ports can run these functions.  


	on stdcore[IFM_TILE]:

- \b Thread: ADC Server.
	
	adc_ad7949_triggered(c_adc, c_adctrig, clk_adc, p_ifm_adc_sclk_conv_mosib_mosia,
				p_ifm_adc_misoa, p_ifm_adc_misob);
	
It captures current values in the motor phases. Read more at \ref module_adc.		
	
- \b Thread: PWM Server.

	do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port, p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);

Responsible for generating a Pulse-Width Modulation signal that drives the motors. Provided by the \b module_pwm_symmetrical at PWM software component \b sc_pwm.

- \b Thread: GPIO Server
			
	gpio_digital_server(p_ifm_ext_d, c_gpio_p1, c_gpio_p2);

See more at \ref module_gpio.


- \b Thread: Commutation Server 

	hall_par hall_params; //Var definition
	qei_par qei_params;
	commutation_par commutation_params;

	init_hall_param(hall_params); //Initialization
	init_qei_param(qei_params);
	init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED); 

	commutation_sinusoidal(c_hall_p1,  c_qei_p2, c_signal, c_watchdog, c_commutation_p1,
				c_commutation_p2, c_commutation_p3, c_pwm_ctrl, hall_params,
				qei_params, commutation_params); //Read feedback

Responsible for proper BLDC motor drive. Read more at \ref module_commutation.

- \b Thread: Hall Sensor Server

	hall_par hall_params;
	init_hall_param(hall_params);
	run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5,
		c_hall_p6, p_ifm_hall, hall_params); // channel priority 1,2..6

To obtain information about motor position for position control loop in case a Hall sensor is used. Read more at \ref module_hall.

- \b Thread: QEI Server

	qei_par qei_params;
	init_qei_param(qei_params);
	run_qei(c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5,
			p_ifm_encoder, qei_params);  	// channel priority 1,2..5

To obtain high precision information about motor position for position control loop in case a Quadrature Encoder sensor is used. Read more at \ref module_qei.

- \b Thread: Watchdog Server

	run_watchdog(c_watchdog, p_ifm_wd_tick, p_ifm_shared_leds_wden);

A watchdog server is used to monitor IFM_TILE and disables motor in case of emergency. Read more at \ref module_watchdog.

\b Please, do not forget to set properly your node and motor configuration when using this application.

- <a href="">Configure your node</a> 
- \ref how_configure_motors

More information about GPIO Server/ Client can be found at \ref module_gpio documentation.

Other dependencies: \ref module_common \ref module_profile \ref module_sm sc_somanet-base/module_nodeconfig

\b See \b also:

- <a href="http://doc.synapticon.com/wiki/index.php/Category:Getting_Started_with_SOMANET">Getting started with SOMANET</a>  