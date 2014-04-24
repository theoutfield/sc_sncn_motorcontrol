test_velocity-ctrl.xc illustrates usage of \ref module_ctrl_loops to do velocity control of a motor. Velocity Loop can be closed with position from either HALL sensor/ QEI Sensor or any other position sensor (if a drive server runs on IFM tile).

<table class="core_usage" align="center" cellpadding="5" width="20%">
<tr>
    <th colspan="2">CORE use</th>
</tr>
<tr>
    <td>Parallel \b THREADS</td>
    <td width="30px" align="center"> 7 </td>
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

- \b THREADS: Profile Velocity, Velocity Control Loop, PWM Server, Commutation Server, Hall Server, QEI Server, Watchdog Server


- \b TILES:

	#define TILE_ONE 0
	#define IFM_TILE 3

\b TILE_ONE (0 by default): It takes care of the client side functions and control loop. Since these functions do not require any port access, any free TILE could run them.

	on stdcore[TILE_ONE]:
				
- \b Thread: Homing Test

	profile_velocity_test(c_velocity_ctrl);	// Test Velocity Profile Mode on slave side

Set new target velocity for the controller. Read more at \ref module_profile.


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

\b IFM_TILE (3 by default). It executes the server side functions, controlling the interfaces. These functions need access to the Interface Module (IFM), just the tile that provides access to the \b IFM ports can run these functions.  


	on stdcore[IFM_TILE]:
				
- \b Thread: PWM Server.

	do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port, p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);

Responsible for generating a Pulse-Width Modulation signal that drives the motors. Provided by the \b module_pwm_symmetrical at PWM software component \b sc_pwm.

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

- \b Thread: Hall Server

	hall_par hall_params;
	init_hall_param(hall_params);
	run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5,
			p_ifm_hall, hall_params); //Channel priority 1,2..5

To obtain information about motor position for position control loop, its use is mandatory since the motor commutation is Hall-based. Read more at \ref module_hall.

- \b Thread: QEI Server

	qei_par qei_params;
	init_qei_param(qei_params);
	run_qei(c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5,
			p_ifm_encoder, qei_params);  	// channel priority 1,2..5

To obtain high precision information about motor position. Read more at \ref module_qei.

- \b Thread: Watchdog Server

	run_watchdog(c_watchdog, p_ifm_wd_tick, p_ifm_shared_leds_wden);

A watchdog server is used to monitor IFM_TILE and disables motor in case of emergency. Read more at \ref module_watchdog.

\b Please, do not forget to set properly your node and motor configuration when using this application.

- <a href="">Configure your node</a> 
- \ref how_configure_motors

More information about Velocity Control Server/Client can be found at \ref module_ctrl_loops documentation.

Other dependencies: \ref sc_pwm/module_pwm_common \ref sc_somanet-base/module_nodeconfig \ref module_common  \ref module_sm 

\b See \b also:

- <a href="http://doc.synapticon.com/wiki/index.php/Category:Getting_Started_with_SOMANET">Getting started with SOMANET</a>  
