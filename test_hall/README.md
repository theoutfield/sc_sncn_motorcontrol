Hall Sensor Test
=========================
<a href="https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/SYNAPTICON.md">
<img align="left" src="https://s3-eu-west-1.amazonaws.com/synapticon-resources/images/logos/synapticon_fullname_blackoverwhite_280x48.png"/>
</a>
<br/>
<br/>

[test_hall.xc]() illustrates usage of [module_hall][module_hall] to get position and velocity information.

<table align="center" cellpadding="5" width="80%">
<tr>
    <th colspan="2">CORE use</th>
    <td rowspan="3" width="1px"></td>
    <th colspan="3">HW compatibility</th>
</tr>
<tr>
    <td>Parallel THREADS</td>
    <td width="30px" align="center"> 2 </td>

    <th align="center">COM</th>
    <th align="center">CORE</th>
    <th align="center">IFM</th>
</tr>
<tr>
    <td>TILES used</td>
    <td width="30px" align="center"> 2 </td>

    <td rowspan="2" align="center">*</td>
    <td rowspan="2" align="center">C21-DX <br/> C22 </td>
    <td rowspan="2" align="center">Drive DC 100 <br/> Drive DC 300</td>
</tr>
</table>

- \b THREADS: Hall Sensor Server, Hall Sensor Client

- \b TILES:
```
	#define TILE_ONE 0
	#define IFM_TILE 3 
```

> **Do not forget to set properly your node and motor configuration when using this application**.

- [Configure your node]() 
- [How to configure your motors](https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/howto/HOW_TO_CONFIGURE_MOTORS.md)

###**TILE_ONE** 
This tile (0 by default) takes care of the client side function. Since these functions do not require any port access, any free TILE could run them.
```
	on stdcore[TILE_ONE]:
```
- **Thread**: Hall Sensor Client
```
	hall_test(c_hall_p1);
```
The client reads position and velocity from HALL Server. See more at [module_hall][module_hall].

It is not recommended  to run this thread in IFM_TILE together with the Server thread since the terminal output will slow down the GPIO Server thread and affect its performance.

**IFM_TILE** 
This tile (3 by default) executes the server side functions, controlling the interfaces. These functions need access to the Interface Module (IFM), just the tile that provides access to the IFM ports can run these functions.  
 
```	
	on stdcore[IFM_TILE]: 
```
- **Thread**: Hall Sensor Server
```
	hall_par hall_params;
	init_hall_param(hall_params);
	run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5,
		c_hall_p6, p_ifm_hall, hall_params); // channel priority 1,2..6
```
The Hall Server captures signal values from the sensors. See more at [module_hall][module_hall].


More information about Hall Server/ Client can be found at [module_hall][module_hall] documentation.

Other dependencies: [module_blocks][module_blocks] [module_common][module_common] [module_nodeconfig][module_nodeconfig]@[sc_somanet-base][sc_somanet-base]


**See also**:

- [Getting started with SOMANET][getting_started_somanet]    


[sc_sncn_ethercat]:https://github.com/synapticon/sc_sncn_ethercat
[sc_pwm]: https://github.com/synapticon/sc_pwm
[sc_somanet-base]: https://github.com/synapticon/sc_somanet-base

[module_adc]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_adc
[module_hall]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_hall
[module_watchdog]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_watchdog
[modle_ecat_drive]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_ecat_drive
[module_ctrl_loops]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_ctrl_loops
[module_blocks]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_blocks
[module_qei]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_qei
[module_commutation]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_commutation
[module_gpio]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_gpio
[module_common]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_common
[module_sm]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_sm
[module_homing]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_homing
[module_profile]:https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_profile

[module_ethercat]: https://github.com/synapticon/sc_sncn_ethercat/tree/master/module_ethercat

[module_pwm_symmetrical]: https://github.com/synapticon/sc_pwm/tree/master/module_pwm_symmetrical

[module_nodeconfig]: https://github.com/synapticon/sc_somanet-base/tree/master/module_nodeconfig

[how_to_configure_motors]: https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/howto/HOW_TO_CONFIGURE_MOTORS.md
[getting_started_somanet]: http://doc.synapticon.com/wiki/index.php/Category:Getting_Started_with_SOMANET
