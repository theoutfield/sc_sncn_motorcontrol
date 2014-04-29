Digital GPIO Test
====================
<a href="https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/SYNAPTICON.md">
<img align="left" src="https://s3-eu-west-1.amazonaws.com/synapticon-resources/images/logos/synapticon_fullname_blackoverwhite_280x48.png"/>
</a>
<br/>
<br/>

test_gpio_digital.xc illustrates the usage of \ref module_gpio to configure GPIO's and read/write the digital ports. 

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
    <td rowspan="2" align="center"> C21-DX <br/> C22 </td>
    <td rowspan="2" align="center"> Drive DC 100 <br/> Drive DC 300</td>
</tr>
</table>


- **THREADS**: GPIO Server, GPIO Client 

- **TILES**:

	#define TILE_ONE 0
	#define IFM_TILE 3  

> **Do not forget to set properly your node and motor configuration when using this application**.

- [Configure your node]() 
- [How to configure your motors][how_to_configure_motors]

**TILE_ONE** 
This tile (0 by default) takes care of the client side functions andcontrol loop. Since these functios do not require any port access, any free TILE could run them.

	on stdcore[TILE_ONE]:

- **Thread**: GPIO Client
	
	gpio_test(c_gpio_p1);

The Client function configures port 0 and port 1 as input switches and remaining 2 ports as outputs and shows how to read/write the 
digital ports. See more at \ref module_gpio.

It is not recommended  to run this thread in IFM_TILE together with the Server thread since the terminal output will slow down the GPIO Server thread and affect its performance.

**IFM_TILE** 
This tile, 3 by default, executes the server side functions, controlling the interfaces. These functions need access to the Interface Module (IFM), just the tile that provides access to the \b IFM ports can run these functions.  

	on stdcore[IFM_TILE]: 

- **Thread**: GPIO Server
			
	gpio_digital_server(p_ifm_ext_d, c_gpio_p1, c_gpio_p2);

It access the GPIO ports at the IFM module. See more at \ref module_gpio.


Other dependencies: sc_somanet-base/module_nodeconfig \ref module_common

**See also**:

- [Getting started with SOMANET][getting_started_somanet]    

[sc_sncn_ethercat]:https://github.com/synapticon/sc_sncn_ethercat
[sc_pwm]: https://github.com/synapticon/sc_pwm

[module_adc]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_adc
[module_hall]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_hall
[module_watchdog]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_watchdog
[modle_ecat_drive]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_ecat_drive
[module_ctrl_loops]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_ctrl_loops
[module_blocks]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_blocks
[module_qei]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_qei
[module_commutation]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_commutation
[module_gpio]: https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_gpio

[module_ethercat]: https://github.com/synapticon/sc_sncn_ethercat/tree/master/module_ethercat

[module_pwm_symmetrical]: https://github.com/synapticon/sc_pwm/tree/master/module_pwm_symmetrical

[how_to_configure_motors]: https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/howto/HOW_TO_CONFIGURE_MOTORS.md
[getting_started_somanet]: http://doc.synapticon.com/wiki/index.php/Category:Getting_Started_with_SOMANET
