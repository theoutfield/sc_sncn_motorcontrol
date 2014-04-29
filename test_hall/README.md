Hall Sensor Test
=========================

test_hall.xc illustrates usage of \ref module_hall to get position and velocity information.

<table class="core_usage" align="center" cellpadding="5" width="20%">
<tr>
    <th colspan="2">CORE use</th>
</tr>
<tr>
    <td>Parallel \b THREADS</td>
    <td width="30px" align="center"> 2 </td>
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

- \b THREADS: Hall Sensor Server, Hall Sensor Client

- \b TILES:
```
	#define TILE_ONE 0
	#define IFM_TILE 3 
```

> **Do not forget to set properly your node and motor configuration when using this application**.

- [Configure your node]() 
- [How to configure your motors](https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/howto/HOW_TO_CONFIGURE_MOTORS.md)

\b TILE_ONE (0 by default): It takes care of the client side functions andcontrol loop. Since these functios do not require any port access, any free TILE could run them.
```
	on stdcore[TILE_ONE]:
```
- \b Thread: Hall Sensor Client
```
	hall_test(c_hall_p1);
```
The client reads position and velocity from HALL Server. See more at \ref module_hall.

It is not recommended  to run this thread in IFM_TILE together with the Server thread since the terminal output will slow down the GPIO Server thread and affect its performance.

\b IFM_TILE (3 by default). It executes the server side functions, controlling the interfaces. These functions need access to the Interface Module (IFM), just the tile that provides access to the \b IFM ports can run these functions.  
```	
	on stdcore[IFM_TILE]: 
```
- \b Thread: Hall Sensor Server
```
	hall_par hall_params;
	init_hall_param(hall_params);
	run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5,
		c_hall_p6, p_ifm_hall, hall_params); // channel priority 1,2..6
```
The Hall Server captures signal values from the sensors. See more at \ref module_hall.


More information about Hall Server/ Client can be found at \ref module_hall documentation.

Other dependencies: sc_somanet-base/module_nodeconfig \ref module_blocks \ref module_common

\b See \b also:

- [Getting started with SOMANET](http://doc.synapticon.com/wiki/index.php/Category:Getting_Started_with_SOMANET)  