Digital GPIO Test
====================

test_gpio_digital.xc illustrates the usage of \ref module_gpio to configure GPIO's and 
read/write the digital ports. 

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

<br/>
<br/>
<br/>
<br/>
<br/>
<br/>

- \b THREADS: GPIO Server, GPIO Client 

- \b TILES:

	#define TILE_ONE 0
	#define IFM_TILE 3  

> **Do not forget to set properly your node and motor configuration when using this application**.

- [Configure your node]() 
- [How to configure your motors](https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/howto/HOW_TO_CONFIGURE_MOTORS.md)

\b TILE_ONE (0 by default): It takes care of the client side functions andcontrol loop. Since these functios do not require any port access, any free TILE could run them.

	on stdcore[TILE_ONE]:

- \b Thread: GPIO Client
	
	gpio_test(c_gpio_p1);

The Client function configures port 0 and port 1 as input switches and remaining 2 ports as outputs and shows how to read/write the 
digital ports. See more at \ref module_gpio.

It is not recommended  to run this thread in IFM_TILE together with the Server thread since the terminal output will slow down the GPIO Server thread and affect its performance.

\b IFM_TILE (3 by default). It executes the server side functions, controlling the interfaces. These functions need access to the Interface Module (IFM), just the tile that provides access to the \b IFM ports can run these functions.  

	on stdcore[IFM_TILE]: 

- \b Thread: GPIO Server
			
	gpio_digital_server(p_ifm_ext_d, c_gpio_p1, c_gpio_p2);

It access the GPIO ports at the IFM module. See more at \ref module_gpio.


Other dependencies: sc_somanet-base/module_nodeconfig \ref module_common

\b See \b also:

- [Getting started with SOMANET](http://doc.synapticon.com/wiki/index.php/Category:Getting_Started_with_SOMANET)  