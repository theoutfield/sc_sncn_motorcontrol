General Purpose I/O Module
=======================
<a href="https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/SYNAPTICON.md">
<img align="left" src="https://s3-eu-west-1.amazonaws.com/synapticon-resources/images/logos/synapticon_fullname_blackoverwhite_280x48.png"/>
</a>
<br/>
<br/>

This module provides driver for the GPIO digital ports on the interface module (IFM).
The module provides GPIO server thread which configures the GPIO ports; read/write GPIO 
digital ports in a while loop; and provides client functions to configure ports and 
read/write ports. 

Demo:
* [test_gpio_digital.xc](https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/test_gpio_digital/src/test_gpio_digital.xc)

For a better review of all the available functions, check the header files.

* [gpio_server.h](https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/module_gpio/include/gpio_server.h)
* [gpio_client.h](https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/module_gpio/include/gpio_client.h)

###**Quick API** 

#### **gpio_server.h**####

- **Server loop:** 

> TILE constrains: IFM* (need access to IFM ports)

```
void gpio_digital_server(port p_ifm_ext_d[], chanend c_gpio_0, chanend c_gpio_1);
```
* Parameters
	
* Return 

#### **gpio_client.h**####

- **Read on GPIO:** 
```
int read_gpio_digital_input(chanend c_gpio, int port_number);
```
* Parameters
	
* Return 

- **Write on GPIO:** 
```
void write_gpio_digital_output(chanend c_gpio, int port_number, int port_value);
```
* Parameters
	
* Return

####**See also**:

- [How to include a module in your application]()
- [Getting started with SOMANET][getting_started_somanet]    


*For Core C22, IFM Tile is located on TILE 3. For Core C21, IFM Tile is on TILE 1.

[getting_started_somanet]: http://doc.synapticon.com/wiki/index.php/Category:Getting_Started_with_SOMANET
