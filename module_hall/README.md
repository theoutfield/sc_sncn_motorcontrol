Hall Sensor Module
=======================
<a href="https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/SYNAPTICON.md">
<img align="left" src="https://s3-eu-west-1.amazonaws.com/synapticon-resources/images/logos/synapticon_fullname_blackoverwhite_280x48.png"/>
</a>
<br/>
<br/>

This module provides driver for the Hall sensor connected to the interface module (IFM).
The module provides Hall server thread which drives the Hall sensor, acquires position 
information and calculates velocity in a while loop; and provides client functions to
configure Hall Server with number of pole pairs and max ticks; get position and velocity 
from the Hall server.

###**Quick API** 

#### **hall_server.h**####

- **Server loop:** 

> TILE constrains: IFM* (need access to IFM ports)

```
void run_hall(chanend c_hall_p1, chanend c_hall_p2, chanend c_hall_p3, chanend c_hall_p4, 
	chanend c_hall_p5, chanend c_hall_p6, port in p_hall, hall_par &hall_params);
```
* Parameters
	
* Return 

#### **hall_client.h**####

- **Get position value:**
```
int get_hall_position(chanend c_hall);

```
* Parameters
	
* Return 

- **Get velocity value:** 
```
int get_hall_velocity(chanend c_hall, hall_par &hall_params);
```
* Parameters
	
* Return 


For a better review of all the available functions, check the header files.

* [hall_server.h](https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/module_hall/include/hall_server.h)
* [hall_client.h](https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/module_hall/include/hall_client.h)

####**See also**:

- [How to include a module in your application]()
- [Getting started with SOMANET][getting_started_somanet]    



*For Core C22, IFM Tile is located on TILE 3. For Core C21, IFM Tile is on TILE 1.

[getting_started_somanet]: http://doc.synapticon.com/wiki/index.php/Category:Getting_Started_with_SOMANET


