Commutation Module
=======================
<a href="https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/SYNAPTICON.md">
<img align="left" src="https://s3-eu-west-1.amazonaws.com/synapticon-resources/images/logos/synapticon_fullname_blackoverwhite_280x48.png"/>
</a>
<br/>
<br/>

This module provides driver for the BLDC Motor connected to the interface module (IFM). 
The module consists of commutation which internally makes use of the predriver to 
drive fets and configurations under pwm. The module provides Commutation server thread 
which acquires position information from the Hall server and commutates the motor 
in a while loop; and provides client functions to optimize motor commutation with 
commutation offsets, motor winding types, nominal motor speed and number of pole pairs; 
set input voltage for the motor, get fet_state from the Commutation Server.

To include this module add module_commutation to USED_MODULES in the application/test
makefile, and include header files: comm_loop_server.h and comm_loop_client.h

###**Quick API** 

#### **comm_loop_server.h**####



#### **comm_loop_client.h**####


- **Initialization function:** Initialize commutation parameters
```
void init_commutation_param(commutation_par &commutation_params, hall_par &hall_params, int nominal_speed);
```
* Parameters
	
* Return 


```
int init_commutation(chanend c_signal);

```
* Parameters
	
* Return 


####**See also**:

- [How to include a module in your application]()
- [Getting started with SOMANET][getting_started_somanet]    



*For Core C22, IFM Tile is located on TILE 3. For Core C21, IFM Tile is on TILE 1.

[getting_started_somanet]: http://doc.synapticon.com/wiki/index.php/Category:Getting_Started_with_SOMANET
