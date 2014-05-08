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

###**Quick API** 

#### **comm_loop_server.h**####

- **Commutation server:** 

> TILE constrains: IFM* (need access to IFM ports)

```
void commutation_sinusoidal(chanend c_hall, chanend c_qei, chanend c_signal, chanend c_watchdog, 
	chanend c_commutation_p1, chanend c_commutation_p2, chanend c_commutation_p3, chanend c_pwm_ctrl,
	out port p_ifm_esf_rstn_pwml_pwmh, port p_ifm_coastn, port p_ifm_ff1, port p_ifm_ff2,
	hall_par &hall_params, qei_par &qei_params, commutation_par &commutation_params);
```
* Parameters
	
* Return 

#### **comm_loop_client.h**####


- **Parameters initialization:** 
```
void init_commutation_param(commutation_par &commutation_params, hall_par &hall_params, int nominal_speed);
```
* Parameters
	
* Return 

- **Commutation loop initialization:**
```
int init_commutation(chanend c_signal);

```
* Parameters
	
* Return 

For a better review of all the available functions, check the header files.

* [comm_loop_server.h](https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/module_commutation/include/comm_loop_server.h)
* [comm_loop_client.h](https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/module_commutation/include/comm_loop_client.h)

####**See also**:

- [How to include a module in your application]()
- [Getting started with SOMANET][getting_started_somanet]    


*For Core C22, IFM Tile is located on TILE 3. For Core C21, IFM Tile is on TILE 1.

[getting_started_somanet]: http://doc.synapticon.com/wiki/index.php/Category:Getting_Started_with_SOMANET
