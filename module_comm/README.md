Communication Module
=======================
<a href="https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/SYNAPTICON.md">
<img align="left" src="https://s3-eu-west-1.amazonaws.com/synapticon-resources/images/logos/synapticon_fullname_blackoverwhite_280x48.png"/>
</a>
<br/>
<br/>

This module provides functions to read/write data to ctrlproto data structure which
inturn is passed to ethercat Master applications; functions to read SDO (software 
defined objects used for motor/sensor configuration); functions to update respective 
Servers running on interface module (IFM_CORE).

###**Quick API** 

#### **comm.h**####

**Target velocity from EtherCAT:** 

> TILE constrains: IFM* (need access to IFM ports)

```
int get_target_velocity(ctrl_proto_values_t InOut);

```
* Parameters
	
* Return 


**Target position from EtherCAT:** 
```
int get_target_position(ctrl_proto_values_t InOut);

```
* Parameters
	
* Return 


**Target torque from EtherCAT:** 
```
int get_target_position(ctrl_proto_values_t InOut);

```
* Parameters
	
* Return 


**Send actual torque value:** 
```
void send_actual_torque(int actual_torque, ctrl_proto_values_t &InOut);
```
* Parameters
	
* Return 

**Send actual velocity value:** 
```
void send_actual_velocity(int actual_velocity, ctrl_proto_values_t &InOut);

```
* Parameters
	
* Return 

**Send actual position value:** 
```
void send_actual_position(int actual_position, ctrl_proto_values_t &InOut);

```
* Parameters
	
* Return 

For a better review of all the available functions, check the header file.

* [comm.h](https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/module_comm/include/comm.h)

####**See also**:

- [How to include a module in your application]()
- [Getting started with SOMANET][getting_started_somanet]    


[getting_started_somanet]: http://doc.synapticon.com/wiki/index.php/Category:Getting_Started_with_SOMANET
