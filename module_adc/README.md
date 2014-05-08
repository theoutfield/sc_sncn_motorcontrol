ADC Module
=======================
<a href="https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/SYNAPTICON.md">
<img align="left" src="https://s3-eu-west-1.amazonaws.com/synapticon-resources/images/logos/synapticon_fullname_blackoverwhite_280x48.png"/>
</a>
<br/>
<br/>

The module provides ADC server which drives the ADC included within the IFM Drive DC 100/300 boards. 
The server acquires analog input data in a loop.

The module also  and provides client functions to obtain data from the existing ADC server.

Demos:
- [test_adc_external_input.xc](https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/test_adc_external_input/src/test_adc_external_input.xc)

###**Quick API** 
For a better review of all the available functions, check the header files.

* [adc_server_ad7949.h](https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/module_adc/include/adc_server_ad7949.h) - To access the server side functions.
* [adc_client_ad7949.h](https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/module_adc/include/adc_client_ad7949.h) - To access the client side functions.

#### **adc_server_ad7949.h**####

- **Server Loop:** This is the interface to AD7949 ADC devices. It controls two devices so that two channels can be sampled simultaneously. This server is not intended to be used for motor control. 

> TILE constrains: IFM* (need access to IFM ports)

```
void adc_ad7949( chanend c_adc, clock clk, buffered out port:32 p_sclk_conv_mosib_mosia,
		     in buffered port:32 p_data_a, in buffered port:32 p_data_b );
```
* Parameters
	* *c_adc* Channel to communicate to the client function
	* *clk* Clock for the ADC device serial port
	* *p_sclk_conv_mosib_mosia* 4-bit port for ADC control interface
	* *p_data_a* 1-bit port for ADC data channel 0
	* *p_data_b* 1-bit port for ADC data channel 1


#### **adc_client_ad7949.h**####

- **Client function**: Get external analog sensor value from the server
```
{int, int} get_adc_external_ad7949(chanend c_adc);
```
* Parameters
	* *c_adc* Channel to communicate to the server function
* Return 
	* Sensed value on channel 0
	* Sensed value on channel 1




####**See also**:

- [How to include a module in your application]()
- [Getting started with SOMANET][getting_started_somanet]    



*For Core C22, IFM Tile is located on TILE 3. For Core C21, IFM Tile is on TILE 1.

[getting_started_somanet]: http://doc.synapticon.com/wiki/index.php/Category:Getting_Started_with_SOMANET
