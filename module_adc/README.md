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

TILE constrains: IFM* (need access to IFM ports)

Demos:
- [test_adc_external_input.xc](https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/test_adc_external_input/src/test_adc_external_input.xc)

###**QUICK API** 
For a better review of all the available functions, check the header files.

* [adc_server_ad7949.h](https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/module_adc/include/adc_server_ad7949.h) - To access the server side functions.
* [adc_client_ad7949.h](https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/module_adc/include/adc_client_ad7949.h) - To access the client side functions.

#### **adc_server_ad7949.h**####

- **Server Initialization function:**
This is the interface to AD7949 ADC devices. It controls two devices so that two channels can be sampled simultaneously. This server is not intended to be used for motor control. 

```
void adc_ad7949( chanend c_adc, clock clk, buffered out port:32 p_sclk_conv_mosib_mosia,
		     in buffered port:32 p_data_a, in buffered port:32 p_data_b );
```
Parameters
	* c_adc
	* clk
	* p_sclk_conv_mosib_mosia
	* p_data_a
	* p_data_b


#### **adc_client_ad7949.h**####

- **Client function**:
```
{int, int} get_adc_external_ad7949(chanend c_adc);
```
**See also**:

- [How to include a module in your application]()
- [Getting started with SOMANET][getting_started_somanet]    



*For Core C22, IFM Tile is located on TILE 3. For Core C21, IFM Tile is on TILE 1.

[getting_started_somanet]: http://doc.synapticon.com/wiki/index.php/Category:Getting_Started_with_SOMANET
