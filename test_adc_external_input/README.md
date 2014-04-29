External ADC Input Test
============================

**test_adc_external_input.xc** illustrates usage of [module_adc]() to get external analog sensor input values. By default all analog inputs are configured as differential only.

<p align="center">
<table class="core_usage" align="center" cellpadding="5" width="80%">
<tr>
    <th colspan="2">CORE use</th>
    <td rowspan="3" width="5px"></td>
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
    <td rowspan="2" align="center">C21-DX <br/> C22 </td>
    <td rowspan="2" align="center">Drive DC 100 <br/> Drive DC 300</td>
</tr>
</table>
</p>


- **THREADS**: ADC Client Side, ADC Server Side.
- **TILES**:
```objectivec 
	#define TILE_ONE 0
	#define IFM_TILE 3
```
> **Do not forget to set properly your node and motor configuration when using this application**.

- [Configure your node]() 
- [How to configure your motors](https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/howto/HOW_TO_CONFIGURE_MOTORS.md)


###TILE_ONE
This tile (0 by default) takes care of the client side functions . Since these functions do not require any port access, any free TILE could run them.
```objectivec 
	on stdcore[TILE_ONE]:
```
- **Thread**: ADC Client
```objectivec 
	adc_test(c_adc);
```
Read and print on the console the readed values on both ADC ports. Read more at \ref module_adc.

###IFM_TILE 
This tile (3 by default) executes the server side functions, controlling the interfaces. These functions need access to the Interface Module (IFM), just the tile that provides access to the **IFM ports can run these functions. 
```objectivec 
	on stdcore[IFM_TILE]: 
```
- **Thread**: ADC Server
```objectivec 
	adc_ad7949( c_adc, clk_adc, p_ifm_adc_sclk_conv_mosib_mosia, p_ifm_adc_misoa, p_ifm_adc_misob );
```
Interfaces the ADC and provide the readed values to the client side thread. Read more at \ref module_adc.

More information about ADC module can be found at \ref module_adc documentation.

**See also**:

- [Getting started with SOMANET](http://doc.synapticon.com/wiki/index.php/Category:Getting_Started_with_SOMANET)   


