.. _adc_demo:

================================
ADC DEMO
================================

.. contents:: In this document
    :backlinks: none
    :depth: 3

In many applications, it is required to measure analogue signals. These measured signals can later be used for controlling/demonstration purposes.
The purpose of this app (app_demo_adc) is to show how an ADC block can be used on a SOMANET Drive module. It includes one of simplest forms of using an adc block in
SOMANET Drive module. 


* **Minimum Number of Cores**: 2
* **Minimum Number of Tiles**: 1

.. cssclass:: github

  `See Application on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/examples/app_demo_adc/>`_

Quick How-to
============
1. :ref:`Assemble your SOMANET device <assembling_somanet_node>`.
2. Wire up your device. Check how at your specific :ref:`hardware documentation <hardware>`. Connect your analogue inputs, power supply cable, and XTAG.
3. :ref:`Set up your XMOS development tools <getting_started_xmos_dev_tools>`. 
4. Download and :ref:`import in your workspace <getting_started_importing_library>` the SOMANET Motor Control Library and its dependencies.
5. Open the **main.xc** within  the **app_demo_adc**. Include the :ref:`board-support file according to your device <somanet_board_support_module>`. Also set the :ref:`appropriate target in your Makefile <somanet_board_support_module>`.

.. important:: Make sure the SOMANET Motor Control Library supports your SOMANET device. For that, check the :ref:`Hardware compatibility <motor_control_hw_compatibility>` section of the library.

6. :ref:`Set the type of your ADC block (AD_7949 or AD_7265) as the second input of adc_client_demo function in the main.xc file (ADC block type is defined as SOMANET_DRIVE_ADC value in bsp file of your Drive module in module-board-support).
 
.. code-block:: c
  
               // ADC interface
               interface ADCInterface i_adc[2];

               par
               {
                   on tile[APP_TILE]:
                   {
                        adc_client_demo(i_adc[1], AD_7949/*or AD_7265*/);
                   }

                   on tile[IF2_TILE]:
                   {
                        if(!isnull(adc_ports.ad7949_ports.clk))         adc_ad7949_service_demo(adc_ports.ad7949_ports, i_adc);
                        else if(!isnull(adc_ports.ad7265_ports.xclk))   adc_ad7265_service_demo(adc_ports.ad7265_ports, i_adc);
                   }
               }                

7. :ref:`By default, all possible combinations of analogue inputs will be measured by adc block. However, you can enable/disable their xscope probes in config.xscope file of this application (app_demo_adc).

8. :ref:`Run the application enabling XScope <running_an_application>`.

.. seealso:: Did everything go well? If you need further support please check out our `forum <http://forum.synapticon.com/>`_.
