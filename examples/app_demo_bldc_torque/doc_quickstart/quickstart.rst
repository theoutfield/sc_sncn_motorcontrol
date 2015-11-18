.. _SOMANET_Standalone_Torque_Control_Demo_Quickstart:

SOMANET Standalone Profile Torque Control Demo Quick Start Guide
================================================================

This demonstrative application illustrates usage of ``module_ctrl_loops`` and dependent modules to do a torque control of a brushless DC motor. Torque control loop is closed with a current feedback measured on phases B and C. 

Hardware setup
++++++++++++++

A minimal requirement for this application to run is having the *SOMANET* stack assembled consisting of the *SOMANET Core-C22*, *SOMANET Core* to xTAG-2 Debug Adapter, and *SOMANET IFM-Drive-DC100/300* modules. The stack should be powered via the *SOMANET IFM* board. An example of an assembled stack is shown in the image below. For the motor supplied with the kit the required power supply voltage should be 24 Volts. For the best experience please make sure that your stabilized DC power supply is capable of delivering more that 2 Amperes of power. Please mind that at high motor accelerations starting current may be as high as 10 times the nominal.     

.. figure:: images/assembly_p9.jpg
   :align: center

   Hardware Setup for standalone Torque Control Demo

To setup the system:

   #. If you don't have the stack assembled, assemble it as shown in the image above. Make sure to connect the IFM side of the *SOMANET Core* module to the IFM-DC100/300 board and COM side to the Core Debug Adapter (see markings on the Core module)
   #. Connect the xTAG-2 Adapter to the Core Debug Adapter.
   #. Connect the xTAG-2 to host PC. 
   #. Connect the motor supplied with the kit as shown in the image bellow.
   #. Connect the *IFM-DC100* board to a 24 V DC power supply
   #. Switch on the power supply. If everything is connected properly, drained current should not exceed 100mA. 

.. figure:: images/standalone_motorcontrol.jpg
   :align: center

   Connecting the motor and cables to your kit

Import and build the application
++++++++++++++++++++++++++++++++

   #. Open *xTIMEcomposer* Studio and check that it is operating in online mode. Open the edit perspective (Window->Open Perspective->XMOS Edit).
   #. Locate the ``'SOMANET Standalone Profile Torque Control Demo'`` item in the *xSOFTip* pane on the bottom left of the window and drag it into the Project Explorer window in *xTIMEcomposer*. This will also cause the modules on which this application depends to be imported as well. 
   #. Click on the ``app_demo_bldc_torque`` item in the Project Explorer plane then click on the build icon (hammer) in *xTIMEcomposer*. Check the Console window to verify that the application has built successfully. 

For help in using *xTIMEcomposer*, try the *xTIMEcomposer* tutorial, which you can find by selecting Help->Tutorials from the *xTIMEcomposer* menu.

Note that the Developer Column in *xTIMEcomposer* Studio on the right hand side of your screen provides information on the *xSOFTip* components you are using. Select the ``'SOMANET Motor Control loops'`` component in the Project Explorer, and you will see its description together with API documentation. Having done this, click the `back` icon until you return to this Quick Start Guide within the Developer Column.


Run the application
+++++++++++++++++++

When the application has been compiled, the next step is to run it on the *SOMANET Core* module using the tools to load the application over JTAG (via the xTAG-2 and Core Debug Adapter) into the xCORE multi-core micro-controller.

   #. Select the file ``app_demo_bldc_torque.xe`` in the ``app_demo_bldc_torque`` project from the Project Explorer.
   #. Click on the ``Run`` icon (the white arrow in the green circle). 
   #. At the ``Select Device`` dialog, select ``XMOS xTAG-2 connect to L1[0..3]`` and click ``OK``.
   #. The debug console window in *xTIMEcomposer* will not display any message. With the non modified application the rotor of the motor should should ramp up to reach a target torque value and then slow down till complete stop. After that it will start permanently rotating to an opposite direction with a very low set torque value. During execution of the program you are free to try blocking the rotor gently to feel the torque regulation.  
   
Next steps
++++++++++

As a next step you can try changing the target torque in the ``demo-bldc-torque.xc`` file located in the ``src`` folder of the app. In the function ``profile_torque_test`` change the variable ``target_torque`` to some other value. Note that if no load applied, with high target torque values the motor may reach its maximum velocity without being able to reach the desired torque value.

You might also try varying the ``torque_slope`` parameter. That parameter influencing how fast the target torque value is reached.

To get xscope display, enable XScope Real-Time Mode in ``Run Configurations``. You will get ``actual torque`` and ``target torque`` displayed in xscope.

Examine the code
................

   #. In *xTIMEcomposer* navigate to the ``src`` directory under ``app_demo_bldc_torque`` and double click on the ``demo-bldc-torque.xc`` file within it. The file will open in the central editor window.
   #. Find the main function and note that application runs one logical core on the COM_TILE (tile 0) for the user motor control application, one logical core on tile 2 for the main torque control loop, and six cores on the IFM_TILE for commutation, watchdog, and motor feedback sensor servers.
   #. Core 1:  Profile Torque Test Client function. It implements a user application for the profile torque control. ::

       profile_torque_test(c_torque_ctrl);

   #. Core 2: Torque Control Loop. This is the main torque control loop server for cyclic torque control mode. Some parameters have to be initialized prior starting the controller. ::

       torque_control( torque_ctrl_params, hall_config, qei_params, SENSOR_USED, c_adc, c_commutation_p1,  c_hall_p3,  c_qei_p3, c_torque_ctrl);
   
   #. Core 3: ADC Loop. It implements the ADC server to measure current in motor phases. ::

       adc_ad7949_triggered(c_adc, c_adctrig, clk_adc, p_ifm_adc_sclk_conv_mosib_mosia, p_ifm_adc_misoa, p_ifm_adc_misob);

   #. Core 4: PWM Loop. It implements the PWM Server. ::

       do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port, p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);

   #. Core 5: Motor Commutation loop. The main commutation loop that implements sinusoidal commutation. Some parameters have to be initialized prior starting the loop. ::

       commutation_sinusoidal(c_hall_p1,  c_qei_p1, c_signal, c_watchdog, c_commutation_p1, c_commutation_p2, c_commutation_p3, c_pwm_ctrl, p_ifm_esf_rstn_pwml_pwmh, p_ifm_coastn, p_ifm_ff1, p_ifm_ff2, hall_config, qei_params, commutation_params);

   #. Core 6: Watchdog Server. In case of application crash to prevent the hardware damages this server is required to constantly run. If the server is not running, the motor phases are disabled and no motor commutation is possible. ::

       run_watchdog(c_watchdog, p_ifm_wd_tick, p_ifm_shared_leds_wden);

   #. Core 7: Hall Server. Reads states of the motor Hall feedback sensor and calculates velocity and incremental position. Some parameters have to be initialized prior starting the server. ::

       run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6, p_ifm_hall, hall_config); 

   #. Core 8: QEI Server. Reads states of an incremental encoder feedback sensor in a quadrature mode and calculates velocity and incremental position. Some parameters have to be initialized prior starting the server. ::

       run_qei(c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5, c_qei_p6, p_ifm_encoder, qei_params);  


Now please have a closer look onto the ``profile_torque_test`` function that implements the torque profile and includes a linear ramp generator. First you will find already familiar variables that define desired movement parameters and parameters initialization functions. The variable ``torque_slope`` defines torque's ramping up and down parameter, i.e. they are equal. 
 
To start with the motion profile generation first you need to initialize control parameters. Please call the ``init_cst_param`` for that. After that you can call the profile torque controller ``set_profile_torque`` that takes as input the desired profile parameters and the target torque and executes the motion profile. 

You can get the torque feedback buy calling the ``get_torque`` method. In this demo application you can use XScope to monitor the feedback in real-time.  

