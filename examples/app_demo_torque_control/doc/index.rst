.. _app_demo_torque_control:

======================================================
Torque Control Demo
======================================================

.. contents:: In this document
    :backlinks: none
    :depth: 3

Introduction
================

Electric motors are used in many motion control applications. In all of these applications, the final duty of the electric motor is to generate the required torque. In speed controlled applications the reference value of torque is calculated by speed controller. A similar situation is existing in the case of position controlled application. It is also possible to directly set the reference value of the torque by the user. The purpose of this application is to let the user directly work with torque controller.
By running this application, the user will be able to:

- switch between different operational modes (enabled/disabled torque controller, safe torque off mode)
- directly set the reference value of torque in [milli Nm]
- automatically find, read and set the commutation offset for position sensor
- observe different parameters of torque controller and position feedback on xscope
 
After starting the application, the torque controller will be activated automatically (with reference torque equal to 0), and the user can directly send the required torque command in [milli-Nm].

Figure 1 gives a general view of torque controller application including different blocks of your software and hardware modules.

.. image:: images/photo.jpg
   :width: 100%

**Fig. 1: General structure of your software/hardware modules within torque control application**

**important**

- Before sending the reference torque it is recommended to find and set the commutation sensor offset
- If you are using an electric brake, release the brake before applying a new torque command
- It is recommended to start the application with lower voltages (such as 16V, and increase the voltage to its nominal value after your hardware is checked)

* **Minimum Number of Cores**: 6
* **Minimum Number of Tiles**: 2

Quick How-to
============
**Important**

1. :ref:`Assemble your SOMANET device <assembling_somanet_node>`.

2. Wire up your device. Check how at your specific :ref:`hardware documentation <hardware>`. Connect your position sensor, motor terminals, power supply cable, and XTAG. Power up!

    **important**
    For safety please use a current limited power supply and check current consumption.

3. :ref:`Set up your XMOS development tools <getting_started_xmos_dev_tools>`. 

4. Download and :ref:`import in your workspace <getting_started_importing_library>` the SOMANET Motor Control Library and its dependencies.

5. Open the **main.xc** within  the **app_demo_general_pwm**. Include the :ref:`board-support file according to your device <DRIVE_BOARD_REQUIRED>`, and Include the :ref:`Core-support file according to your Core module <CORE_BOARD_REQUIRED>`. Moreover, set the :ref:`appropriate core target in your Makefile.

    **important** Make sure the SOMANET Motor Control Library supports your SOMANET device. For that, check the :ref:`Hardware compatibility <motor_control_hw_compatibility>` section of the library.
    
6. If you are using an electric brake, update the settings of your electric brake in file **user_interface_service.xc** of your application. These settings can also be changed while the application is running.

    .. code-block:: c

		    pull_brake_voltage= 16000; //milli-Volts
		    hold_brake_voltage=  1000; //milli-Volts
		    pull_brake_time   =  2000; //milli-Seconds

7. :ref:`Set the configuration <motor_configuration_label>` for Motor Control, position sensor, and Motion Control Services. 

8. :ref:`Run the application enabling XScope <running_an_application>`.


**Console commands**

It is able to communicate with the app_demo_torque_control through XTimeComposer console. Each command can be sent by entering a letter in XTimeComposer console. Sometimes, the combination of one letter and one value is needed to have a complete command (for example, writing letter "x" and after that number "100" in console command and then pressing Enter key).
In the following, a brief explanation is brought about different command types and their functionality:

- "a"
    Automatically find the sensor offset and sets the commutation offsets. If the sensor polarity is not set properly, the message "ERROR: wrong polarity for commutation position sensor" will be printed. In this case, the sensor polarity should be changed, and the application should be recompiled.

- "o" and then value "x"
    Sets the commutation offset to value "x". The value "x" should be a number between 0 and 4095 (corresponding to 0 to 360 degrees).

- "t"
    Enables/Disables the torque controller.

- value "x"
    If the torque controller is activated, Writing a number (and then Enter) will send the same value to the torque controller (as the reference value of torque).

- "r"
    This command changes the sign of reference torque.

- "b"
    Enables/Disables the electric brake. 

- "bvn" and then value "x"
    bvn stands for "brake_voltage_nominal". This command sets the nominal (rated) voltage of dc bus to the value "x". The value "x" should be entered in Volts (for example, writing "bvn20" will set the rated value of dc bus voltage to 20 Volts.)

- "bvp" and then value "x"
    bvp stands for "brake_voltage_pull". This command sets the pulling voltage of brake to the value "x". The value "x" should be entered in milli-Volts. (for example, writing "bvp12000" will set the pulling voltage for the brake to 12 Volts.)

- "bvh" and then value "x"
    bvh stands for "brake_voltage_hold". This command sets the holding voltage of brake to the value "x". The value "x" should be entered in milli-Volts. (for example, writing "bvp12000" will set the rated value of dc bus voltage to 12 Volts.)

- "bt" and then value "x"
    bt stands for "brake_pulling_time". This command sets the pulling time of brake to the value "x". The value "x" should be entered in milli-seconds. (for example, writing "bt2000" will lead to 2 seconds of pulling time for electric brake.)

- "s"
    This command activates the "safe_torque_off" mode. In this mode, all inverter switches will be in open state, and motor terminals will be in floating state.

- "x"
    This command activates the xscope functionality for a certain period of time. The default value is 10 seconds. 

- "z"
    In case a fault is detected (such as over current/over voltage), pressing "z" will clean the fault error message. After this command, it is needed to reset the offset, and activate the torque controller.

- "m" and then value "x"
    This command generates a square-waveform signal and uses this signal as the reference value of torque. The frequency and period of torque reference is changing between 0.5-2.5 kHz which will result to play a simple melody by your motor! The amplitude of the square waveform will be equal to the value "x" in milli-Nm which in practice can act as the volume of this melody.



Did everything go well? If you need further support please check out our `forum <http://forum.synapticon.com>`_



