SOMANET Motor Control Software
==============================

BLDC/Brushed DC Motor Control Software for SOMANET devices.

-  `How
   to? <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/howto>`_

.. raw:: html

   <table>
   <tr>
     <td width="150px" height="30px">

Latest release:

.. raw:: html

   </td>
     <td width="300px">

1.0

.. raw:: html

   </td>
   </tr>
   <tr>
     <td height="30px">

Maintainer:

.. raw:: html

   </td>
     <td>

support@synapticon.com

.. raw:: html

   </td>
   </tr>
   </table> 

Key Features
------------

-  Commutation (sinusoidal)
-  Profile Position Control
-  Profile Velocity Control
-  Profile Torque Control
-  Homing feature
-  Ethercat Operating Modes
-  Support QEI sensor with Index/ no Index
-  Support Hall sensor
-  Support Analog sensor
-  Support GPIO Digital
-  Precise position control based on position sensor ticks

Content
-------

\| Module \| Demo \| \| :-------------: \|:------------- \| \|
`module\_adc <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_adc>`_
\|
`test\_adc\_external\_input <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_ecat_drive>`_
\| \|
`module\_blocks <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_blocks>`_
\| \| \|
`module\_comm <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_comm>`_
\| \| \|
`module\_common <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_common>`_
\| \| \|
`module\_commutation <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_commutation>`_
\| \| \|
`module\_ctrl\_loops <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_ctrl_loops>`_
\|
`test\_position-ctrl <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/test_position-ctrl>`_
`test\_velocity-ctrl <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/test_velocity-ctrl>`_
`test\_torque-ctrl <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/test_torque-ctrl>`_
\| \|
`module\_ecat\_drive <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_ecat_drive>`_
\|
`test\_ethercat-motorctrl-mode <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/test_ethercat-motorctrl-mode>`_
\| \|
`module\_gpio <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_gpio>`_
\|
`test\_gpio\_digital <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/test_gpio_digital>`_
`test\_homing <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/test_homing>`_
\| \|
`module\_hall <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_hall>`_
\|
`test\_hall <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/test_hall>`_
\| \|
`module\_profile <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_profile>`_
\| \| \|
`module\_qei <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_qei>`_
\|
`test\_qei <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/test_qei>`_
\| \|
`module\_sm <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_sm>`_
\| \|

Required software (dependencies)
--------------------------------

-  `sc\_somanet-base <https://github.com/synapticon/sc_somanet-base>`_
-  `sc\_pwm <https://github.com/synapticon/sc_pwm>`_
-  `sc\_sncn\_ethercat <https://github.com/synapticon/sc_sncn_ethercat>`_
   (only if using Ethercat Operating Modes)

Changelog
---------

-  `1.0 <https://github.com/synapticon/sc_sncn_motorcontrol/releases/tag/v1.0>`_
   (2014-04-17)

   -  Support GPIO Digital ports
   -  Homing feature
   -  Precise Position Control based on position sensor ticks

-  `0.9beta <https://github.com/synapticon/sc_sncn_ctrlproto/releases/tag/v0.9-beta>`_
   (2013-01-24)

License
-------

Please see
`LICENSE <https://github.com/synapticon/sc_sncn_motorcontrol/blob/master/LICENSE.md>`_.
