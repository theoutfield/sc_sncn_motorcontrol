Blocks Module
=============

.. figure:: https://s3-eu-west-1.amazonaws.com/synapticon-resources/images/logos/synapticon_fullname_blackoverwhite_280x48.png
   :align: center
   :alt: SYNAPTICON

   SYNAPTICON
This module provides watchdog server, filter implementation and drive
utilities.

The watchdog server disables the motor phases in case of emergency. A
moving average filter implementation is provided under filter\_blockd
for sensor data filtering.

TILE constrains: IFM\* (need access to IFM ports)

Demos: -
`test\_position-ctrl.xc <https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/test_position-ctrl/src/test_position-ctrl.xc>`_
-
`test\_torque-ctrl.xc <https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/test_torque-ctrl/src/test_torque-ctrl.xc>`_
-
`test\_velocity-ctrl.xc <https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/test_velocity-ctrl/src>`_

**Quick API**
~~~~~~~~~~~~~

**watchdog.h**
^^^^^^^^^^^^^^

**Server loop:** Run the watchdog timer server

    TILE constrains: IFM\* (need access to IFM ports)

``void run_watchdog(chanend c_watchdog, out port p_wd_tick, out port p_shared_leds_wden);``
#### **filter\_blocks.h**#### **Filter Initialization function:**
Initialise Moving Average Filter Parameters
``void init_filter(int filter_buffer[], int &index, int filter_length);``
\* Parameters

-  Return

**Filter function:**
``int filter(int filter_buffer[], int &index, int filter_length, int input);``
\* Parameters

-  Return

For a better review of all the available functions, check the header
files.

-  `watchdog.h <https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/module_blocks/include/watchdog.h>`_
-  `filter\_blocks.h <https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/module_blocks/include/filter_blocks.h>`_

**See also**:
^^^^^^^^^^^^^

-  `How to include a module in your application <>`_
-  `Getting started with
   SOMANET <http://doc.synapticon.com/wiki/index.php/Category:Getting_Started_with_SOMANET>`_

\*For Core C22, IFM Tile is located on TILE 3. For Core C21, IFM Tile is
on TILE 1.
