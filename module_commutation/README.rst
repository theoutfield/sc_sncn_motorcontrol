Commutation Module
==================

:scope: General Use
:description: SOMANET Motor Control Commutation module
:keywords: SOMANET
:boards: SOMANET IFM-DC100, SOMANET IFM-DC300

.. figure:: https://s3-eu-west-1.amazonaws.com/synapticon-resources/images/logos/synapticon_fullname_blackoverwhite_280x48.png
   :align: center
   :alt: SYNAPTICON

Description
-----------

This module provides driver for the BLDC Motor connected to the
interface module (IFM). The module consists of commutation which
internally makes use of the predriver to drive fets and configurations
under pwm. The module provides Commutation server thread which acquires
position information from the Hall server and commutates the motor in a
while loop; and provides client functions to optimize motor commutation
with commutation offsets, motor winding types, nominal motor speed and
number of pole pairs; set input voltage for the motor, get fet\_state
from the Commutation Server.

Demos: -
`test\_position-ctrl.xc <https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/test_position-ctrl/src/test_position-ctrl.xc>`_
-
`test\_torque-ctrl.xc <https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/test_torque-ctrl/src/test_torque-ctrl.xc>`_
-
`test\_velocity-ctrl.xc <https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/test_velocity-ctrl/src>`_

**Quick API**
~~~~~~~~~~~~~

**comm\_loop\_server.h**
^^^^^^^^^^^^^^^^^^^^^^^^

**Server loop:**

    TILE constrains: IFM\* (need access to IFM ports)

``void commutation_sinusoidal(chanend c_hall, chanend c_qei, chanend c_signal, chanend c_watchdog,      chanend c_commutation_p1, chanend c_commutation_p2, chanend c_commutation_p3, chanend c_pwm_ctrl,     out port p_ifm_esf_rstn_pwml_pwmh, port p_ifm_coastn, port p_ifm_ff1, port p_ifm_ff2,     hall_par &hall_params, qei_par &qei_params, commutation_par &commutation_params);``
\* Parameters \* c\_hall \* c\_qei \* c\_singal \* c\_watchdog \*
c\_commutation\_p1 \* c\_commutation\_p2 \* c\_commutation\_p3

**comm\_loop\_client.h**
^^^^^^^^^^^^^^^^^^^^^^^^

**Parameters initialization:**
``void init_commutation_param(commutation_par &commutation_params, hall_par &hall_params, int nominal_speed);``
\* Parameters \* commutation\_params \* hall\_params

**Commutation loop initialization:**
``int init_commutation(chanend c_signal);`` \* Parameters

-  Return

For a better review of all the available functions, check the header
files.

-  `comm\_loop\_server.h <https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/module_commutation/include/comm_loop_server.h>`_
-  `comm\_loop\_client.h <https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/module_commutation/include/comm_loop_client.h>`_

**See also**:
^^^^^^^^^^^^^

-  `How to include a module in your application <>`_
-  `Getting started with
   SOMANET <http://doc.synapticon.com/wiki/index.php/Category:Getting_Started_with_SOMANET>`_

\*For Core C22, IFM Tile is located on TILE 3. For Core C21, IFM Tile is
on TILE 1.
