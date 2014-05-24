Quadrature Encoder Interface Module
===================================

.. figure:: https://s3-eu-west-1.amazonaws.com/synapticon-resources/images/logos/synapticon_fullname_blackoverwhite_280x48.png
   :align: center
   :alt: SYNAPTICON

   SYNAPTICON
This module provides driver for the Incremental Encoders connected to
the interface module (IFM). The module provides QEI server thread which
acquires position information from the Incremental encoder in quadrature
mode in a while loop; and provides client functions to configure QEI
Server with encoder resolution, encoder type, polarity and max ticks;
get position from QEI Server and to calculate velocity from the QEI
position.

Demos: -
`test\_qei.xc <https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/test_qei/src/test_qei.xc>`_

**Quick API**
~~~~~~~~~~~~~

For a better review of all the available functions, check the header
files.

-  `qei\_server.h <https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/module_qei/include/qei_server.h>`_
   - To access the server side functions.
-  `qei\_client.h <https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/module_qei/include/qei_client.h>`_
   - To access the client side functions.

**qei\_server.h**
^^^^^^^^^^^^^^^^^

-  **Server Initialization function:**
   ``Server limitations:   Maximum 8800 rpm on 1000 count encoder   Maximum 2200 rpm on 4000 count encoder   Maximum 425 rpm on 20000 count encoder``
   > TILE constrains: IFM\* (need access to IFM ports)

``void run_qei(chanend c_qei_p1, chanend c_qei_p2, chanend c_qei_p3, chanend c_qei_p4, chanend c_qei_p5, \         chanend c_qei_p6, port in p_qei, qei_par &qei_params);``
\* Parameters

**qei\_client.h**
^^^^^^^^^^^^^^^^^

-  **Obtain Position**:
   ``{unsigned int, unsigned int} get_qei_position(chanend c_qei, qei_par &qei_params);``
-  Parameters

-  Return

-  **Obtain Velocity**:
   ``int get_qei_velocity(chanend c_qei, qei_par &qei_params, qei_velocity_par &qei_velocity_params);``
-  Parameters

-  Return

**See also**:
^^^^^^^^^^^^^

-  `How to include a module in your application <>`_
-  `Getting started with
   SOMANET <http://doc.synapticon.com/wiki/index.php/Category:Getting_Started_with_SOMANET>`_

\*For Core C22, IFM Tile is located on TILE 3. For Core C21, IFM Tile is
on TILE 1.
