SOMANET Motor Control Hall Sensor Module
========================================

:scope: General Use
:description: Hall sensor interface
:keywords: SOMANET
:boards: SOMANET IFM-DC100, SOMANET IFM-DC300

.. figure:: https://s3-eu-west-1.amazonaws.com/synapticon-resources/images/logos/synapticon_fullname_blackoverwhite_280x48.png
   :align: center
   :alt: SYNAPTICON

Description
-----------

This module provides driver for the Hall sensor connected to the
interface module (IFM). The module provides Hall server thread which
drives the Hall sensor, acquires position information and calculates
velocity in a while loop; and provides client functions to configure
Hall Server with number of pole pairs and max ticks; get position and
velocity from the Hall server.

Demos: -
`test\_hall.xc <https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/test_hall/src/test_hall.xc>`_

**Quick API**
~~~~~~~~~~~~~

**hall\_server.h**
^^^^^^^^^^^^^^^^^^

-  **Server loop:**

    TILE constrains: IFM\* (need access to IFM ports)

``void run_hall(chanend c_hall_p1, chanend c_hall_p2, chanend c_hall_p3, chanend c_hall_p4,      chanend c_hall_p5, chanend c_hall_p6, port in p_hall, hall_par &hall_params);``
\* Parameters

-  Return

**hall\_client.h**
^^^^^^^^^^^^^^^^^^

-  **Get position value:** \`\`\` int get\_hall\_position(chanend
   c\_hall);

\`\`\` \* Parameters

-  Return

-  **Get velocity value:**
   ``int get_hall_velocity(chanend c_hall, hall_par &hall_params);``
-  Parameters

-  Return

For a better review of all the available functions, check the header
files.

-  `hall\_server.h <https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/module_hall/include/hall_server.h>`_
-  `hall\_client.h <https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/module_hall/include/hall_client.h>`_

**See also**:
^^^^^^^^^^^^^

-  `How to include a module in your application <>`_
-  `Getting started with
   SOMANET <http://doc.synapticon.com/wiki/index.php/Category:Getting_Started_with_SOMANET>`_

\*For Core C22, IFM Tile is located on TILE 3. For Core C21, IFM Tile is
on TILE 1.
