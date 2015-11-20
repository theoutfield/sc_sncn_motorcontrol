SOMANET Hall Sensor Test App
=============================

`test\_hall.xc <https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/test_hall/src/test_hall.xc>`_
illustrates usage of
`module\_hall <https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_hall>`_
to get position and velocity information.

.. raw:: html

   <table align="center" cellpadding="5" width="80%">
   <tr>
       <th colspan="2">

CORE use

.. raw:: html

   </th>
       <td rowspan="3" width="1px"></td>
       <th colspan="3">

HW compatibility

.. raw:: html

   </th>
   </tr>
   <tr>
       <td>

Parallel THREADS

.. raw:: html

   </td>
       <td width="30px" align="center"> 

2

.. raw:: html

   </td>

::

    <th align="center">COM</th>
    <th align="center">CORE</th>
    <th align="center">IFM</th>

.. raw:: html

   </tr>
   <tr>
       <td>

TILES used

.. raw:: html

   </td>
       <td width="30px" align="center"> 

2

.. raw:: html

   </td>

::

    <td rowspan="2" align="center">*</td>
    <td rowspan="2" align="center">C21-DX <br/> C22 </td>
    <td rowspan="2" align="center">Drive-DC100 <br/> Drive-DC300</td>

.. raw:: html

   </tr>
   </table>

-  **THREADS**: Hall Sensor Server, Hall Sensor Client
-  **TILES**: ``#define TILE_ONE 0     #define IFM_TILE 3``

    **Do not forget to set properly your node and motor configuration
    when using this application**.

-  `How to configure your
   motors <https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/howto/HOW_TO_CONFIGURE_MOTORS.md>`_

**TILE\_ONE**
~~~~~~~~~~~~~

This tile (0 by default) takes care of the client side function. Since
these functions do not require any port access, any free TILE could run
them. ``on stdcore[TILE_ONE]:`` - **Thread**: Hall Sensor Client
``hall_test(c_hall_p1);`` The client reads position and velocity from
HALL Server. See more at
`module\_hall <https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_hall>`_.

It is not recommended to run this thread in IFM\_TILE together with the
Server thread since the terminal output will slow down the GPIO Server
thread and affect its performance.

**IFM\_TILE** This tile (3 by default) executes the server side
functions, controlling the interfaces. These functions need access to
the Interface Module (IFM), just the tile that provides access to the
IFM ports can run these functions.

``on stdcore[IFM_TILE]:`` - **Thread**: Hall Sensor Server
``HallConfig hall_config;     init_hall_config(hall_config);     run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5,         c_hall_p6, p_ifm_hall, hall_config); // channel priority 1,2..6``
The Hall Server captures signal values from the sensors. See more at
`module\_hall <https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_hall>`_.

More information about Hall Server/ Client can be found at
`module\_hall <https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_hall>`_
documentation.

Other dependencies:
`module\_blocks <https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_blocks>`_
`module\_common <https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_common>`_
`module\_nodeconfig <https://github.com/synapticon/sc_somanet-base/tree/master/module_board-support>`_@`sc\_somanet-base <https://github.com/synapticon/sc_somanet-base>`_

**See also**:

-  `Getting started with
   SOMANET <http://doc.synapticon.com/index.php/Category:Getting_Started_with_SOMANET>`_

