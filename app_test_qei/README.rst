SOMANET Encoder Test App
========================

`test\_qei.xc <https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/test_qei/src/test_qei.xc>`_
illustrates the usage of
`module\_qei <https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_qei>`_
to get position and calculate velocity information.

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

-  **THREADS**: QEI Server, QEI Client.
-  **TILES**: ``#define TILE_ONE 0     #define IFM_TILE 3`` > **Do not
   forget to set properly your motor configuration when using this
   application**.

 - `How to configure your motors <https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/howto/HOW_TO_CONFIGURE_MOTORS.md>`_

**TILE\_ONE**
~~~~~~~~~~~~~

This tile (0 by default) takes care of the client side functions and
control loop. Since these functions do not require any port access, any
free TILE could run them. ``on stdcore[TILE_ONE]:`` - **Thread**: QEI
Client ``qei_test(c_qei_p1);`` The client reads position fron QEI Server
and calculates velocity from the position info. Read more at
`module\_qei <https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_qei>`_.

**IFM\_TILE**
~~~~~~~~~~~~~

This tile (3 by default) executes the server side functions, controlling
the interfaces. These functions need access to the Interface Module
(IFM), just the tile that provides access to the IFM ports can run these
functions.

``on stdcore[IFM_TILE]:`` - **Thread**: QEI Server
``qei_par qei_params;     init_qei_param(qei_params);     run_qei(c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4,          c_qei_p5, c_qei_p6, p_ifm_encoder, qei_params); // channel priority 1,2..6``
QEI Server that captures the signals on the sensor. Read more at
`module\_qei <https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_qei>`_.

More information about QEI Server/ Client can be found at
`module\_qei <https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_qei>`_.

Other dependancies:
`module\_nodeconfig <https://github.com/synapticon/sc_somanet-base/tree/master/module_board-support>`_@`sc\_somanet-base <https://github.com/synapticon/sc_somanet-base>`_
`module\_blocks <https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_blocks>`_
`module\_common <https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_common>`_

**See also**:

-  `Getting started with
   SOMANET <http://doc.synapticon.com/index.php/Category:Getting_Started_with_SOMANET>`_

