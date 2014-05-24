Digital GPIO Demo
=================

.. figure:: https://s3-eu-west-1.amazonaws.com/synapticon-resources/images/logos/synapticon_fullname_blackoverwhite_280x48.png
   :align: center
   :alt: SYNAPTICON

   SYNAPTICON
`test\_gpio\_digital.xc <https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/test_gpio_digital/src/test_gpio_digital.xc>`_
illustrates the usage of
`module\_gpio <https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_gpio>`_
to configure GPIO's and read/write the digital ports.

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
    <td rowspan="2" align="center"> C21-DX <br/> C22 </td>
    <td rowspan="2" align="center"> Drive DC 100 <br/> Drive DC 300</td>

.. raw:: html

   </tr>
   </table>

-  **THREADS**: GPIO Server, GPIO Client

-  **TILES**: ``#define TILE_ONE 0     #define IFM_TILE 3`` > **Do not
   forget to set properly your motor configuration when using this
   application**.

 - `How to configure your
motors <https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/howto/HOW_TO_CONFIGURE_MOTORS.md>`_

**TILE\_ONE** This tile (0 by default) takes care of the client side
functions andcontrol loop. Since these functios do not require any port
access, any free TILE could run them. ``on stdcore[TILE_ONE]:`` -
**Thread**: GPIO Client ``gpio_test(c_gpio_p1);`` The Client function
configures port 0 and port 1 as input switches and remaining 2 ports as
outputs and shows how to read/write the digital ports. See more at
`module\_gpio <https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_gpio>`_.

It is not recommended to run this thread in IFM\_TILE together with the
Server thread since the terminal output will slow down the GPIO Server
thread and affect its performance.

**IFM\_TILE** This tile, 3 by default, executes the server side
functions, controlling the interfaces. These functions need access to
the Interface Module (IFM), just the tile that provides access to the
IFM ports can run these functions.
``on stdcore[IFM_TILE]:`` - **Thread**: GPIO Server
``gpio_digital_server(p_ifm_ext_d, c_gpio_p1, c_gpio_p2);`` It access
the GPIO ports at the IFM module. See more at
`module\_gpio <https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_gpio>`_.

Other dependencies:
`module\_common <https://github.com/synapticon/sc_sncn_motorctrl_sin/tree/master/module_common>`_
module\_nodeconfig@sc\_somanet-base

**See also**:

-  `Getting started with
   SOMANET <http://doc.synapticon.com/wiki/index.php/Category:Getting_Started_with_SOMANET>`_

