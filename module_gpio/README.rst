General Purpose I/O Module
==========================

.. figure:: https://s3-eu-west-1.amazonaws.com/synapticon-resources/images/logos/synapticon_fullname_blackoverwhite_280x48.png
   :align: center
   :alt: SYNAPTICON

   SYNAPTICON
This module provides driver for the GPIO digital ports on the interface
module (IFM). The module provides GPIO server thread which configures
the GPIO ports; read/write GPIO digital ports in a while loop; and
provides client functions to configure ports and read/write ports.

Demo: \*
`test\_gpio\_digital.xc <https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/test_gpio_digital/src/test_gpio_digital.xc>`_

For a better review of all the available functions, check the header
files.

-  `gpio\_server.h <https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/module_gpio/include/gpio_server.h>`_
-  `gpio\_client.h <https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/module_gpio/include/gpio_client.h>`_

**Quick API**
~~~~~~~~~~~~~

**gpio\_server.h**
^^^^^^^^^^^^^^^^^^

**Server loop:**

    TILE constrains: IFM\* (need access to IFM ports)

``void gpio_digital_server(port p_ifm_ext_d[], chanend c_gpio_0, chanend c_gpio_1);``
\* Parameters

-  Return

**gpio\_client.h**
^^^^^^^^^^^^^^^^^^

**Read on GPIO:**
``int read_gpio_digital_input(chanend c_gpio, int port_number);`` \*
Parameters

::

    * c_gpio
    * port_number

**Write on GPIO:**
``void write_gpio_digital_output(chanend c_gpio, int port_number, int port_value);``
\* Parameters

-  Return

**See also**:
^^^^^^^^^^^^^

-  `How to include a module in your application <>`_
-  `Getting started with
   SOMANET <http://doc.synapticon.com/wiki/index.php/Category:Getting_Started_with_SOMANET>`_

\*For Core C22, IFM Tile is located on TILE 3. For Core C21, IFM Tile is
on TILE 1.
