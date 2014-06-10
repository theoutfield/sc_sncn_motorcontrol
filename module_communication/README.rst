SOMANET Motor Control Communication Module
============================

:scope: General Use
:description: SOMANET Motor Control communication module
:keywords: SOMANET
:boards: SOMANET IFM-DC100, SOMANET IFM-DC300

.. figure:: https://s3-eu-west-1.amazonaws.com/synapticon-resources/images/logos/synapticon_fullname_blackoverwhite_280x48.png
   :align: center
   :alt: SYNAPTICON

Description
-----------

This module provides functions to read/write data to ctrlproto data
structure which in turn is passed to EtherCAT Master applications;
functions to read SDO (software defined objects used for motor/sensor
configuration); functions to update respective Servers running on
interface module (IFM\_CORE).

**Quick API**
~~~~~~~~~~~~~

**comm.h**
^^^^^^^^^^

**Target velocity from EtherCAT:**

    TILE constrains: IFM\* (need access to IFM ports)

\`\`\` int get\_target\_velocity(ctrl\_proto\_values\_t InOut);

\`\`\` \* Parameters

-  Return

**Target position from EtherCAT:** \`\`\` int
get\_target\_position(ctrl\_proto\_values\_t InOut);

\`\`\` \* Parameters

-  Return

**Target torque from EtherCAT:** \`\`\` int
get\_target\_position(ctrl\_proto\_values\_t InOut);

\`\`\` \* Parameters

-  Return

**Send actual torque value:**
``void send_actual_torque(int actual_torque, ctrl_proto_values_t &InOut);``
\* Parameters

-  Return

**Send actual velocity value:** \`\`\` void send\_actual\_velocity(int
actual\_velocity, ctrl\_proto\_values\_t &InOut);

\`\`\` \* Parameters

-  Return

**Send actual position value:** \`\`\` void send\_actual\_position(int
actual\_position, ctrl\_proto\_values\_t &InOut);

\`\`\` \* Parameters

-  Return

For a better review of all the available functions, check the header
file.

-  `comm.h <https://github.com/synapticon/sc_sncn_motorctrl_sin/blob/master/module_comm/include/comm.h>`_

**See also**:
^^^^^^^^^^^^^

-  `How to include a module in your application <>`_
-  `Getting started with
   SOMANET <http://doc.synapticon.com/wiki/index.php/Category:Getting_Started_with_SOMANET>`_

