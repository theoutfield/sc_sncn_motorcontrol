=============================
SOMANET Control Loops Module 
=============================

.. contents:: In this document
    :backlinks: none
    :depth: 3

Lorem ipsum...

How to use
==========

Lorem ipsum...

API
===

Global Types
-------------

.. doxygenstruct:: ControlConfig

Position
--------

Lorem ipsum...

Position Types
````````````````

.. doxygenstruct:: CyclicSyncPositionConfig

Position Service
``````````````````

.. doxygenfunction:: init_position_control
.. doxygenfunction:: position_limit
.. doxygenfunction:: set_position_csp
.. doxygenfunction:: position_control_service

Position Interface
```````````````````

.. doxygeninterface:: PositionControlInterface


Velocity
--------

Lorem ipsum...

Velocity Types
``````````````

.. doxygenstruct:: CyclicSyncVelocityConfig

Velocity Service
``````````````````
.. doxygenfunction:: init_velocity_control
.. doxygenfunction:: max_speed_limit
.. doxygenfunction:: set_velocity_csv
.. doxygenfunction:: velocity_control_service

Velocity Interface
````````````````````

.. doxygeninterface:: VelocityControlInterface

Torque
------

Lorem ipsum...

Torque Types
``````````````

.. doxygenstruct:: CyclicSyncTorqueConfig

Torque Service
````````````````
.. doxygenfunction:: init_torque_control
.. doxygenfunction:: torque_limit
.. doxygenfunction:: set_torque_cst
.. doxygenfunction:: torque_control_service
.. doxygenfunction:: enable_adc

Torque Interface
````````````````
.. doxygeninterface:: TorqueControlInterface