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

Definitions
-------------

.. doxygendefine:: PID_DENOMINATOR

Global Types
-------------

.. doxygenstruct:: ControlConfig

Position Control Loop
---------------------

Position Control Service
````````````````````````

.. doxygenfunction:: init_position_control
.. doxygenfunction:: position_control_service
.. doxygenfunction:: position_limit

Position Control Interface
``````````````````````````

.. doxygeninterface:: PositionControlInterface


Velocity Control Loop
---------------------

Velocity Service
````````````````

.. doxygenfunction:: init_velocity_control
.. doxygenfunction:: velocity_control_service
.. doxygenfunction:: max_speed_limit

Velocity Interface
``````````````````

.. doxygeninterface:: VelocityControlInterface

Torque Control Loop
-------------------

Torque Service
````````````````
.. doxygenfunction:: init_torque_control
.. doxygenfunction:: torque_control_service
.. doxygenfunction:: torque_limit

Torque Interface
````````````````
.. doxygeninterface:: TorqueControlInterface
