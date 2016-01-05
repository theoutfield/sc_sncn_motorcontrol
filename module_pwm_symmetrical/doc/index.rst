SOMANET Symmetrical PWM module
==============================

.. contents:: In this document
    :backlinks: none
    :depth: 3

This is an implementation of a complementary PWM to control both high- and low-side switches of three H-brigdes. The three channels are
center aligned which means that the outputs are symmetrical to the center of the pulses.

API
---

Client
``````

.. doxygenfunction:: update_pwm_inv
.. doxygenfunction:: pwm_share_control_buffer_address_with_server

Server
``````

.. doxygenfunction:: pwm_triggered_service
.. doxygenfunction:: pwm_service


