==============================
SOMANET Symmetrical PWM module
==============================

.. contents:: In this document
    :backlinks: none
    :depth: 3

This module offers a service that generates PWM signals over the provided ports. These generated pulses can be
controlled by a client function. Two different versions of the service are available, one triggers a signal with
every generated pulse (required sometimes for proper ADC sampling) and the other one does not. 

These PWM signals are intended to control both high- and low-side switches of three H-brigdes. 
The three channels are center aligned which means that the outputs are symmetrical to the center of the pulses.

This module was originally `created by XMOS and then forked`_ and maintained in a dedicated `repo by Synapticon`_.
Later on, it was moved into this Motor Control Library.

How to use
==========

.. note:: You might find useful the **PWM Symmetrical Demo** example app, which illustrates the use of this module. 


First add the module to your app Makefile

::

 USED_MODULES = module_pwm_symmetrical etc etc

Include the header in your app

::

 #include <pwm_service.h>

Service Initialization
---------------------

Using the Service
---------------------

API
===

Types
-----

.. doxygenenum:: PwmPorts

Server
-----

.. doxygenfunction:: pwm_service
.. doxygenfunction:: pwm_triggered_service


Client
------

.. doxygenfunction:: update_pwm_inv


.. _`created by XMOS and then forked`: https://github.com/xcore/sc_pwm/tree/53f275204764669c9d8ae10378453aa279a5bc47
.. _`repo by Synapticon`: https://github.com/synapticon/sc_pwm/tree/30623702ab9b535e34113f41abb429d55edd26ec
