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

.. important:: We assume that you are using SOMANET Base and your app includes the required board support for your hardware.
          You might find useful the **PWM Symmetrical Demo** example app, which illustrates the use of this module. 

Service Initialization
----------------------
First add the module to your app Makefile

::

 USED_MODULES = module_pwm_symmetrical etc etc

Include the Service header in your app

::
 #include <pwm_service.h>

Declare the channels for Service-Client communication.

::
 chan c_pwm_ctrl;

Configure the ports and clocks for your service. This configuration is defined within the **board support** files.

::
 PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;

Add a new parallel core in your main app where the PWM Service will run.

::
 on tile[IFM_TILE]: pwm_service(pwm_ports, c_pwm_ctrl);

Using the Service
-----------------

Include the Service Client header in your app

::
 #include <pwm_service_client.h>

Instanciate the shared control structure, array for the PWM target values. 
Initialize the communication calling **pwm_share_control_buffer_address_with_server**.
Then you can start updating your PWM outputs through client calls. 

::

  on tile[IFM_TILE]: 
  {
        t_pwm_control pwm_ctrl;
        unsigned int pwm[3] = {0, 0, 0};  

        pwm_share_control_buffer_address_with_server(c_pwm_ctrl, pwm_ctrl);
        update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);
  }



.. note:: If you are interested in the use of the Triggered PWM Service, have a look at the **Torque Control Demo App**.

API
===

Types
-----

.. doxygenstruct:: PwmPorts

Server
-----

.. doxygenfunction:: pwm_service
.. doxygenfunction:: pwm_triggered_service


Client
------

.. doxygenfunction:: pwm_share_control_buffer_address_with_server
.. doxygenfunction:: update_pwm_inv


.. _`created by XMOS and then forked`: https://github.com/xcore/sc_pwm/tree/53f275204764669c9d8ae10378453aa279a5bc47
.. _`repo by Synapticon`: https://github.com/synapticon/sc_pwm/tree/30623702ab9b535e34113f41abb429d55edd26ec
