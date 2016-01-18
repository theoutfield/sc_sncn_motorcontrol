.. _pwm_symmetrical_module:
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

When running the PWM Service, the **Reference Frequency** of the tile where the Service is allocated will be
automatically changed to **250MHz**.

The PWM Service should always run over an **IFM tile** so it can access the ports to your SOMANET IFM device.

This module was originally created by XMOS and then reworked by Synapticon before being include into the SOMANET Motor Control Library.

* `Original forked XMOS repository`_
* `Forked repository by Synapticon`_

.. cssclass:: github

  `See Module on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_pwm_symmetrical>`_

How to use
==========

.. important:: We assume that you are using :ref:`SOMANET Base <somanet_base>` and your app includes the required **board support** files for your SOMANET device.

.. seealso:: You might find useful the :ref:`PWM Symmetrical Demo <pwm_symmetrical_demo>` example app, which illustrates the use of this module. 

1. First, add all the :ref:`SOMANET Motor Control <somanet_motor_control>` modules to your app Makefile.

    ::

        USED_MODULES = module_pwm_symmetrical module_adc module_ctrl_loops module_hall module_misc module_motorcontrol module_profile module_qei module_watchdog module_board-support


    .. note:: Not all modules will be required, but when using a library it is recommended to include always all the contained modules. 
          This will help solving internal dependancy issues.

2. Include the PWM Service headers in your app, for Service **pwm_service.h** and Client **pwm_service_client.h**.
3. Instanciate the ports where the Service will be outputting the PWM signals. 
4. Inside your main function, declare the channels for Service-Client communication.
5. At your IFM tile, instanciate the Service.
6. At whichever other core, you can update your PWM outputs through a client call. 
    But first you will need to initialize the communication by calling ``pwm_share_control_buffer_address_with_server``.

    .. code-block:: C

        #include <CORE_C22-rev-a.bsp>   //Board Support file for SOMANET Core C22 device 
        #include <IFM_DC100-rev-b.bsp>  //Board Support file for SOMANET IFM DC100 device 
                                        //(select your board support files according to your device)

        #include <pwm_service.h> // 2
        #include <pwm_service_client.h>

        PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS; // 3
       
        int main (void)
        {
            chan c_pwm_ctrl; // 4         
      
            par {

                on tile[IFM_TILE]:
                {
                    static t_pwm_control pwm_ctrl; // 6 
                    unsigned int pwm_values[3] = { 1000, 2000, 4000 };

                    pwm_share_control_buffer_address_with_server(c_pwm_ctrl, pwm_ctrl);
                    update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm_values);
                }

                on tile[IFM_TILE]: pwm_service(pwm_ports, c_pwm_ctrl); // 5
            }

            return 0;
        }

.. seealso:: If you are interested in the use of the **Triggered PWM Service**, have a look at the :ref:`BLDC Torque Control Demo App<bldc_torque_control_demo>`.

API
===

Types
-----

.. doxygenstruct:: PwmPorts

Server
------

.. doxygenfunction:: pwm_service
.. doxygenfunction:: pwm_triggered_service


Client
------

.. doxygenfunction:: pwm_share_control_buffer_address_with_server
.. doxygenfunction:: update_pwm_inv


.. _`Original forked XMOS repository`: https://github.com/xcore/sc_pwm/tree/53f275204764669c9d8ae10378453aa279a5bc47
.. _`Forked repository by Synapticon`: https://github.com/synapticon/sc_pwm/tree/30623702ab9b535e34113f41abb429d55edd26ec
