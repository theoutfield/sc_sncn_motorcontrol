.. _motor_configuration_label:

How to configure parameters of SOMANET Motion Control Applications
==================================================================

.. important:: It is very important to set your Services configuration according to your motor specifications, otherwise you might experience misbehavior when driving your motors.

All services that compose the SOMANET Motion Control Component receive their very own and independent configuration. These configurations are provided as
arguments of the Services and therefore they must be set before the Service instantiation.

.. note:: Check the API of each Service you use for further information of its specific configuration.

However, for simplicity and in order to use a common configuration for all your apps. We encourage you to gather your configuration in a common header
for all your apps. We do exactly this for most of our example apps, in a way that each app fetches its configuration from `configuration_parameters/user_config.h <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/examples/configuration_parameters>`. 

.. note:: In order to include such a configuration header in your app, you must tell the Makefile to use the module where the header is contained. e.g.

       .. code-block:: c
       
                USED_MODULES = configuration_parameters etc etc

For most apps, it is not strictly necessary to fill up all the configuration parameters. Normally they just need a few, but the header is thought to be used generally for any app. You can easily check in your app **main.xc** which parameters are used for the configuration of your services.

.. note:: Of course, you could avoid using any configuration header and simply hardcode your configuration within your **main.xc**. But it is not the a recommendable practice for managing several apps in a workspace.
