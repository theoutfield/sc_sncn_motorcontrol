===========================
SOMANET Hall Sensor Module
===========================

.. contents:: In this document
    :backlinks: none
    :depth: 3

This module provides a Service that will read and process the data coming from your 
Feedback Hall Sensor. A client that needs this data could retrieve it from the service
through a common interface.

.. image:: images/core-diagram-hall-interface.png
   :width: 50%

How to use
==========

.. important:: We assume that you are using **SOMANET Base** and your app includes the required **board support** files for your SOMANET device.
          You might find useful the **Hall Sensor Test** example app, which illustrates the use of this module. 

Service Initialization
----------------------
First add all the **SOMANET Motor Control Library** modules to your app Makefile.

::

 USED_MODULES = module_hall etc etc

.. note:: Not all modules will be required, but when using a library it is recommended to include always all the contained modules. 
          This will help solving internal dependancy issues.

Include the Service header in your app

.. code-block:: C

 #include <hall_service.h>

API
===

Definitions
-----

.. doxygendefine:: HALL_SENSOR

Types
-----

.. doxygenstruct:: HallConfig
.. doxygenstruct:: HallPorts

Service
-------

.. doxygenfunction:: hall_service

Interface
---------

.. doxygeninterface:: HallInterface
