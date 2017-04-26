
.. _module_shared_memory:

====================
Shared Memory Module 
====================

.. contents:: In this document
    :backlinks: none
    :depth: 3

This module provides...

.. cssclass:: github

  `See Module on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_shared_memory>`_


How to use
==========

.. important:: We assume that you are using :ref:`SOMANET Base <somanet_base>` and your app includes the required **board support** files for your SOMANET device.
          
1. First, add all the **SOMANET Motor Control Library** modules to your app Makefile.

    ::

	USED_MODULES = …


    .. note:: Not all modules will be required, but when building an application it is recommended to include always all the listed modules. 
          It will help to solve internal dependency issues.

2. Include ...

API
===

