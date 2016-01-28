============================
SOMANET Miscellaneous Module 
============================

.. contents:: In this document
    :backlinks: none
    :depth: 3

This module contains certain utilities and constants used internally
by the library. 

From all its functionality, the user might find useful the following resources:

- Implementation of a Moving Average Filter.
- Time definitions for proper time management.

.. cssclass:: github

  `See Module on Public Repository <https://github.com/synapticon/sc_sncn_motorcontrol/tree/master/module_misc>`_

How to use
==========

1. First, add all the :ref:`SOMANET Motor Control <somanet_motor_control>` modules to your app Makefile.

  ::

    USED_MODULES = module_misc module_hall module_pwm_symmetrical module_adc module_ctrl_loops module_motorcontrol module_profile module_gpio module_qei module_watchdog module_board-support

  .. note:: Not all modules will be required, but when using a library it is recommended to include always all the contained modules. 
          This will help solving internal dependancy issues.

Moving Average Filter
`````````````````````

2. Include the filter header in your app. 

.. code-block:: C
        
  #include <filter_blocks.h>

3. Now you can just use filter API.

.. code-block:: C

  int filter_buffer[8] = {0};   
  int index = 0;

  init_filter(filter_buffer, index, 8);  
  filter(filter_buffer, index, 8, NEW_SAMPLE_OF_THE_SIGNAL_TO_FILTER);

Time Definitions
````````````````

2. Include the header in your app.

.. code-block:: C
        
  #include <refclk.h>

3. If you need to set 1 ms time in your code, and your code is running on tile with a **100MHz Reference Freq.**:

.. code-block:: C
        
  timer t;
  unsigned ts;

  t :> ts; 
  t when timerafter(ts + MSEC_STD) :> ts;

4. However, if you need to set 1 ms time in your code, but your code is running on tile with a **250MHz Reference Freq.** (e.g. IFM tile):

.. code-block:: C
        
  timer t;
  unsigned ts;

  t :> ts; 
  t when timerafter(ts + MSEC_FAST) :> ts;

API
===

Time definitions
````````````````
.. doxygendefine:: USEC_STD
.. doxygendefine:: MSEC_STD
.. doxygendefine:: SEC_STD
.. doxygendefine:: USEC_FAST
.. doxygendefine:: MSEC_FAST
.. doxygendefine:: SEC_FAST

Moving Average Filter
`````````````````````

.. doxygenfunction:: init_filter
.. doxygenfunction:: filter
