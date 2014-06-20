Component Overview
==================

Like shown in the core diagram below, 4 timing critical task are executed on the SOMANET IFM Tile:

* **Commutation Server** - Executes sinusoidal commutation software
* **PWM Server** - Generates PWM signals
* **Hall Server** - Delivers rotor angle information required for sinusoidal commutation
* **Watchdog Server** - Provides WDT (Watchdog timer) functionality

As shown, custom controllers should run in a different tile to avoid influencing the performance of motor control. Please refer to the :ref:`Programming Guide <commutation_programming_label>`


.. figure:: images/core-diagram-commutation.*
   :width: 100%

   Custom control application core diagram