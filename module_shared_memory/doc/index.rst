
.. _module_shared_memory:

====================
Shared Memory Module 
====================

.. contents:: In this document
    :backlinks: none
    :depth: 3

This module provides a Shared Memory Service which allow other service to exchange data asynchronously without blocking. For example the Positin feedback service write the position to the Shared memory and the Motorcontrol service can read it later without being blocked by the Positin feedback service. The Shared Memory Service is also a distributable task so it doesn't need a core and it is actually run by the calling task (if all calling task are runnig on the same tile).


How to use
==========

.. important:: We assume that you are using :ref:`SOMANET Base <somanet_base>` and your app includes the required **board support** files for your SOMANET device.
          
#. First, add all the **SOMANET Motor Control Library** modules to your app Makefile.

    ::

	USED_MODULES = lib_bldc_torque_control module_board-support module_shared_memory



    .. note:: Not all modules will be required, but when building an application it is recommended to include always all the listed modules. It will help to solve internal dependency issues.

#. Include **shared_memory.h** header file.

#. Instantiate the interfaces for the shared memory, you need to specify the maximum number of clients.

#. Instantiate the service, you need to specify the maximum number of clients. You also need to put the ``[[distribute]]`` instruction.

#. Then you can use client interface call to write or read data to the shared memory.

    .. code-block:: c

        #include <CORE_C22-rev-a.bsp>   //Board Support file for SOMANET Core C22 device
        #include <IFM_DC100-rev-b.bsp>  //Board Support file for SOMANET IFM DC100 device
                                        //(select your board support files according to your device)

        // 2. Include the header **shared_memory.h** in your app.
        #include <shared_memory.h>
        
        // 3.Instantiate the interfaces for the shared memory
        interface shared_memory_interface i_shared_memory[2];

        int main(void)
        {
            par
            {
                on tile[IFM_TILE]: par
                {
                    par {
                    {
                        // 4. Instantiate the service, you need to specify the maximum number of clients.
                        [[distribute]] shared_memory_service(i_shared_memory, 2);
                    }

                    {
                        //write data
                        i_shared_memory[0].write_angle(1000, 0, 0, 0, 0);
                    }
                    
                    {
                        //read data
                        UpstreamControlData upstream_control_data;
                        upstream_control_data = i_shared_memory>[1].read()
                    }
                }
            }

            return 0;
        }



API
===


Service
--------

.. doxygenfunction:: shared_memory_service

Interface
---------

.. doxygeninterface:: shared_memory_interface

