SOMANET BiSS Encoder Test App
=============================

`test\_biss.xc <https://github.com/synapticon/sc_sncn_motorctrl/blob/feature_biss/test_biss/src/test_biss.xc>`_
illustrates the usage of
`module\_biss <https://github.com/synapticon/sc_sncn_motorctrl/tree/feature_biss/module_biss>`_
to get position information.


-  **THREADS**: BiSS Server, BiSS Client.
-  **TILES**: ``#define TILE_ONE 0     #define IFM_TILE 3`` 

    **Do not forget to set properly your biss encoder configuration
    when using this application**.


**TILE\_ONE**
~~~~~~~~~~~~~

This tile (0 by default) takes care of the client side function. Since these functions do not require any port access, any
free TILE could run them. 

``on tile[TILE_ONE]:`` - **Thread**: BiSS Client ``biss_test(i_biss[0]);`` The client reads position from BiSS Server. Read more at `module\_biss <https://github.com/synapticon/sc_sncn_motorctrl/tree/feature_biss/module_biss>`_.

It is not recommended to run this thread in IFM_TILE together with the Server thread since the xscope output will slow down the GPIO Server thread and affect its performance.

**IFM\_TILE**
~~~~~~~~~~~~~

This tile (3 by default) executes the server side functions, controlling
the ports. These functions need access to the Interface Module
(IFM), just the tile that provides access to the IFM ports can run these
functions.

``on stdcore[IFM_TILE]:`` - **Thread**: BiSS Server
``biss_par biss_params;    run_biss(i_biss, qei_q_ch1, p_ifm_encoder_ch2, clk_biss, 10, 1, biss_params, 2);``
BiSS Server that reads the data from the encoder. Read more at
`module\_biss <https://github.com/synapticon/sc_sncn_motorctrl/tree/feature_biss/module_biss>`_.

More information about BiSS Server/ Client can be found at
`module\_biss <https://github.com/synapticon/sc_sncn_motorctrl/tree/feature_biss/module_biss>`_.

Other dependencies:
`module\_nodeconfig <https://github.com/synapticon/sc_somanet-base/tree/master/module_board-support>`_@`sc\_somanet-base <https://github.com/synapticon/sc_somanet-base>`_
`module\_blocks <https://github.com/synapticon/sc_sncn_motorctrl/tree/master/module_blocks>`_
`module\_common <https://github.com/synapticon/sc_sncn_motorctrl/tree/master/module_common>`_

**See also**:

-  `Getting started with
   SOMANET <http://doc.synapticon.com/index.php/Category:Getting_Started_with_SOMANET>`_