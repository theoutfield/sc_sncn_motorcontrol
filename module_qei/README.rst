Quadrature Encoder Interface Module
===================================

:scope: General Use
:description: SOMANET Motor Control Quadrature Encoder Interface
:keywords: SOMANET
:boards: SOMANET IFM-DC100, SOMANET IFM-DC300


Description
-----------

This module provides driver for the Incremental Encoders connected to
the interface module (IFM). The module provides QEI server thread which
acquires position information from the Incremental encoder in quadrature
mode in a while loop; and provides client functions to configure QEI
Server with encoder resolution, encoder type, polarity and max ticks;
get position from QEI Server and to calculate velocity from the QEI
position.
