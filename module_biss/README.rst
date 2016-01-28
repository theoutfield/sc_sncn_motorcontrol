SOMANET Motor Control BiSS Encoder Component
============================================

:scope: General Use
:description: BiSS encoder interface
:keywords: SOMANET
:boards: SOMANET IFM-Drive-DC100, SOMANET IFM-Drive-DC1K

Description
-----------

This component provides driver for a BiSS encoder connected to the SOMANET IFM-Drive hardware module. The component provides BiSS server thread which read BiSS sensor data from the encoder and extract position information when called by the client with an interface; and provides an interface for the client to get position from the BiSS server.
