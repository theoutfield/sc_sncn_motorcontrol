.. _motor_configuration_label:

How to configure your motors
============================

Any application that uses this software component must contain a motor
configuration file. This file, called **bldc\_motor\_config.h**, must be
located at the folder **config/motor/** on your application. Since It
defines the features of your BLDC motor, it is important to set it
properly before running any motor control application.

You can find many examples of this file at the different application
examples provided.

The main parameters to define in our configuration file will be:

-  Number of pole pairs:

   ::

       #define POLE_PAIRS n // e.g. 8

-  Maximum nominal speed (in rpms):

   ::

       #define MAX_NOMINAL_SPEED n // e.g. 4000

-  Maximum nominal current (A):

   ::

       #define MAX_NOMINAL_CURRENT n // e.g. 2

-  Motor torque constant (required for torque control):

   ::

       #define MOTOR_TORQUE_CONSTANT 34 // mNm/A

-  Your motor winding type (star or delta):

   ::

       #define WINDING_TYPE DELTA_WINDING // or STAR_WINDING

-  Your DC board (DC100 or DC300):

   ::

       #define IFM_RESOLUTION DC100_RESOLUTION // or DC300_RESOLUTION

-  Sensor used for control (Hall or Quadrature Encoder):

   ::

       #define SENSOR_USED HALL // or QEI

-  If QEI sensor is used then set the type (w/ or w/o index),
   resolution, or polarity (to match the hall sensor polarity):

::

    #define QEI_SENSOR_POLARITY NORMAL // or NORMAL
    #define ENCODER_RESOLUTION 4000 // 4 x Max count of Quadrature Encoder
    #define QEI_SENSOR_POLARITY NORMAL // NORMAL

-  If you are using a gear, specify the ratio, otherwise set to 1:

   ::

       #define GEAR_RATIO n // e.g. 26, or 1 if no gear is attached


Tuning the commutation offset angles
+++++++++++++++++++++++++++++++++++++

Please refer to the :ref:`motor_tuning_label`
