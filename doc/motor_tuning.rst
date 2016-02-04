.. _motor_tuning_label:

BLDC Hall Effect Feedback sensor offset adjustment 
==================================================

It is common that Hall Effect sensors used for position feedback are not perfectly placed physically along the windings. This strongly affects the commutation efficiency of your motor.

In order to avoid such loss of efficiency we can configure at the Motor Control Service the offsets to compensate the Hall sensors error on clock-wise and counter-clock-wise spin.
We provide :ref:`an app <offset_commutation_tuning_demo>` to find out what are the offset values that optimize the commutation of your motor.

