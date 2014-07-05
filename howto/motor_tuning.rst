.. _motor_tuning_label:

BLDC Motor Tuning Guide 
=======================

After you have set up most of your motor parameters according to :ref:`motor_configuration_label` you can further fine tune your motors efficiency and performance. Inside the bldc_motor_config file you will find following lines::

    #define COMMUTATION_OFFSET_CLK      683
    #define COMMUTATION_OFFSET_CCLK     2731

Steps
+++++

   #. Run your motor in velocity control mode at half of it's maximum nominal speed (MAX_NOMINAL_SPEED / 2) in the clock-wise direction
   #. Monitor the current consumption on you power supply
   #. Modify the parameter COMMUTATION_OFFSET_CLK by gradually increasing its original vale (683 + X). You should not add more than 342 to it (X <= 342)
   #. Repeat the previous step as long as the current drained is decreasing, and thereby find the optimum COMMUTATION_OFFSET_CLK angle.
   #. Set the COMMUTATION_OFFSET_CCLK to its original value 2731 minus the identified value X (2731 - X)
