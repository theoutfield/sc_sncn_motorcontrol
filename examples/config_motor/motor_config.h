// IMPORTANT PARAMETERS (=> lead to mulfunction or damage if set wrong)
#define POLE_PAIRS              2       //number of motor pole-pairs
#define TORQUE_CONSTANT         150000  //Torque constant [micro-Nm/Amp-RMS]
#define RATED_CURRENT           320    //rated phase current [milli-Amp-RMS]
#define MAXIMUM_TORQUE          5000    //maximum value of torque which can be produced by motor [milli-Nm]
#define RATED_TORQUE            180    //rated motor torque [milli-Nm].

// OTHER PARAMETERS (do not change if not having access to the following parameter values)
#define RATED_POWER             230     // rated power [W]
#define PEAK_SPEED              12000   // maximum motor speed [rpm]
#define PHASE_RESISTANCE        490000  // motor phase resistance [micro-ohm]
#define PHASE_INDUCTANCE        580     // motor phase inductance [micro-Henry]

// GENERAL PARAMETERS
#define MOTOR_TYPE              BLDC_MOTOR      //MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]
