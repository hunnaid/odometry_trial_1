#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor LFMotor = motor(PORT2, ratio18_1, false);
motor LBMotor = motor(PORT11, ratio18_1, false);
motor RFMotor = motor(PORT7, ratio18_1, true);
motor RBMotor = motor(PORT14, ratio18_1, true);
inertial imu = inertial(PORT17);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
