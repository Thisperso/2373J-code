#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
inertial inert = inertial(PORT4);
motor elevator = motor(PORT2, ratio18_1, true);
motor lfdrive = motor(PORT9, ratio18_1, true);
motor lbdrive = motor(PORT7, ratio18_1, false);
motor rbdrive = motor(PORT10, ratio18_1, true);
motor rfdrive = motor(PORT8, ratio18_1, false);
motor rintake = motor(PORT6, ratio6_1, false);
motor lintake = motor(PORT5, ratio6_1, true);
motor roller = motor(PORT3, ratio18_1, true);
optical opti = optical(PORT18);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}