#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor lft1 = motor(PORT1, ratio18_1, false);
motor lft2 = motor(PORT2, ratio18_1, false);
motor lft3 = motor(PORT3, ratio18_1, false);
motor rht1 = motor(PORT11, ratio18_1, true);
motor rht2 = motor(PORT12, ratio18_1, true);
motor rht3 = motor(PORT13, ratio18_1, true);
inertial inert = inertial(PORT19);

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