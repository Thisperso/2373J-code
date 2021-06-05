#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor roller = motor(PORT1, ratio6_1, false);
motor rintake = motor(PORT11, ratio18_1, false);
motor rbdrive = motor(PORT12, ratio18_1, true);
motor elevator = motor(PORT15, ratio18_1, true);
motor rfdrive = motor(PORT16, ratio18_1, false);
motor lintake = motor(PORT18, ratio18_1, true);
motor lfdrive = motor(PORT19, ratio18_1, true);
motor lbdrive = motor(PORT20, ratio18_1, false);
inertial inert = inertial(PORT17);
optical opti = optical(PORT14);
bumper auton1 = bumper(Brain.ThreeWirePort.A);
bumper auton2 = bumper(Brain.ThreeWirePort.B);
bumper auton3 = bumper(Brain.ThreeWirePort.C);
bumper auton4 = bumper(Brain.ThreeWirePort.D);
bumper auton5 = bumper(Brain.ThreeWirePort.E);
bumper presentation = bumper(Brain.ThreeWirePort.F);

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