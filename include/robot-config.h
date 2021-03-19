using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern inertial inert;
extern motor elevator;
extern motor lfdrive;
extern motor lbdrive;
extern motor rbdrive;
extern motor rfdrive;
extern motor rintake;
extern motor lintake;
extern motor roller;
extern optical opti;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );