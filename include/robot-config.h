using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor roller;
extern motor rintake;
extern motor rbdrive;
extern motor elevator;
extern motor rfdrive;
extern motor lintake;
extern motor lfdrive;
extern motor lbdrive;
extern inertial inert;
extern optical opti;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );