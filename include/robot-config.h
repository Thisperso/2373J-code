using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor lft1;
extern motor lft2;
extern motor lft3;
extern motor rht1;
extern motor rht2;
extern motor rht3;
extern inertial inert;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );