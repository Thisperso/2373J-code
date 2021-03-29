
#include "vex.h"
#include "RobotState.h"

using namespace vex;
// A global instance of competition+
competition Competition;

motor_group intakes (rintake, lintake);
motor_group score (elevator, roller);

motor_group Left (lfdrive, lbdrive);
motor_group Right (rfdrive, rbdrive);

smartdrive Drivetrain (Left, Right, inert, 12.56, 8, 13.75, distanceUnits::in);

motor_group lft(lfdrive, lbdrive);
motor_group rht(rfdrive, rbdrive);

/* move_to function inputs:
  11 rows and 11 columns
  move_to(col, row); example:: (5, 4)
*/
double encl = lfdrive.position(turns);
double encr = rfdrive.position(turns);
double rot = inert.rotation(degrees);

double ptunedrive;

int PID (double dist, double curr, double p){
  double mtrspeed = p *  (dist - curr);

  double velmax = 70;

  if (mtrspeed > velmax){
    mtrspeed = velmax;
  } 
  
  if (mtrspeed < -velmax){
    mtrspeed = -velmax;
  }

  return (int) mtrspeed; 
} 

double short_error(double cmd, double cur){

  double error = cmd - cur;

  if (error > 180){
    error = error - 360;
  }

  else if (error < -180){
    error = error + 360;
  }

  return error;
}

int Turn_PID (double degrees, double curr_t, double t){

  double error = short_error(degrees, curr_t);

  double mtrspeed = t * (error);

  double velmax = 70;

  if (mtrspeed > velmax){
    mtrspeed = velmax;
  } 
  
  if (mtrspeed < -velmax){
    mtrspeed = -velmax;
  }

  return (int) mtrspeed;
}

int drive(double dist, double hold_angle){
  lfdrive.setPosition(0, turns);
  rfdrive.setPosition(0, turns);
  vex::task::sleep(50);
  double curr = 0;
  ptunedrive = 4;
  double t = 1;
  double variance;
  Brain.Screen.clearScreen();
  Brain.Screen.print("move to: ");
  Brain.Screen.print(dist);
  while (fabs(dist-curr) > 4){
    double lrot = lfdrive.position(turns);
    double rrot = rfdrive.position(turns);
    variance = inert.rotation(degrees);

    double ldist = lrot*12.57;
    double rdist = rrot*12.57;

    double turn_error = short_error(hold_angle, variance);
    int lspeed = PID(dist, ldist, ptunedrive) + turn_error;
    int rspeed = PID(dist, rdist, ptunedrive) - turn_error;
    
    curr = (ldist+rdist)/2;

    lft.spin(forward, lspeed, pct);
    rht.spin(forward, rspeed, pct);

    robot_pos.Calculate(lfdrive.position(turns), rfdrive.position(turns), inert.rotation(degrees));
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("x:");
    Brain.Screen.print(robot_pos.GetX());
    Brain.Screen.setCursor(2,1);
    Brain.Screen.print("y:");
    Brain.Screen.print(robot_pos.GetY());
    Brain.Screen.setCursor(3,1);
    Brain.Screen.print("facing:");
    Brain.Screen.print(inert.rotation());

    /*Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("variance: ");
    Brain.Screen.print(variance);
    printf("variance: %.2f hold angle: %.2f turn error: %.2f lspeed: %d rspeed: %d ldist: %.2f rdist: %.2f dist: %.2f \n", variance, hold_angle, turn_error, lspeed, rspeed, ldist, rdist, curr);
    Brain.Screen.setCursor(2,1);
    Brain.Screen.print("hold angle: ");
    Brain.Screen.print(hold_angle);
    Brain.Screen.setCursor(3,1);
    Brain.Screen.print("turn error: ");
    Brain.Screen.print(turn_error);*/
    vex::task::sleep(10);
  }
  lft.stop(hold);
  rht.stop(hold);
  variance = inert.rotation(degrees);
  lfdrive.setPosition(0,turns);
  rfdrive.setPosition(0,turns);
  robot_pos.reset();
  return 1; // void function
}

int turn(int turn_d){
  double curr_t = inert.rotation(degrees);
  double turn_tune = 2;
  double variance;
  Brain.Screen.clearScreen();
  Brain.Screen.print("turn to: ");
  Brain.Screen.print(turn_d);
    while(fabs(short_error(turn_d, curr_t)) > 1){
      curr_t = inert.rotation(degrees);

      int b_speed = Turn_PID(turn_d, curr_t, turn_tune);

      lft.spin(fwd, b_speed, pct);
      rht.spin(reverse, b_speed, pct);
    }
  variance = inert.rotation(degrees);
  lft.stop(brake);
  rht.stop(brake);
  lfdrive.setRotation(0,turns);
  rfdrive.setRotation(0,turns);
  return 1;
}

struct starting_pos {
  int x;
  int y;
};


int start_x = 7;
int start_y = 11;
int prev_angl = 0;

void move_to(double row ,double col){
  //printf("start: %d %d end %.2f %.2f\n", start_x, start_y, row, col);
  struct starting_pos strt = {start_x, start_y};
  // (x,y)

  // starting postion
  double curr_col = strt.y;
  double curr_row = strt.x;

  // theoretical final destination
  double dest_col = col;
  double dest_row = row;

  double dx = dest_row-curr_row;
  double dy = dest_col-curr_col;

  double dist = sqrt(dy*dy + dx*dx);

  double angl = atan2(dy, dx) * 180.0/M_PI;

  double movement = dist*12;
  //printf("dx: %.2f dy: %.2f angle: %.2f row: %.2f col: %.2f\n", dx, dy, angl, dest_row, dest_col);
  //Brain.Screen.print(angl);
  turn(angl);
  drive(movement, angl);

  strt.x = dest_row;
  strt.y = dest_col;
  start_x = dest_row;
  start_y = dest_col;
  prev_angl = angl;
}

int ball_count = 0;
int screenx;
int screeny;
bool pressed = false;
int auton_select;

void press(){
  if (Brain.Screen.pressing()){
    pressed = true;
  }  

  else{
    pressed = false;
  }
}


int auton_ball_count(){
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(1,1, 100, 50);
  Brain.Screen.drawRectangle(60, 1, 100, 50);
  Brain.Screen.drawRectangle(120, 1, 100, 50);

  screenx = Brain.Screen.xPosition();
  screeny = Brain.Screen.yPosition();

  while (pressed == false){
    wait(5,msec);
    press();
  }

  if (screenx < 100 && screeny < 55){
    ball_count = 1;
  }

  else if (screenx < 100 &&  110 > screeny > 55){
    ball_count = 2;
  }

  else if (screenx < 100 && screeny > 110){
    ball_count = 3;
  }

  return 1;
}

int auton_selection(int ball_count){
  Brain.Screen.clearScreen();
  Brain.Screen.setFillColor(blue);
  Brain.Screen.drawRectangle(1,1, 100, 50);
  Brain.Screen.drawRectangle(60, 1, 100, 50);
  Brain.Screen.drawRectangle(120, 1, 100, 50);

  screenx = Brain.Screen.xPosition();
  screeny = Brain.Screen.yPosition();

  while (pressed == false){
    wait(5,msec);
    press();
  }

  if (screenx < 100 && screeny < 55){
    auton_select = 1;
  }

  else if (screenx < 100 &&  110 > screeny > 55){
    auton_select = 2;
  }

  else if (screenx < 100 && screeny > 110){
    auton_select = 3;
  }

  return auton_select;
}



void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Brain.Screen.setCursor(1,1);
  auton_ball_count();
  auton_selection(ball_count);


  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {

  inert.calibrate();
  while (inert.isCalibrating()){
    wait(10, msec);
  }

  //corner and mid-mid auton
  if (auton1.pressing()){
    drive(24, 225);
    turn(225);
    intakes.spin(reverse, 100, pct);
    drive(28, 225);
    intakes.stop(hold);
    score.spin(fwd,100,pct);
    wait(2, sec);
    drive(-28, 225);
    turn(0);
    drive(38, 0);
    intakes.spin(fwd, 100,pct);
    drive(-38, 0);
    intakes.spin(reverse, 100,pct);
    turn(45);
    drive(42, 45);
    score.spin(fwd,100,pct);
    wait(3, sec);
    drive(-10, 45);
  }

  //corner and mid home row right
  else if (auton2.pressing()){
    drive(24,0);
    turn(135);
    drive(28, 135);
    score.spin(fwd, 100, pct);
    wait(2, sec);
    intakes.spin(fwd, 100, pct);
    drive(-28, 135);
    intakes.stop(hold);
    turn(270);
    drive(38, 270);
    turn(180);
    drive(28, 180);
    score.spin(fwd, 100, pct);
    wait(2, sec);
    drive(-28, 180);
  }

  //corner and mid home row left
  else if (auton2.pressing() && auton1.pressing()){
    drive(24,0);
    turn(225);
    drive(28, 225);
    score.spin(fwd, 100, pct);
    wait(2, sec);
    intakes.spin(fwd, 100, pct);
    drive(-28, 225);
    intakes.stop(hold);
    turn(90);
    drive(38, 90);
    turn(180);
    drive(28, 180);
    score.spin(fwd, 100, pct);
    wait(2, sec);
    drive(-28, 180);
  }

  //skills auton
  else{
  intakes.spin(reverse, 100, pct);
  drive(28, 0);
  intakes.stop(hold);
  turn(135);
  drive(34, 135);
  score.spin(fwd, 100, pct);
  wait(2,sec);
  drive(-20, 135);
  turn(0);
  intakes.spin(fwd, 100, pct);
  drive(50, 0);
  turn(90);
  drive(20, 90);
  score.spin(fwd, 100, pct);
  wait(2,sec);
  score.stop(hold);
  drive(-20, 90);
  turn(0);
  drive(36, 0);
  turn(90);
  intakes.spin(fwd, 100, pct);
  drive(24, 90);
  drive(-24, 90);
  turn(0);
  drive(12, 0);
  turn(45);
  drive(24, 45);
  score.spin(fwd, 100, pct);
  wait(3,sec);
  score.stop(hold);
  drive(-24, 45);
  intakes.spin(fwd, 100, pct);
  }
  //skills autononomous
  /*turn(180);
  drive(-26, 180);
  drive(20, 180);
  drive(-26, 180);
  drive(20, 180);
  drive(-26, 180);
  drive(20, 180);
  drive(-26, 180);
  drive(20, 180);
  drive(-26, 180);
  drive(22, 180);
  turn(270);*/
  /*drive(46, 270);
  turn(0);
  intakes.spin(fwd, 100, pct);
  drive(74, 0);
  intakes.stop(hold);
  turn(-45);
  drive(20, -45);
  score.spin(fwd, 100, pct);
  wait(3, sec);
  intakes.spin(reverse, 100, pct);
  drive(-20,-45);
  intakes.stop(hold);*/

  //end of skills autonomous
  //move_to(11,11);
  /*inert.calibrate();
  score.spin(fwd, 100, pct);
  vex::task::sleep(2000);
  rfdrive.setPosition(0, turns);
  lfdrive.setPosition(0, turns);
  inert.setRotation(0, degrees);
  move_to(8,8);
  intakes.spin(fwd, 100, pct);
  move_to(10.5,11.5);
  vex::task::sleep(500);
  intakes.stop(hold);
  score.spin(fwd, 100, pct);
  vex::task::sleep(3000);
  score.stop(hold);
  move_to(9,9);*/


}

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {


    rfdrive.spin(directionType::fwd, (Controller1.Axis3.value() - (.8*Controller1.Axis1.value())), velocityUnits::pct); //(Axis3+Axis4)/2;
	  lfdrive.spin(directionType::fwd, (Controller1.Axis3.value() +  (.8*Controller1.Axis1.value())), velocityUnits::pct);//(Axis3-Axis4)/2;
    rbdrive.spin(directionType::fwd, (Controller1.Axis3.value() -  (.8*Controller1.Axis1.value())), velocityUnits::pct); //(Axis3+Axis4)/2;
	  lbdrive.spin(directionType::fwd, (Controller1.Axis3.value() +  (.8*Controller1.Axis1.value())), velocityUnits::pct);//(Axis3-Axis4)/2;
    
    // This is the main execution loop for the user control program.
    opti.setLight(ledState::on);
    opti.setLightPower(100, pct);

    if (Controller1.ButtonL2.pressing()){
        roller.setVelocity(100,percent);
        elevator.setVelocity(80,percent);
        if (opti.hue() > 50){
          elevator.spin(fwd);
          roller.spin(reverse);
        }
        else{
          elevator.spin(forward);
          roller.spin(forward);
        }
        elevator.spin(forward);
        roller.spin(forward);

      }
    

    else if (Controller1.ButtonL1.pressing()){
      elevator.spin(directionType::rev, 200, velocityUnits::pct);
      roller.spin(directionType::rev, 200, velocityUnits::pct);
    }

    else {
      roller.stop(brakeType::brake);
      elevator.stop(brakeType::brake);
    }
    
    if (Controller1.ButtonR1.pressing()){
      lintake.spin(directionType::rev, 100, velocityUnits::pct);
      rintake.spin(directionType::rev, 100, velocityUnits::pct);
      //elevator.spin(directionType::rev, 100, velocityUnits::pct);
    }

    else if (Controller1.ButtonR2.pressing()){
      lintake.spin(directionType::fwd, 100, velocityUnits::pct);
      rintake.spin(directionType::fwd, 100, velocityUnits::pct);
      //elevator.spin(directionType::fwd, 100, velocityUnits::pct);
    }

    else {
      lintake.stop(brakeType::brake);
      rintake.stop(brakeType::brake);
      //elevator.stop(brakeType::brake);
    }
    

    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(15, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}
//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
