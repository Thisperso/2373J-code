/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// inert                inertial      4               
// elevator             motor         2               
// lfdrive              motor         9               
// lbdrive              motor         7               
// rbdrive              motor         10              
// rfdrive              motor         8               
// rintake              motor         6               
// lintake              motor         5               
// roller               motor         3               
// opti                 optical       18              
// ---- END VEXCODE CONFIGURED DEVICES ----

//10 and 20 = left drive
//1 and 11 = right drive
//3 = inertial sensor
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

#include "math.h"
#include <tuple>
#include <stdio.h>

using namespace vex;

motor_group lft(lfdrive, lbdrive);
motor_group rht(rfdrive, rbdrive);

/* move_to function inputs:
  11 rows and 11 columns
  move_to(col, row); example:: (5, 4)
*/
  double encl = lfdrive.position(turns);
  double encr = rfdrive.position(turns);
  double rot = inert.rotation(degrees);

RobotState::RobotState(){
      lread = 0;
      rread = 0;
      wheel_size = 0;
      avgread = 0;
      gyrot = 0;
      xe = 0;
      ye = 0;
      dxe = 0;
      dye = 0;
      prevavg = 0;
      totaldist = 0;
      facing = 0;
      ox = 0;
      oy = 0;
    }

    void RobotState::Calculate(double  lenc, double renc, double gyr){
    double ldist = lenc * 12.56;
    double rdist = renc * 12.56;
    gyrot = gyr;
    avgread = (ldist + rdist)/2.0;

    dxe = (avgread - prevavg) * cos(gyrot*(M_PI/180));
    dye = (avgread - prevavg) * sin(gyrot*(M_PI/180));

    xe += dxe;
    ye += dye;
    facing = inert.rotation(degrees);

    ox = xe/12;
    oy = ye/12;

    prevavg = avgread;
    }

    double RobotState::GetX(){
      return ox;
    }

    double RobotState::GetY(){
      return oy;
    }

    double RobotState::GetAvg(){
      return prevavg;
    }
    
    double RobotState::GetGyro(){
      return facing;
    }

    void RobotState::reset(){
      prevavg = 0;
    }

RobotState robot_pos;


double ptunedrive;

int PID (double dist, double curr, double p){
  double mtrspeed = p *  (dist - curr);

  double velmax = 60;

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

  double velmax = 30;

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
  //rfdrive.setPosition(0, turns);
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
    Brain.Screen.print(ox);
    Brain.Screen.setCursor(2,1);
    Brain.Screen.print("y:");
    Brain.Screen.print(oy);
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

void Drive_to(double x, double y, double tolerance){
  lfdrive.setPosition(0, turns);
  rfdrive.setPosition(0, turns);

  double sx = start_x;
  double sy = start_y;

  double dx = x-sx;
  double dy = y-sy;

  double dist = sqrt(dy*dy + dx*dx);

  double angl = atan2(dy, dx) * 180.0/M_PI;
}



// 1 : red score
// 2 : blue score


// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  inert.calibrate();
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

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
  vex::task::sleep(2000);
  move_to(9,7);
  move_to(8,9);
  move_to(7,9);
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

  /*opti.setLightPower(100,pct);
  opti.setLight(ledState::on);
  Drivetrain.setDriveVelocity(90, percent);
  Drivetrain.setTurnVelocity(80,pct);
  roller.spin(directionType::fwd, 600, velocityUnits::rpm);
  vex::task::sleep(250);
  roller.stop(brakeType::brake);
  Drivetrain.driveFor(forward, 36, inches);
  Drivetrain.turnFor(right, 65, degrees);
  intakes.spin(fwd, 100, velocityUnits::pct);
  Drivetrain.driveFor(forward, 18 , inches);
  intakes.stop(brake);
  elevator.spin(directionType::fwd, 100, velocityUnits::pct);
  roller.spin(directionType::fwd, 100, velocityUnits::pct);
  vex::task::sleep(2500);
  elevator.stop(brakeType::brake);
  roller.stop(brakeType::brake);
  intakes.stop(brakeType::brake);
  Drivetrain.setDriveVelocity(70, pct);
  Drivetrain.driveFor(reverse, 25, inches);
  vex::task::sleep(375);
  Drivetrain.turnFor(right, 100/2, degrees);
  Drivetrain.setDriveVelocity(100, pct);
  Drivetrain.driveFor(forward, 6.2*12, inches);
  Drivetrain.setTurnVelocity(95, pct);
  vex::task::sleep(250);
  Drivetrain.turnFor(left, 90/2, degrees);
  intakes.spin(fwd, 100, velocityUnits::pct);
  elevator.spin(directionType::fwd, 100, velocityUnits::pct);
  roller.spin(directionType::fwd, 100, velocityUnits::pct);
  Drivetrain.driveFor(forward, 26, inches);
  vex::task::sleep(1750);
  intakes.stop(brakeType::brake);
  roller.stop(brakeType::brake);
  elevator.stop(brakeType::brake);
  intakes.spin(reverse, 100, pct);
  Drivetrain.driveFor(reverse, 10, inches);*/


  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

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
