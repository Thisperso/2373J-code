
#include "vex.h"
#include "RobotState.h"
#include <vex_controller.h>

using namespace vex;
// A global instance of competition+
competition Competition;

motor_group intakes (rintake, lintake);
motor_group score (elevator, roller);

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

  double velmax = 90;

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

  double mtrspeed = t * error;

  double velmax = 70;

  if (mtrspeed > velmax){
    mtrspeed = velmax;
  } 
  
  else if (mtrspeed < -velmax){
    mtrspeed = -velmax;
  }

  return (int) mtrspeed;
}

int drive(double dist, double hold_angle){
  lfdrive.setPosition(0, turns);
  rfdrive.setPosition(0, turns);
  vex::task::sleep(50);
  double curr = 0;
  ptunedrive = 6;
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
  double turn_tune = 4;
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

  if (angl - 180 == prev_angl){
    angl = prev_angl;
    movement = -1*movement;
  }
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



void display_position(){
  //controller display
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("x: ", robot_pos.GetX());
  Controller1.Screen.setCursor(2,1);
  Controller1.Screen.print("y: ", robot_pos.GetY());
  Controller1.Screen.setCursor(3,1);
  Controller1.Screen.print("heading: ", inert.rotation());

  //Brain display
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  Brain.Screen.print("x: ");
  Brain.Screen.print(robot_pos.GetX());
  Brain.Screen.setCursor(2,1);
  Brain.Screen.print("y: ");
  Brain.Screen.print(robot_pos.GetY());
  Brain.Screen.setCursor(3,1);
  Brain.Screen.print("heading: ");
  Brain.Screen.print(inert.rotation());
}



//DRIVE_TO FUNCTION
void driveTo(double x, double y, double tolerance){
  
  /*-------------------- LOGIC -------------------------*/
  /* inititalize variables and reset encoders           */
  /* **while loop ( while the dist > tolerance)**       */
  /* get current x and y                                */
  /* calculate the distance and angle to the final point*/
  /* send stuff into pid and calculate speeds           */
  /* write to the motors                                */
  /* exit while loop and move on                        */
  /*----------------------------------------------------*/

  //resetting encoders
  lfdrive.setRotation(0, turns);
  rfdrive.setRotation(0, turns);

  //establishing variables/ deltas
  double dist = 1 + tolerance; 
  double dx = 0;
  double dy = 0;

  //algorithm starts
  while ((fabs(dist) > tolerance)){

    

    //int speed = 5;

    //acceleration curve
    /*while (lfdrive.velocity(pct) < 60){

      turn_error = short_error(angl, inert.rotation(degrees));

      lft.spin(forward, speed + turn_error, pct);
      rht.spin(forward, speed - turn_error, pct);

      //reccuring math to increase speed slowly
      speed = speed + 5;
      
      //calculate position from start
      robot_pos.Calculate(lfdrive.rotation(turns), rfdrive.rotation(turns), inert.rotation(degrees));

      dx = x-robot_pos.GetX();
      dy = y-robot_pos.GetY();

      //calculate total distance and angle
      dist = sqrt(dy*dy + dx*dx);
      angl = atan2(dy, dx) * 180.0/M_PI;

      //check to see if the point has been reached
      if (dist <= tolerance){
        exit = true;
        break;
      }

      display_position();
      wait(35, msec);
    }

    //quick exit function for acceleration arrival
    if (exit){
      break;
    }*/

    

    //re-calculating the current position
    robot_pos.Calculate(lfdrive.rotation(turns), rfdrive.rotation(turns), inert.rotation(degrees));

    //calculating delta x and y
    dx = x-robot_pos.GetX();
    dy = y-robot_pos.GetY();

    //calculating distance and angle to the next coordinate/point;
    double dist = sqrt(dy*dy + dx*dx);
    double angl = atan2(dy, dx) * 180.0/M_PI;

    //calcualting the turn error and speeds
    double turn_error = Turn_PID(angl, inert.rotation(degrees), .75);

    //displaying the position on the field

    double lspeed = PID(dist, 0, 35) + turn_error;
    double rspeed = PID(dist, 0, 35) - turn_error;

    //motor drive functions
    lft.spin(forward, lspeed, pct);
    rht.spin(forward, rspeed, pct);
    //display_position();

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("x: ");
    Brain.Screen.print(robot_pos.GetX());
    Brain.Screen.setCursor(1,20);
    Brain.Screen.print("goal x: ");
    Brain.Screen.print(x);
    Brain.Screen.setCursor(2,20);
    Brain.Screen.print("goal y: ");
    Brain.Screen.print(y);

    Brain.Screen.setCursor(2,1);
    Brain.Screen.print("y: ");
    Brain.Screen.print(robot_pos.GetY());
    Brain.Screen.setCursor(3,1);
    Brain.Screen.print("heading: ");
    Brain.Screen.print(inert.rotation());

    Brain.Screen.setCursor(5,1);
    Brain.Screen.print("lft: ");
    Brain.Screen.print(lspeed);
    Brain.Screen.setCursor(6,1);
    Brain.Screen.print("rht: ");
    Brain.Screen.print(rspeed);
    Brain.Screen.setCursor(7,1);
    Brain.Screen.print("dist: ");
    Brain.Screen.print(fabs(dist));

    Brain.Screen.setCursor(9,1);
    Brain.Screen.print("tolerance: ");
    Brain.Screen.print(tolerance);
    Brain.Screen.setCursor(10,1);
    Brain.Screen.print("absolute distance: ");
    Brain.Screen.print(bool (fabs(dist) > tolerance));
    if ((fabs(dist) > tolerance) == false){
      break;
    }
    Brain.Screen.setCursor(11,1);
    Brain.Screen.print("not done");
    wait(10, msec);
  }



//Brain.Screen.clearScreen();
    Brain.Screen.setCursor(11,1);
    Brain.Screen.print("done");

}




/*int ball_count = 0;
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

*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
}

void autonomous(void) {

  inert.calibrate();
  while (inert.isCalibrating()){
    wait(10, msec);
  }

  //------------------------ ISM TESTING -------------------------//

    driveTo(3, 0, 3);
    driveTo(3, -4, 3);
    driveTo(3, 0, 3);
    driveTo(3, 3, 3);
    driveTo(-4, 3, 3);
    driveTo(0, 1, .5);
    driveTo(1, 3, 2);
    driveTo(3, 0, .5);
    lft.stop(hold);
    rht.stop(hold);

    /*driveTo(6, 6, 2);
    driveTo(4, 4, .5);*/

  //------------------------ END ISM TESTING ---------------------//


  //corner and mid-mid auton
  /*if (auton1.pressing()){
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
*/
  //skills auton
  //else{
    
  //first tower
  /*intakes.spin(reverse, 100, pct);
  drive(28, 0);
  intakes.stop(hold);
  turn(135);
  drive(34, 135);
  score.spin(fwd, 100, pct);
  wait(2,sec);
  score.stop(hold);

  //second tower
  drive(-18, 135);
  turn(0);
  intakes.spin(fwd, 100, pct);
  elevator.spin(fwd, 100, pct);
  drive(48, 5);
  turn(90);
  intakes.spin(reverse,100,pct);
  drive(14, 90);
  score.spin(fwd, 100, pct);
  wait(2,sec);
  score.stop(hold);

  //third tower
  drive(-18, 90);
  turn(0);
  drive(36, 0);
  turn(90);
  intakes.spin(fwd, 100, pct);
  drive(23, 90);
  drive(-23, 90);
  turn(0);
  drive(8, 0);
  turn(45);
  drive(24, 45);
  score.spin(fwd, 100, pct);
  wait(3,sec);
  score.stop(hold);

  //fourth tower
  drive(-54, 45);
  turn(273);
  intakes.spin(fwd, 100, pct);
  elevator.spin(fwd, 100, pct);
  drive(24, 273);
  turn(0);
  intakes.spin(reverse, 100, pct);
  drive(44, 0);
  score.spin(fwd, 100, pct);
  wait(2,sec);
  score.stop(hold);

  //fifth tower
  drive(-20, 0);
  intakes.spin(fwd,100,pct);
  turn(273);
  drive(50, 273);
  turn(-45);
  drive(24, -45);
  intakes.spin(reverse, 100, pct);
  score.spin(fwd, 100, pct);
  wait(2,sec);
  score.stop(hold);

  //sixth tower
  drive(-24, -45);
  turn(183);
  intakes.spin(fwd,100,pct);
  elevator.spin(fwd, 100, pct);
  drive(50, 183);
  turn(270);
  intakes.spin(reverse, 100, pct);
  drive(14, 270);
  score.spin(fwd, 100, pct);
  wait(2,sec);
  score.stop(hold);

  //seventh tower
  drive(-18, 270);
  intakes.spin(fwd,100,pct);
  elevator.spin(fwd, 100, pct);
  turn(90);
  drive(24, 90);
  turn(180);
  drive(24, 180);
  turn(225);
  intakes.spin(reverse, 100, pct);
  drive(60, 225);
  score.spin(fwd, 100, pct);
  wait(2,sec);
  score.stop(hold);

  //eighth tower 
  drive(-34, 225);
  turn(270);
  intakes.spin(fwd, 100, pct);
  elevator.spin(fwd, 100, pct);
  drive(23, 225);
  drive(-23, 225);
  turn(135);
  drive(13, 135);
  turn(90);
  drive(26, 90);
  turn(180);
  drive(16, 180);
  score.spin(fwd, 100, pct);
  wait(2,sec);
  score.stop(hold);

  //ninth tower
  drive(-18, 180);
  turn(0);
  intakes.spin(fwd, 100, pct);
  elevator.spin(fwd, 100, pct);
  drive(24, 0);
  drive(24, 10);
  drive(-12, 10);
  drive(8, 0);
  score.spin(fwd, 100, pct);
  wait(2,sec);
  score.stop(hold);
  drive(-8, 0);*/

  //}
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
