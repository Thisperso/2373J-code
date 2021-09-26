// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// lft1                 motor         1               
// lft2                 motor         2               
// lft3                 motor         3               
// rht1                 motor         11              
// rht2                 motor         12              
// rht3                 motor         13              
// inert                inertial      19              
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// lft1                 motor         1               
// lft2                 motor         2               
// lft3                 motor         3               
// rht1                 motor         11              
// rht2                 motor         12              
// rht3                 motor         13              
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// lft1                 motor         1               
// lft2                 motor         2               
// lft3                 motor         3               
// rht1                 motor         11              
// rht2                 motor         12              
// rht3                 motor         13              
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// lft1                 motor         1               
// lft2                 motor         2               
// lft3                 motor         3               
// rht1                 motor         11              
// rht2                 motor         12              
// rht3                 motor         13              
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// lft1                 motor         1               
// lft2                 motor         2               
// lft3                 motor         3               
// rht1                 motor         11              
// rht2                 motor         12              
// rht3                 motor         13              
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// lft1                 motor         1               
// lft2                 motor         2               
// lft3                 motor         3               
// rht1                 motor         11              
// rht2                 motor         12              
// rht3                 motor         7               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// lft1                 motor         1               
// lft2                 motor         2               
// lft3                 motor         3               
// rht1                 motor         11              
// rht2                 motor         6               
// rht3                 motor         7               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// lft1                 motor         1               
// lft2                 motor         2               
// lft3                 motor         3               
// rht1                 motor         5               
// rht2                 motor         6               
// rht3                 motor         7               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// lft1                 motor         1               
// lft2                 motor         2               
// lft3                 motor         3               
// rht1                 motor         5               
// rht2                 motor         6               
// rht3                 motor         7               
// inert                inertial      11              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "RobotState.h"
#include <vex_controller.h>

using namespace vex;
// A global instance of competition+
competition Competition;

motor_group lft(lft1, lft2, lft3);
motor_group rht(rht1, rht2, rht3);

/* move_to function inputs:
  11 rows and 11 columns
  move_to(col, row); example:: (5, 4)
*/

double encl = lft2.position(turns);
double encr = rht2.position(turns);
double rot = inert.rotation(degrees);

double ptunedrive;
double dtunedrive;


int PID (double dist, double curr, double p, double d){
  double prevError = 0.0;
  double Error = dist-curr;
  double dError = Error-prevError;

  double mtrspeed = p * Error + d * dError;

  double velmax = 85;

  if (mtrspeed > velmax){
    mtrspeed = velmax;
  } 
  
  if (mtrspeed < -velmax){
    mtrspeed = -velmax;
  }

  prevError = Error;

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

  double velmax = 85;

  if (mtrspeed > velmax){
    mtrspeed = velmax;
  } 
  
  else if (mtrspeed < -velmax){
    mtrspeed = -velmax;
  }

  return (int) mtrspeed;
}

int drive(double dist, double hold_angle){
  lft2.setPosition(0, turns);
  rht2.setPosition(0, turns);
  vex::task::sleep(50);
  double curr = 0;
  ptunedrive = 6;
  dtunedrive = 6;
  double t = 1;
  double variance;
  Brain.Screen.clearScreen();
  Brain.Screen.print("move to: ");
  Brain.Screen.print(dist);
  while (fabs(dist-curr) > 4){
    double lrot = lft2.position(turns);
    double rrot = rht2.position(turns);
    variance = inert.rotation(degrees);

    double ldist = lrot*12.57;
    double rdist = rrot*12.57;

    double turn_error = short_error(hold_angle, variance);
    int lspeed = PID(dist, ldist, ptunedrive, dtunedrive) + turn_error;
    int rspeed = PID(dist, rdist, ptunedrive, dtunedrive) - turn_error;
    
    curr = (ldist+rdist)/2;

    lft.spin(forward, lspeed, pct);
    rht.spin(forward, rspeed, pct);

    robot_pos.Calculate(lft2.position(turns), rht2.position(turns), inert.rotation(degrees));
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
  lft2.setPosition(0,turns);
  rht2.setPosition(0,turns);
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
      robot_pos.Calculate(lft.rotation(turns), rht.rotation(turns), inert.rotation(degrees));
      curr_t = inert.rotation(degrees);

      int b_speed = Turn_PID(turn_d, curr_t, turn_tune);

      lft.spin(fwd, b_speed, pct);
      rht.spin(reverse, b_speed, pct);
    }
  variance = inert.rotation(degrees);
  lft.stop(brake);
  rht.stop(brake);
  lft.setRotation(0,turns);
  rht.setRotation(0,turns);
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
  Controller1.Screen.print("x: ");
  Controller1.Screen.print(robot_pos.GetX());
  Controller1.Screen.setCursor(2,1);
  Controller1.Screen.print("y: ");
  Controller1.Screen.print(robot_pos.GetY());
  

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
void driveTo(double x, double y, double tolerance, bool backwards){
  
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

  //establishing variables/ deltas
  double dist = 1 + tolerance; 
  double dx = 0;
  double dy = 0;
  double myAngl = 0;

  double idx = x-robot_pos.GetX();
  double idy = y-robot_pos.GetY();
  double idist = sqrt(idy*idy + idx*idx);
  double iangl = atan2(dy, dx) * 180.0/M_PI;
  double accel_speed = 7;
  //algorithm starts
  while ((fabs(dist) > tolerance*dist)){

    //re-calculating the current position
    robot_pos.Calculate(lft.rotation(turns), rht.rotation(turns), inert.rotation(degrees));

    //calculating delta x and y
    dx = x-robot_pos.GetX();
    dy = y-robot_pos.GetY();

    //calculating distance and angle to the next coordinate/point;
    double dist = sqrt(dy*dy + dx*dx);
    double angl = atan2(dy, dx) * 180.0/M_PI;

    


    //calcualting the turn error and speeds

    /*if ((idist - dist) > idist/12){
      angl = iangl;
    }*/

    if (backwards){
      dist = -dist;
      angl = angl+180;
    }

    double turn_error = Turn_PID(angl, inert.rotation(degrees), .75);

    double speed = PID(dist, 0, 90, 90); //+ turn_error;
    //double rspeed = PID(dist, 0, 90) - turn_error;

    /*if ((speed - lft.velocity(pct)) > 50){
    speed = 50;
    }*/

    //motor drive functions

    double speed_avg = (rht.velocity(pct) + lft.velocity(pct))/2;

    if ((speed - speed_avg) > 30){
      accel_speed += 12;
      speed = accel_speed;
    }

    lft.spin(fwd, speed + turn_error, pct);
    rht.spin(fwd, speed - turn_error, pct);
    display_position();

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
    Brain.Screen.print(PID(dist, 0, 90, 90) + turn_error);
    Brain.Screen.setCursor(6,1);
    Brain.Screen.print("rht: ");
    Brain.Screen.print(PID(dist, 0, 90, 90) - turn_error);
    Brain.Screen.setCursor(7,1);
    Brain.Screen.print("dist: ");
    Brain.Screen.print(fabs(dist));

    Brain.Screen.setCursor(9,1);
    Brain.Screen.print("angl: ");
    Brain.Screen.print(angl);
    Brain.Screen.setCursor(10,1);
    Brain.Screen.print("absolute distance: ");
    Brain.Screen.print(bool (fabs(dist) > tolerance));
    Controller1.Screen.setCursor(3,1);
    Controller1.Screen.print("angl: ");
    Controller1.Screen.print(angl);
    if ((fabs(dist) > tolerance) == false){
      break;
    }
    Brain.Screen.setCursor(11,1);
    Brain.Screen.print("not done");
    wait(10, msec);

   /*static int hold = 3;
   if(hold == 0)
 {   while (1){
      ;
    }
 } hold--;
 */
  }
    //Brain.Screen.clearScreen();
    Brain.Screen.setCursor(11,1);
    Brain.Screen.print("done");

}

std::pair <double,double> inter_point;

void driveToAngle(double x, double y, double incomingAngle, double tolerance, bool backwards){
  
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

  //establishing variables/ deltas
  double dist = 1 + tolerance; 
  double dx = 0;
  double dy = 0;
  double myAngl = 0;

  double idx = x-robot_pos.GetX();
  double idy = y-robot_pos.GetY();
  double idist = sqrt(idy*idy + idx*idx);
  double iangl = atan2(dy, dx) * 180.0/M_PI;
  double accel_speed = 7;
  double spacingCount = 8;
  double spacing_tolerance = tolerance/spacingCount;

  
  
  //algorithm starts
  while ((fabs(dist) > tolerance)){

    //re-calculating the current position
    robot_pos.Calculate(lft.rotation(turns), rht.rotation(turns), inert.rotation(degrees));

    //calculating delta x and y
    dx = x-robot_pos.GetX();
    dy = y-robot_pos.GetY();

    //calculating distance and angle to the next coordinate/point;
    double dist = sqrt(dy*dy + dx*dx);
    double angl = atan2(dy, dx) * 180.0/M_PI;

    


    //calcualting the turn error and speeds

    /*if ((idist - dist) > idist/12){
      angl = iangl;
    }*/

    if (backwards){
      dist = -dist;
      angl = angl+180;
    }

    double turn_error = Turn_PID(angl, inert.rotation(degrees), .75);

    double speed = PID(dist, 0, 90, 90); //+ turn_error;
    //double rspeed = PID(dist, 0, 90) - turn_error;

    /*if ((speed - lft.velocity(pct)) > 50){
    speed = 50;
    }*/

    //motor drive functions

    double speed_avg = (rht.velocity(pct) + lft.velocity(pct))/2;

    if ((speed - speed_avg) > 30){
      accel_speed += 12;
      speed = accel_speed;
    }

    lft.spin(fwd, speed + turn_error, pct);
    rht.spin(fwd, speed - turn_error, pct);
    display_position();

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
    Brain.Screen.print(PID(dist, 0, 90, 90) + turn_error);
    Brain.Screen.setCursor(6,1);
    Brain.Screen.print("rht: ");
    Brain.Screen.print(PID(dist, 0, 90, 90) - turn_error);
    Brain.Screen.setCursor(7,1);
    Brain.Screen.print("dist: ");
    Brain.Screen.print(fabs(dist));

    Brain.Screen.setCursor(9,1);
    Brain.Screen.print("angl: ");
    Brain.Screen.print(angl);
    Brain.Screen.setCursor(10,1);
    Brain.Screen.print("absolute distance: ");
    Brain.Screen.print(bool (fabs(dist) > tolerance));
    Controller1.Screen.setCursor(3,1);
    Controller1.Screen.print("angl: ");
    Controller1.Screen.print(angl);
    if ((fabs(dist) > tolerance) == false){
      break;
    }
    Brain.Screen.setCursor(11,1);
    Brain.Screen.print("not done");
    wait(10, msec);

   /*static int hold = 3;
   if(hold == 0)
 {   while (1){
      ;
    }
 } hold--;
 */
  }
    //Brain.Screen.clearScreen();
    Brain.Screen.setCursor(11,1);
    Brain.Screen.print("done");

}


void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
}

void autonomous(void) {

  inert.calibrate();
  while (inert.isCalibrating()){
    wait(10, msec);
  }
}

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {


    rht.spin(directionType::fwd, (Controller1.Axis3.value() - (.8*Controller1.Axis1.value())), velocityUnits::pct); //(Axis3+Axis4)/2;
	  lft.spin(directionType::fwd, (Controller1.Axis3.value() +  (.8*Controller1.Axis1.value())), velocityUnits::pct);//(Axis3-Axis4)/2;
    
    // This is the main execution loop for the user control program.
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
