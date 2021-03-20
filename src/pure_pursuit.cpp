#include "pure_pursuit.h"
#include "vex.h"

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

void Drive_to(double x, double y, double tolerance){
  lfdrive.setPosition(0, turns);
  rfdrive.setPosition(0, turns);
  double pid_tune = 4*12;
  double turn_tune = 3;
  
  double dist = 0;

  while (dist > tolerance){
    double dx = x-robot_pos.GetX();
    double dy = y-robot_pos.GetY();

    dist = sqrt(dy*dy + dx*dx);

    double angl = atan2(dy, dx) * 180.0/M_PI;

    double turn_error = turn_tune*(angl - inert.rotation());
    
    double speed = PID(0, dist, pid_tune);

  }
}

