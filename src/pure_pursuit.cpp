#include "pure_pursuit.h"
#include "vex.h"

void Drive_to(double x, double y, double tolerance){
  lfdrive.setPosition(0, turns);
  rfdrive.setPosition(0, turns);
  double pid_tune = 4*12;
  double turn_tune = 3;
  
  double dist = 0;

  while (dist > tolerance){


  }

  double dx = x-sx;
  double dy = y-sy;

  double dist = sqrt(dy*dy + dx*dx);

  double angl = atan2(dy, dx) * 180.0/M_PI;
}

