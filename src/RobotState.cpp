#include "RobotState.h"
#include "vex.h"

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
