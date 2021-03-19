#include "vex.h"
#include "math.h"

class RobotState{
  private:
    double lread;
    double rread;
    double wheel_size;
    double avgread;
    double gyrot;
    double xe;
    double ye;
    double dxe;
    double dye;
    double prevavg;
    double totaldist;
    double facing;
    double ox;
    double oy;
    
  public:
    RobotState();
    void Calculate(double, double, double);
    void reset();
    double GetX();
    double GetY();
    double GetAvg();
    double GetGyro();
};