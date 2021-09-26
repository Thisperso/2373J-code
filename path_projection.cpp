#include "vex.h"
#include "RobotState.h"
#include <vex_controller.h>
#include <vector>
#include <numeric>
#include <iostream>

int IX = 0;
int IY = 0;

int DX = 12;
int DY = 12;

std::vector<float> point_count;

float p = 10;

int main(){
  for(int p1 = 0; p1 <= p; ++p1){
    point_count.push_back((float)p1 * 1/ p );
    
  }

}



