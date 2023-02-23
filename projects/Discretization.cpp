#include "../headfiles/Discretization.h"

uint16_t Discretization(double angle,int Type) {
    if(Type == 2 ){
        int x;
        x = int(angle*100  / 2 / M_PI*360) ;
        return uint16_t (x);
    }
    else if(Type == 1) {//大电机
        int x = int(angle/25/ pow(2.0,12.0)+pow(2.0,11.0));
        return uint16_t (x);
    }
    else{//z轴
        int x = int(angle/90/ pow(2.0,12.0)+pow(2.0,11.0));
        return uint16_t (x);
    }
}
