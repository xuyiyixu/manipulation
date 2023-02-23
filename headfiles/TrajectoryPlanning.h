#ifndef MANIPULATOR_TRAJECTORYPLANNING_H
#define MANIPULATOR_TRAJECTORYPLANNING_H

#include <cmath>
#include <memory>
#include <queue>
#include <array>


using OFFSET_TYPE_e = enum class OFFSET_TYPE_e{
    WITHOUT_OFFSET_DOWN= 0,
    WITHOUT_OFFSET_UP,
    WITH_OFFSET_DOWN,
    WITH_OFFSET_UP,
};

struct ChangeInformation{
    double x_axis;
    double y_axis;
    double angle;
};

using Trajectory_Point_t = struct Trajectory_Point_t{
    struct {
        double x;
        double y;
        double z;
        int condition;
    } point;
    OFFSET_TYPE_e offsetType;
};

constexpr Trajectory_Point_t trajectory[] = {
        //中下
        {{-136.329 ,-218.380,0,0},OFFSET_TYPE_e::WITH_OFFSET_UP},
        {{-136.329 ,-218.380,0,0},OFFSET_TYPE_e::WITH_OFFSET_UP},
        {{-136.329 ,-218.380,0,0},OFFSET_TYPE_e::WITH_OFFSET_UP},

        //左上
        {{-125.034 ,-209.3630,0,1},OFFSET_TYPE_e::WITH_OFFSET_UP},
        {{-125.034 ,-209.3630,0,1},OFFSET_TYPE_e::WITH_OFFSET_UP},
        {{-125.034 ,-209.3630,0,1},OFFSET_TYPE_e::WITH_OFFSET_UP},

        //右上
        {{-106.951 ,-199.8970,0,2},OFFSET_TYPE_e::WITH_OFFSET_UP},
        {{-106.951 ,-199.8970,0,2},OFFSET_TYPE_e::WITH_OFFSET_UP},
        {{-106.951 ,-199.8970,0,2},OFFSET_TYPE_e::WITH_OFFSET_UP},

        //中上
        {{-156.402 ,-101.0860,0,3},OFFSET_TYPE_e::WITH_OFFSET_DOWN},
        {{-156.402 ,-101.0860,0,3},OFFSET_TYPE_e::WITH_OFFSET_DOWN},
        {{-156.402 ,-101.0860,0,3},OFFSET_TYPE_e::WITH_OFFSET_DOWN},
        //左下
        {{-156.402 ,-101.0860,0,4},OFFSET_TYPE_e::WITH_OFFSET_DOWN},
        {{-156.402 ,-101.0860,0,4},OFFSET_TYPE_e::WITH_OFFSET_DOWN},
        {{-156.402 ,-101.0860,0,4},OFFSET_TYPE_e::WITH_OFFSET_DOWN},
        //右下
        {{-156.402 ,-101.0860,0,5},OFFSET_TYPE_e::WITH_OFFSET_DOWN},
        {{-156.402 ,-101.0860,0,5},OFFSET_TYPE_e::WITH_OFFSET_DOWN},
        {{-156.402 ,-101.0860,0,5},OFFSET_TYPE_e::WITH_OFFSET_DOWN},
        //放回
        {{-150.872 ,-116.2990,3440,6},OFFSET_TYPE_e::WITH_OFFSET_UP},
        {{-170.756 ,-69.66370,0,6},OFFSET_TYPE_e::WITH_OFFSET_UP},
        {{-157.02 ,12.9620,0,6},OFFSET_TYPE_e::WITH_OFFSET_UP},
        {{-137.633 ,41.6390,0,6},OFFSET_TYPE_e::WITH_OFFSET_UP},

};



Trajectory_Point_t *Correct(ChangeInformation change);




#endif //MANIPULATOR_TRAJECTORYPLANNING_Hc
