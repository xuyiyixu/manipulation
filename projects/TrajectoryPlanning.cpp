#include "../headfiles/TrajectoryPlanning.h"

//由于z方向不会出现误差，故不写校正，直接定，三维转二维
//距离150mm
//x向右，y向后，z向下
//原点 距物体
//cm会不会更好一些



constexpr size_t TRAJ_POINT_NUM = sizeof(trajectory) / sizeof(Trajectory_Point_t);
Trajectory_Point_t Point[TRAJ_POINT_NUM];
Trajectory_Point_t *Correct(ChangeInformation change) {
    Trajectory_Point_t initial;
    //参数注意改
    for (int i = 0; i < 10 ; i++) {
        initial.point.condition = trajectory->point.condition;
        initial.offsetType = trajectory->offsetType;
        switch (trajectory[i].offsetType) {
            case OFFSET_TYPE_e::WITHOUT_OFFSET_DOWN:
            case OFFSET_TYPE_e::WITHOUT_OFFSET_UP: {
                initial.point.x = trajectory->point.x;
                initial.point.y = trajectory->point.y;
                initial.point.z = trajectory->point.z;
                break;
            }
            case OFFSET_TYPE_e::WITH_OFFSET_DOWN:
            case OFFSET_TYPE_e::WITH_OFFSET_UP: {
                initial.point.x = trajectory->point.x + change.x_axis;
                initial.point.y = trajectory->point.y + change.y_axis;
                initial.point.z = trajectory->point.z;
                break;
            }
        }
        Point[i] = initial;
    }
    return Point;
}

