#include "../headfiles/Spline.h"

//三次样条重写了再发


/**
 * IK逆运动学程序，将xy转换为θ
 * @param x x坐标                                                                                                                                                                             * @param y y坐标
 * @return 相应的θ
 */


Target target = {-157.02 ,12.9620,0.0, 1};

AfterCorrect Inverse(Trajectory_Point_t CorrectPoint, AfterCorrect Before) {
    double theta[3];
    AfterCorrect L;
    //后面会为已知
    const double l1 = 105.9, l2 = 154.8;
    //是theta2
    theta[1] = acos(
            (CorrectPoint.point.x * CorrectPoint.point.x + CorrectPoint.point.y * CorrectPoint.point.y -
             l1 * l1 - l2 * l2) / l1 / l2 / 2);//θ2第二个关节的角度
    theta[2] = acos(
            (CorrectPoint.point.x * CorrectPoint.point.x + CorrectPoint.point.y * CorrectPoint.point.y +
             l1 * l1 - l2 * l2) / 2 /
            sqrt(CorrectPoint.point.x * CorrectPoint.point.x +
                 CorrectPoint.point.y * CorrectPoint.point.y) / l1);//一个参考角度
    if(abs(Before.ThetaTwo - theta[1]) >= abs(Before.ThetaTwo + theta[1])){
        theta[1] = -theta[1];
        theta[2] = -theta[2];
    }
    else {
        theta[1] = theta[1];
        theta[2] = theta[2];
    }
    theta[0] = atan(CorrectPoint.point.y / CorrectPoint.point.x) - theta[2];
    L.Zaixs = CorrectPoint.point.z;
    L.ThetaOne = theta[0];
    L.ThetaTwo = theta[1];
    return L;
}

/**
 * 逆运动学分类
 * @param1   12得改，改initial
 * @return
 */
Position Divide(const Trajectory_Point_t *CorrectPoint){
    Position final;
    AfterCorrect initial = {1.5369, 0.947845,10};
    AfterCorrect k;
    for(int i = 0;i < 10;i++){
        switch (CorrectPoint[i].point.condition) {
            case 0:
                if(final.LeftDown.empty())
                    final.LeftDown.push(Inverse(CorrectPoint[i], initial));
                else if(final.LeftDown.size() == 2){
                    Trajectory_Point_t w1 = {{target.x_final,target.y_final,target.z_final,4},OFFSET_TYPE_e::WITH_OFFSET_DOWN};
                    for(int m = 0; m < 10;m++){
                        k = Inverse(w1,final.LeftDown.front());
                        k.status = 1;
                        final.LeftDown.push(k);
                    }
                }
                else
                    final.LeftDown.push(Inverse(CorrectPoint[i], final.LeftDown.front()));
                break;
            case 1:
                if(final.MiddleDown.empty())
                    final.MiddleDown.push(Inverse(CorrectPoint[i],initial));
                else if(final.MiddleDown.size() == 2){
                    Trajectory_Point_t w1 = {{target.x_final,target.y_final,target.z_final,0},OFFSET_TYPE_e::WITH_OFFSET_DOWN};
                    for(int m = 0; m < 10;m++){
                        k = Inverse(w1,final.MiddleDown.front());
                        k.status = 1;
                        final.MiddleDown.push(k);
                    }
                }
                else
                    final.MiddleDown.push(Inverse(CorrectPoint[i],  final.MiddleDown.front()));
                break;
            case 2:
                if(final.RightDown.empty())
                    final.RightDown.push(Inverse(CorrectPoint[i],initial));
                else if(final.RightDown.size() == 2){
                    Trajectory_Point_t w1 = {{target.x_final,target.y_final,target.z_final,5},OFFSET_TYPE_e::WITH_OFFSET_DOWN};
                    for(int m = 0; m < 10;m++){
                        k = Inverse(w1,final.RightDown.front());
                        k.status = 1;
                        final.RightDown.push(k);
                    }
                }
                else
                    final.RightDown.push(Inverse(CorrectPoint[i], final.RightDown.front()));
                break;
            case 3:
                if(final.LeftUp.empty())
                    final.LeftUp.push(Inverse(CorrectPoint[i],initial));
                else if(final.LeftUp.size() == 2){
                    Trajectory_Point_t w1 = {{target.x_final,target.y_final,target.z_final,1},OFFSET_TYPE_e::WITH_OFFSET_DOWN};
                    for(int m = 0; m < 10;m++){
                        k = Inverse(w1,final.LeftUp.front());
                        k.status = 1;
                        final.LeftUp.push(k);
                    }
                }
                else
                    final.LeftUp.push(Inverse(CorrectPoint[i], final.LeftUp.front()));
                break;
            case 4:
                if(final.MiddleUp.empty())
                    final.MiddleUp.push(Inverse(CorrectPoint[i], initial));
                else if(final.MiddleUp.size() == 2){
                    Trajectory_Point_t w1 = {{target.x_final,target.y_final,target.z_final,3},OFFSET_TYPE_e::WITH_OFFSET_DOWN};
                    for(int m = 0; m < 10;m++){
                        k = Inverse(w1,final.MiddleUp.front());
                        k.status = 1;
                        final.MiddleUp.push(k);
                    }
                }
                else
                    final.MiddleUp.push(Inverse(CorrectPoint[i],  final.MiddleUp.front()));
                break;
            case 5:
                if(final.RightUp.empty())
                    final.RightUp.push(Inverse(CorrectPoint[i],initial));
                else if(final.RightUp.size() == 2){
                    Trajectory_Point_t w1 = {{target.x_final,target.y_final,target.z_final,2},OFFSET_TYPE_e::WITH_OFFSET_DOWN};
                    for(int m = 0; m < 10;m++){
                        k = Inverse(w1,final.RightUp.front());
                        k.status = 1;
                        final.RightUp.push(k);
                    }
                }
                else
                    final.RightUp.push(Inverse(CorrectPoint[i],  final.RightUp.front()));
                break;
            case 6:
                if(final.PutBack.empty())
                    final.PutBack.push(Inverse(CorrectPoint[i], initial));
                else if(final.PutBack.size() == 2){
                    Trajectory_Point_t w1 = {{target.x_final,target.y_final,target.z_final,6},OFFSET_TYPE_e::WITH_OFFSET_DOWN};
                    k = Inverse(w1,final.PutBack.front());
                    k.status = 1;
                    for(int m = 0; m < 10;m++){
                        final.PutBack.push(k);
                    }
                }
                else
                    final.PutBack.push(Inverse(CorrectPoint[i],final.PutBack.front()));
                break;
        }
    }
    return final;
}
/**
 * 三次样条规划，θ1 θ2 和 z 均通过一个三次样条规划
 * @param 输入是一个队列，前面需要经过switch来得到相应的队列
 * @return FINAL 里面是三个的队列
 */
AfterSplinePoint Spline(std::queue<AfterCorrect> insert){
    int n = insert.size();
    std::cout << n << std::endl;
    double time[n],theta1[n],theta2[n],z[n];
    AfterSplinePoint FINAL;
    double time_i,theta_1,theta_2,z_0;
    double k = 0.0;
    double change;
    for(int i = 0;i < n; i++){
        time[i] = k ;
        theta1[i] = insert.front().ThetaOne;
        theta2[i] = insert.front().ThetaTwo;
        z[i] = insert.front().Zaixs;
        std::cout << theta1[i] << " " << theta2[i] << " " << z[i] <<std::endl;
        k += 1.0;
        if(insert.front().status == 1)
            change = z[i];
        insert.pop();
    }
    gsl_interp_accel *acc
            = gsl_interp_accel_alloc ();
    gsl_spline *spline_theta1
            = gsl_spline_alloc (gsl_interp_cspline, n);
    gsl_spline *spline_theta2
            = gsl_spline_alloc (gsl_interp_cspline, n);
    gsl_spline *spline_z
            = gsl_spline_alloc (gsl_interp_cspline, n);

    gsl_spline_init (spline_theta1, time, theta1, n);
    gsl_spline_init (spline_theta2, time, theta2, n);
    gsl_spline_init (spline_z, time, z, n);

    for (time_i = time[0]; time_i <= time[n-1];){
        theta_1 = gsl_spline_eval (spline_theta1, time_i, acc);
        theta_2 = gsl_spline_eval (spline_theta2, time_i, acc);
        z_0 = gsl_spline_eval (spline_z, time_i, acc);
        FINAL.Theta_1.push(theta_1);
        FINAL.Theta_2.push(theta_2);
        FINAL.Z.push(z_0);
        if(z_0 == change )
            FINAL.st.push(1);
        else FINAL.st.push(-1);
        std::cout << theta_1 << " " << theta_2 << " " << z_0 << std::endl;
        time_i += 0.5;
    }
    gsl_spline_free (spline_theta1);
    gsl_spline_free (spline_theta2);
    gsl_spline_free (spline_z);
    gsl_interp_accel_free (acc);
    return FINAL;//之后改为struct
}

AfterSplinePoint Connection(int signal){
    //之后为函数调用
    ChangeInformation m = {0.1,0.1,3};
    Position final = Divide(Correct(m));
    std::cout << final.PutBack.size();
    int x;
    switch (signal) {
        case 3:
            return Spline(final.LeftUp);
        case 4:
            return Spline(final.MiddleUp);
        case 5:
            return Spline(final.RightUp);
        case 0:
            return Spline(final.LeftDown);
        case 1:
            return Spline(final.MiddleDown);
        case 2:
            return Spline(final.RightDown);
        default :
            return Spline(final.PutBack);
    }
}

