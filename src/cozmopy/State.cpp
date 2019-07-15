
#include "State.h"

namespace libcozmo {

    //State::State() : State(0, 0, 0) { } 
    //State::State(int w, int h, int th) : State( w,  h,  th) {}

} //namespace libcozmo


SE2::State Cozmo::createState(const double x, const double y, const double th) 
{
    SE2::State s;
    Eigen::Isometry2d t = Eigen::Isometry2d::Identity();
    const Eigen::Rotation2D<double> rot(th);
    t.linear() = rot.toRotationMatrix();
    Eigen::Vector2d trans;
    trans << x, y;
    t.translation() = trans;
    s.setIsometry(t);
    return s;
}