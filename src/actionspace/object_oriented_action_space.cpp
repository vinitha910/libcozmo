#include "actionspace/object_oriented_action_space.hpp"
#include <algorithm>
#include <iostream>
#include <iterator>
using namespace std;

namespace libcozmo {
namespace actionspace {

/**
    Finds the angles, relative to a cartesian plane, to all
    4 sides of the cube given the angle of one of the sides

    The angles are found in clockwise order where index
        0 corresponds to front of cube
        1 corresponds to left of cube
        2 corresponds to back of cube
        3 corresponds to right of cube
    Angles are in radians and are in the range [-pi, pi]
*/
void ObjectOrientedActionSpace::find_sides(vector<double>& cube_sides, const double& theta) const {
    double angle = theta;
    cube_sides.push_back(angle);
    for (size_t i = 0; i < 3; ++i) {
        angle -= M_PI / 2;
        if (angle < -M_PI) {
            angle = 2 * M_PI + angle;
        }
        cube_sides.push_back(angle);
    }
}

/**
    Finds the start position given some other object's position,
    its theta, and the vertical (v_offset) and horizontal (cube_offset)
    distances away from said object
*/
void ObjectOrientedActionSpace::find_start_pos(
    const Eigen::Vector2d& obj_pos,
    Eigen::Vector2d& start_pos,
    const double& v_offset,
    const double& cube_offset,
    const double& theta) const
{
    double x = obj_pos(0);
    double y = obj_pos(1);

    double x_new = x - v_offset * cos(theta) + cube_offset * sin(theta);
    double y_new = y - v_offset * sin(theta) - cube_offset * cos(theta);

    start_pos(x_new, y_new);
}

void ObjectOrientedActionSpace::generate_actions(
    const Eigen::Vector2d& obj_pos,
    const double& theta,
    const double& h_offset,
    const double& v_offset)
{
    vector<double> cube_offsets;
    if (num_offset == 1) { // if only one offset, just use center of side of object
        cube_offsets.push_back(0);
    } else {
        cube_offsets = utils::linspace(-h_offset, h_offset, num_offset);
    }
    vector<double> cube_sides;
    find_sides(cube_sides, theta);

    for (size_t i = 0; i < cube_sides.size(); ++i) {
        double side = cube_sides[i];
        for (const auto& cube_offset : cube_offsets) {
            for (const auto& speed : speeds) {
                for (const auto& duration : durations) {
                    Eigen::Vector2d start_pos;
                    find_start_pos(obj_pos, start_pos, v_offset, cube_offset, theta);
                    actions.push_back(new ObjectOrientedAction(speed, duration, start_pos, theta));
                }
            }
        }
    }
}

ObjectOrientedAction* ObjectOrientedActionSpace::get_action(const int& action_id) const {
    return actions[action_id];
}

void ObjectOrientedActionSpace::view_action_space() const {
    for (size_t i = 0; i < actions.size(); ++i) {
        if (i % 5 == 0) {
            cout << "\n";
        }
        ObjectOrientedAction* a = actions[i];
        cout << i << " : ";
        cout << "Location: " << a->start_pos(0) << ", " << a->start_pos(1) << ", " << a->theta << ",";
        cout << "Speed: " << a->speed << ", ";
        cout << "Duration: " << a->duration << "\n";
    }
}

} // namespace actionspace
} // namespace libcozmo

int main() {
    // Angle_z value should be between -pi and pi
    // libcozmo::actionspace::Pose pose{100, 200, 10, 2};
    // libcozmo::actionspace::ObjectOrientedActionSpace oos {};
    // oos.generate_actions(pose, 3, 10, 100, 3, 1, 5, 3);
    // vector<libcozmo::actionspace::Object_Oriented_Action> oo_actions = oos.get_action_space();
    // oos.view_action_space();
    // cout << "Total actions: " << oo_actions.size() << '\n';

}
