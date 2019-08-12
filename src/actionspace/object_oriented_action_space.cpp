#include "actionspace/object_oriented_action_space.hpp"
#include <algorithm>
#include <iostream>
#include <iterator>
using namespace std;

namespace libcozmo {
namespace actionspace {

double ObjectOrientedActionSpace::action_similarity(
    const int& action_id1,
    const int& action_id2) const
{
    if (!(is_valid_action_id(action_id1) && is_valid_action_id(action_id2))) {
        throw std::out_of_range("Action ID invalid");
    }
    return sqrt(
        pow((actions[action_id1]->speed -
            actions[action_id2]->speed), 2) +
        pow((actions[action_id1]->duration -
            actions[action_id2]->duration), 2) +
        pow((actions[action_id1]->start_pos(0) -
            actions[action_id2]->start_pos(0)), 2) +
        pow((actions[action_id1]->start_pos(1) -
            actions[action_id2]->start_pos(1)), 2) +
        pow((actions[action_id1]->theta -
            actions[action_id2]->theta), 2));
}

void ObjectOrientedActionSpace::clear_actions() {
    for (size_t i = 0; i < actions.size(); ++i) {
        delete(actions[i]);
    }
    actions.clear();
}

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

    Vertical means perpendicular to the edge of an object
    Horizontal means parallel to the edge of an object
*/
void ObjectOrientedActionSpace::find_start_pos(
    const Eigen::Vector2d& obj_pos,
    Eigen::Vector2d& start_pos,
    const double& v_offset,
    const double& cube_offset,
    const double& theta) const
{
    start_pos.x() = obj_pos.x() - v_offset * cos(theta) + cube_offset * sin(theta);
    start_pos.y() = obj_pos.y() - v_offset * sin(theta) - cube_offset * cos(theta);
}

/**
    Generates actions given another object's position and theta

    Parameters
    ----------
    obj_pos is an (x, y) coordinate in millimeters
    theta is an orientation angle in radians
    h_offset (optional) is the max horizontal distance
        from center of the edge of the object
    v_offset (optional) is the max vertical distance
        from center of the object

    Horizontal means parallel to the edge of an object
    Vertical means perpendicular to the edge of an object

    This function will clear any previously generated actions in the action space
*/
void ObjectOrientedActionSpace::generate_actions(
    const Eigen::Vector2d& obj_pos,
    const double& theta,
    const double& h_offset,
    const double& v_offset)
{
    clear_actions();
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
                    find_start_pos(obj_pos, start_pos, v_offset, cube_offset, side);
                    actions.push_back(new ObjectOrientedAction(speed, duration, start_pos, side));
                }
            }
        }
    }
}

ObjectOrientedAction* ObjectOrientedActionSpace::get_action(const int& action_id) const {
    if (!is_valid_action_id(action_id)) {
        throw out_of_range("Action ID invalid");
    }
    return actions[action_id];
}

int ObjectOrientedActionSpace::get_action_space_size() const {
    return actions.size();
}

bool ObjectOrientedActionSpace::is_valid_action_id(const int& action_id) const {
    return action_id < actions.size() && action_id >= 0;
}

void ObjectOrientedActionSpace::view_action_space() const {
    for (size_t i = 0; i < actions.size(); ++i) {
        if (i % 5 == 0) {
            ObjectOrientedAction* a = actions[i];
            cout << i << " : ";
            cout << "Position: (" << a->start_pos(0) << ", " << a->start_pos(1) << "), ";
            cout << "Angle: "<< a->theta << ", ";
            cout << "Speed: " << a->speed << ", ";
            cout << "Duration: " << a->duration << "\n";
        }
    }
}

} // namespace actionspace
} // namespace libcozmo
