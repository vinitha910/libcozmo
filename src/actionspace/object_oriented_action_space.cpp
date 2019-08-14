#include "actionspace/object_oriented_action_space.hpp"
#include <algorithm>
#include <iostream>
#include <iterator>

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

void ObjectOrientedActionSpace::find_headings(std::vector<double>* headings, const double& theta) const {
    double angle = theta;
    // Force angle to be between [0, 2pi]
    if (angle > 2 * M_PI || angle < 0) {
        int cycles = angle / (2 * M_PI);
        angle -= 2 * M_PI * cycles;
        if (angle < 0) {
            angle += 2 * M_PI;
        }
    }
    headings->push_back(angle);
    for (size_t i = 0; i < 3; ++i) {
        angle -= M_PI / 2;
        if (angle < 0) {
            angle = 2 * M_PI + angle;
        }
        headings->push_back(angle);
    }
}

void ObjectOrientedActionSpace::find_start_pos(
    const Eigen::Vector2d& obj_pos,
    Eigen::Vector2d* start_pos,
    const double& v_offset,
    const double& cube_offset,
    const double& theta) const
{
    start_pos->x() = obj_pos.x() - v_offset * cos(theta) + cube_offset * sin(theta);
    start_pos->y() = obj_pos.y() - v_offset * sin(theta) - cube_offset * cos(theta);
}

void ObjectOrientedActionSpace::generate_actions(
    const Eigen::Vector2d& obj_pos,
    const double& theta,
    const double& h_offset,
    const double& v_offset)
{
    clear_actions();
    std::vector<double> cube_offsets;
    if (num_offset == 1) { // if only one offset, just use center of side of object
        cube_offsets.push_back(0);
    } else {
        cube_offsets = utils::linspace(-h_offset, h_offset, num_offset);
    }
    std::vector<double> headings;
    find_headings(&headings, theta);

    for (size_t i = 0; i < headings.size(); ++i) {
        double heading = headings[i];
        for (const auto& cube_offset : cube_offsets) {
            for (const auto& speed : speeds) {
                for (const auto& duration : durations) {
                    Eigen::Vector2d start_pos;
                    find_start_pos(obj_pos, &start_pos, v_offset, cube_offset, heading);
                    actions.push_back(new ObjectOrientedAction(speed, duration, start_pos, heading));
                }
            }
        }
    }
}

ObjectOrientedActionSpace::ObjectOrientedAction* ObjectOrientedActionSpace::get_action(const int& action_id) const {
    if (!is_valid_action_id(action_id)) {
        throw std::out_of_range("Action ID invalid");
    }
    return actions[action_id];
}

int ObjectOrientedActionSpace::size() const {
    return actions.size();
}

bool ObjectOrientedActionSpace::is_valid_action_id(const int& action_id) const {
    return action_id < actions.size() && action_id >= 0;
}

void ObjectOrientedActionSpace::view_action_space() const {
    for (size_t i = 0; i < actions.size(); ++i) {
        if (i % 5 == 0) {
            ObjectOrientedAction* a = actions[i];
            std::cout << i << " : ";
            std::cout << "Position: (" << a->start_pos(0) << ", " << a->start_pos(1) << "), ";
            std::cout << "Angle: "<< a->theta << ", ";
            std::cout << "Speed: " << a->speed << ", ";
            std::cout << "Duration: " << a->duration << "\n";
        }
    }
}

} // namespace actionspace
} // namespace libcozmo
