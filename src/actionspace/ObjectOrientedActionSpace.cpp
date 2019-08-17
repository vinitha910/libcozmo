#include "actionspace/ObjectOrientedActionSpace.hpp"
#include <algorithm>
#include <iostream>
#include <iterator>

namespace libcozmo {
namespace actionspace {

bool ObjectOrientedActionSpace::action_similarity(
    const int& action_id1,
    const int& action_id2,
    double* similarity) const
{
    if (!(is_valid_action_id(action_id1) &&
          is_valid_action_id(action_id2))) {
        return false;
    }
    Action* action1 = actions[action_id1];
    Action* action2 = actions[action_id2];
    std::vector<double> action1_vector{
        action1->speed,
        action1->duration,
        action1->start_pos(0),
        action1->start_pos(1),
        action1->theta};
    std::vector<double> action2_vector{
        action2->speed,
        action2->duration,
        action2->start_pos(0),
        action2->start_pos(1),
        action2->theta};
    *similarity = utils::euclidean_distance(action1_vector, action2_vector);
    return true;
}

void ObjectOrientedActionSpace::find_headings(
    const double& theta,
    std::vector<double>* headings) const
{
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
    const double& cube_offset,
    const double& theta,
    Eigen::Vector2d* start_pos) const
{
    start_pos->x() = obj_pos.x() - v_offset * cos(theta) +
                                cube_offset * sin(theta);
    start_pos->y() = obj_pos.y() - v_offset * sin(theta) -
                                cube_offset * cos(theta);
}

void ObjectOrientedActionSpace::generate_actions(
    const Eigen::Vector2d& obj_pos,
    const double& theta,
    const double& h_offset)
{
    std::vector<double> cube_offsets = num_offset == 1 ?
        std::vector<double>{0} :
        utils::linspace(-h_offset, h_offset, num_offset);

    std::vector<double> headings;
    find_headings(theta, &headings);

    int action_id = 0;
    for (size_t i = 0; i < headings.size(); ++i) {
        double heading = headings[i];
        for (const auto& cube_offset : cube_offsets) {
            for (const auto& speed : speeds) {
                for (const auto& duration : durations) {
                    Eigen::Vector2d start_pos;
                    find_start_pos(obj_pos, cube_offset, heading, &start_pos);
                    if (is_valid_action_id(action_id)) {
                        ObjectOrientedActionSpace::Action* action =
                            static_cast<ObjectOrientedActionSpace::Action*>(
                                get_action(action_id));
                        action->update_action(
                            speed,
                            duration,
                            start_pos,
                            heading);
                    } else {
                        actions.push_back(new Action(
                            speed,
                            duration,
                            start_pos,
                            heading));
                    }
                    action_id++;
                }
            }
        }
    }
}

ActionSpace::Action* ObjectOrientedActionSpace::get_action(
    const int& action_id) const
{
    if (!is_valid_action_id(action_id)) {
        throw std::out_of_range("Action ID invalid");
    }
    return actions[action_id];
}

bool ObjectOrientedActionSpace::is_valid_action_id(
    const int& action_id) const
{
    return action_id < actions.size() && action_id >= 0;
}

void ObjectOrientedActionSpace::publish_action(
    const int& action_id,
    const ros::Publisher& publisher) const
{
    if (!is_valid_action_id(action_id)) {
        throw std::out_of_range("Action ID invalid");
    }
    libcozmo::ObjectOrientedAction msg;
    const Action* action = static_cast<Action*>(get_action(action_id));
    msg.speed = action->speed;
    msg.duration = action->duration;
    msg.x = action->start_pos.x();
    msg.y = action->start_pos.y();
    msg.theta = action->theta;

    publisher.publish(msg);
}

int ObjectOrientedActionSpace::size() const {
    return actions.size();
}

} // namespace actionspace
} // namespace libcozmo
