////////////////////////////////////////////////////////////////////////////////
//// Copyright (c) 2019, Eric Pan, Vinitha Ranganeni
//// All rights reserved.
////
//// Redistribution and use in source and binary forms, with or without
//// modification, are permitted provided that the following conditions are met:
////
////     1. Redistributions of source code must retain the above copyright notice
////        this list of conditions and the following disclaimer.
////     2. Redistributions in binary form must reproduce the above copyright
////        notice, this list of conditions and the following disclaimer in the
////        documentation and/or other materials provided with the distribution.
////     3. Neither the name of the copyright holder nor the names of its
////        contributors may be used to endorse or promote products derived from
////        this software without specific prior written permission.
////
//// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

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
        action1->start_pose(0),
        action1->start_pose(1),
        action1->start_pose(2)};
    std::vector<double> action2_vector{
        action2->speed,
        action2->duration,
        action2->start_pose(0),
        action2->start_pose(1),
        action2->start_pose(2)};
    *similarity = utils::euclidean_distance(action1_vector, action2_vector);
    return true;
}

void ObjectOrientedActionSpace::find_headings(
    const double& theta_rad,
    std::vector<double>* headings) const
{
    double normalized_theta_rad = theta_rad;
    // Force angle to be between [0, 2pi]
    if (abs(theta_rad) > 2.0 * M_PI) {
        normalized_theta_rad = normalized_theta_rad -
            static_cast<int>(normalized_theta_rad / (2.0 * M_PI)) * 2.0 * M_PI;
    }
    if (theta_rad < 0) {
        normalized_theta_rad += 2.0 * M_PI;
    }
    headings->push_back(normalized_theta_rad);
    for (size_t i = 0; i < 3; ++i) {
        normalized_theta_rad -= M_PI / 2;
        if (normalized_theta_rad < 0) {
            normalized_theta_rad = 2 * M_PI + normalized_theta_rad;
        }
        headings->push_back(normalized_theta_rad);
    }
}

void ObjectOrientedActionSpace::find_start_pos(
    const Eigen::Vector3d& obj_pose,
    const double& edge_offset,
    const double& heading,
    Eigen::Vector2d* start_pos) const
{
    start_pos->x() = obj_pose.x() - center_offset * cos(heading) +
                     edge_offset * sin(heading);
    start_pos->y() = obj_pose.y() - center_offset * sin(heading) -
                     edge_offset * cos(heading);
}

void ObjectOrientedActionSpace::generate_actions(
    const Eigen::Vector3d& obj_pose,
    const double& edge_offset)
{
    std::vector<double> cube_offsets = num_offset == 1 ?
        std::vector<double>{0} :
        utils::linspace(-edge_offset, edge_offset, num_offset);

    std::vector<double> headings;
    find_headings(obj_pose(2), &headings);

    int action_id = 0;
    for (const auto& heading : headings) {
        for (const auto& cube_offset : cube_offsets) {
            for (const auto& speed : speeds) {
                for (const auto& duration : durations) {
                    Eigen::Vector2d start_pos;
                    find_start_pos(obj_pose, cube_offset, heading, &start_pos);

                    ObjectOrientedActionSpace::Action* action =
                        static_cast<ObjectOrientedActionSpace::Action*>(
                            get_action(action_id));
                    if (action == nullptr) {
                        actions.push_back(new Action(
                            speed,
                            duration,
                            Eigen::Vector3d(
                                start_pos(0),
                                start_pos(1),
                                heading),
                            -cube_offset / edge_offset
                            ));
                    } else {
                        action->update_action(
                            speed,
                            duration,
                            Eigen::Vector3d(
                                start_pos(0),
                                start_pos(1),
                                heading),
                            -cube_offset / edge_offset
                            );
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
        return nullptr;
    }
    return actions[action_id];
}

bool ObjectOrientedActionSpace::is_valid_action_id(
    const int& action_id) const
{
    return action_id < actions.size() && action_id >= 0;
}

bool ObjectOrientedActionSpace::publish_action(
    const int& action_id,
    const ros::Publisher& publisher) const
{
    libcozmo::ObjectOrientedAction msg;
    const Action* action = static_cast<Action*>(get_action(action_id));
    if (action == nullptr) {
        return false;
    }
    msg.speed = action->speed;
    msg.duration = action->duration;
    msg.x = action->start_pose(0);
    msg.y = action->start_pose(1);
    msg.theta = action->start_pose(2);

    publisher.publish(msg);
    return true;
}

int ObjectOrientedActionSpace::size() const {
    return actions.size();
}

void ObjectOrientedActionSpace::view_action_space() const {
    for (size_t i = 0; i < actions.size(); ++i) {
        if (i % 5 == 0) {
            std::cout << "\n";
        }
    	const Action* a = static_cast<Action*>(get_action(i));
        std::cout << i << " : ";
        std::cout << "Position: (" << a->start_pose(0) << ", " << a->start_pose(1) << "), ";
        std::cout << "Angle: "<< a->start_pose(2) << ", ";
        std::cout << "Speed: " << a->speed << ", ";
        std::cout << "Duration: " << a->duration << "\n";
        std::cout << "Offset: " << a->edge_offset << "\n";
    }
}

} // namespace actionspace
} // namespace libcozmo
