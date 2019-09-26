////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019, Brian Lee, Eric Pan, Vinitha Ranganeni
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

#include "actionspace/ObjectOrientedActionSpace.hpp"
#include "statespace/SE2.hpp"
#include "utils/utils.hpp"

namespace libcozmo {
namespace actionspace {

ObjectOrientedActionSpace::ObjectOrientedActionSpace(
    const std::vector<double>& speeds,
    const std::vector<double>& ratios,
    const Eigen::Vector2d& center_offsets,
    const Eigen::Vector2d& max_edge_offsets,
    const int& num_edge_offsets) : \
    m_speeds(speeds),
    m_ratios(ratios),
    m_center_offsets(center_offsets),
    m_max_edge_offsets(max_edge_offsets) {

    /// Get the edge offsets for the x and y axes
    auto edge_offset_lambda = [&num_edge_offsets](const double max_edge_offset) {
        return num_edge_offsets == 1 ? std::vector<double>{0} :
            utils::linspace(-max_edge_offset, max_edge_offset, num_edge_offsets);
    };
    const std::vector<double> x_edge_offsets = 
        edge_offset_lambda(m_max_edge_offsets.x());
    const std::vector<double> y_edge_offsets = 
        edge_offset_lambda(m_max_edge_offsets.y());

    auto object_side_lambda = [](const double heading_offset) {
        return heading_offset == FRONT || heading_offset == BACK;
    };
    // Generate all possible generic actions given the heading offset, aspect
    // ratio, and speed
    int action_id = 0;
    const std::vector<double> sides{FRONT, LEFT, BACK, RIGHT};
    for (const auto& heading_offset : sides) {
        const double ratio =
            object_side_lambda(heading_offset) ? m_ratios[1] : m_ratios[0];
        const double max_edge_offset =
            object_side_lambda(heading_offset) ? 
                m_max_edge_offsets.x() : m_max_edge_offsets.y();
        const std::vector<double> edge_offsets = 
            object_side_lambda(heading_offset) ? x_edge_offsets : y_edge_offsets;
        const double offset_sign =
            (heading_offset == FRONT || heading_offset == LEFT) ? 1 : -1; 

        for (const auto& edge_offset : edge_offsets) {
            for (const auto& speed : speeds) {
                m_actions.push_back(new GenericAction(
                    speed,
                    offset_sign * edge_offset / max_edge_offset,
                    ratio,
                    heading_offset));
                action_id++;
            }
        }
    }
}

bool ObjectOrientedActionSpace::action_similarity(
    const int& action_id1,
    const int& action_id2,
    double* similarity) const {
    if (!(is_valid_action_id(action_id1) && is_valid_action_id(action_id2))) {
        return false;
    }

    GenericAction* action1 = m_actions[action_id1];
    std::vector<double> action1_vector{
        action1->speed(), action1->edge_offset(), action1->aspect_ratio()};

    GenericAction* action2 = m_actions[action_id2];
    std::vector<double> action2_vector{
        action2->speed(), action2->edge_offset(), action2->aspect_ratio()};
    
    *similarity = utils::euclidean_distance(action1_vector, action2_vector);
    
    return true;
}

ActionSpace::Action* ObjectOrientedActionSpace::get_action(
    const int& action_id) const {
    return is_valid_action_id(action_id) ? m_actions[action_id] : nullptr;
}

bool ObjectOrientedActionSpace::is_valid_action_id(const int& action_id) const {
    return action_id < m_actions.size() && action_id >= 0;
}

bool ObjectOrientedActionSpace::get_generic_to_object_oriented_action(
    const int& action_id,
    const aikido::statespace::StateSpace::State& _state,
    ObjectOrientedAction* action) const {

    const auto generic_action = 
        static_cast<GenericAction*>(get_action(action_id));
    if (generic_action == nullptr) {
        return false;
    }

    // Calculate action w.r.t position of the object
    auto state = static_cast<const aikido::statespace::SE2::State&>(_state);
    Eigen::Isometry2d transform = state.getIsometry();
    Eigen::Rotation2Dd rotation = Eigen::Rotation2Dd::Identity();
    rotation.fromRotationMatrix(transform.rotation());
    const Eigen::Vector2d position = transform.translation();
    const double orientation = rotation.angle();

    const double heading_offset = generic_action->heading_offset();
    const double max_edge_offset =
        (heading_offset == FRONT || heading_offset == BACK) ? 
            m_max_edge_offsets.x() : m_max_edge_offsets.y();
    const double center_offset =
        (heading_offset == FRONT || heading_offset == BACK) ? 
            m_center_offsets.x() : m_center_offsets.y();
    const double offset_sign =
            (heading_offset == FRONT || heading_offset == LEFT) ? 1 : -1; 

    *action = ObjectOrientedAction(
        generic_action->speed(),
        Eigen::Vector3d(
            position.x() - center_offset * cos(orientation) +
                offset_sign * generic_action->edge_offset() * max_edge_offset * sin(orientation),
            position.y() - center_offset * sin(orientation) +
                offset_sign * generic_action->edge_offset() * max_edge_offset * cos(orientation),
            utils::angle_normalization(orientation + heading_offset)));

    return true;
}

bool ObjectOrientedActionSpace::publish_action(
    const int& action_id,
    const ros::Publisher& publisher,
    const aikido::statespace::StateSpace::State& _state) const {
    libcozmo::ObjectOrientedAction msg;
    ObjectOrientedAction OO_action(0.0, Eigen::Vector3d(0, 0, 0));
    if (!get_generic_to_object_oriented_action(action_id, _state, &OO_action)) {
        return false;
    }
    msg.speed = OO_action.speed();
    msg.duration = 1;
    Eigen::Vector3d start_pose = OO_action.start_pose();
    msg.x = start_pose[0];
    msg.y = start_pose[1];
    msg.theta = start_pose[2];
    publisher.publish(msg);
    return true;
}

int ObjectOrientedActionSpace::size() const {
    return m_actions.size();
}

}  // namespace actionspace
}  // namespace libcozmo
