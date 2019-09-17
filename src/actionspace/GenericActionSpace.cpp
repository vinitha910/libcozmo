////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019,  Brian Lee, Vinitha Ranganeni
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

#include <actionspace/GenericActionSpace.hpp>
#include <exception>
#include <iostream>

namespace libcozmo {
namespace actionspace {

bool GenericActionSpace::action_similarity(
    const int& action_id1, const int& action_id2, double* similarity) const {
    if (!(is_valid_action_id(action_id1) && is_valid_action_id(action_id2))) {
        return false;
    }

    Action* action1 = m_actions[action_id1];
    Action* action2 = m_actions[action_id2];
    std::vector<double> action1_vector{
        action1->m_speed,
        action1->m_duration,
        action1->m_heading};
    std::vector<double> action2_vector{
        action2->m_speed,
        action2->m_duration,
        action2->m_heading};
    *similarity = utils::euclidean_distance(action1_vector, action2_vector);
    return true;
}

int GenericActionSpace::size() const {
    return m_actions.size();
}

ActionSpace::Action* GenericActionSpace::get_action(const int& action_id) const {
    if (!is_valid_action_id(action_id)) {
        return nullptr;
    }
    return m_actions[action_id];
}

bool GenericActionSpace::is_valid_action_id(const int& action_id) const {
    return ((action_id < m_actions.size() && action_id >= 0));
}

bool GenericActionSpace::publish_action(
    const int& action_id, const ros::Publisher& publisher) const {
    libcozmo::ActionMsg msg;
    Action* action = static_cast<Action*>(get_action(action_id));
    if (action == nullptr) {
        return false;
    }
    msg.speed = action->m_speed;
    msg.heading = action->m_heading;
    msg.duration = action->m_duration;
    publisher.publish(msg);
    return true;
}
}  // namespace actionspace
}  // namespace libcozmo
