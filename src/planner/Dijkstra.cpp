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

#include "planner/Dijkstra.hpp"
#include <iostream>

namespace libcozmo {
namespace planner {

    bool Dijkstra::set_start(const int& start_id) {
        if (m_state_space->is_valid_state(
                *m_state_space->get_state(start_id))) {
            m_start_id = start_id;
            return true;
        }
        return false;
    }

    bool Dijkstra::set_goal(const int& goal_id) {
        if (m_state_space->is_valid_state(
                *m_state_space->get_state(goal_id))) {
            m_goal_id = goal_id;
            return true;
        }
        return false;
    }

    bool Dijkstra::solve(std::vector<int>* actions) {
        if (m_goal_id == -1 || m_start_id == -1) {
            return false;
        }
        CostMap costmap;
        CostMapComparator comparator(costmap);
        std::set<int, CostMapComparator> Q(comparator);
        std::vector<int> path_ids;
        Q.insert(m_start_id);
        costmap[m_start_id] = 0;
        while (!Q.empty()) {
            int curr_state_id = *(Q.begin());
            Q.erase(Q.begin());
            Q.erase(curr_state_id);
            if (is_goal(curr_state_id)) {
                extract_sequence(actions);
                return true;
            }
            const statespace::StateSpace::State* curr_state =
                m_state_space->get_state(curr_state_id);
            const auto curr_state_ =
                static_cast<const statespace::SE2::State*>(curr_state);
            std::vector<statespace::SE2::State> succesor_states;
            get_successors(curr_state_, &succesor_states);
            for (int i = 0; i < succesor_states.size(); i++) {
                auto succesor_state = succesor_states[i];
                if (m_state_space->is_valid_state(succesor_state)) {
                    const int succesor_id =
                        m_state_space->get_or_create_state(succesor_state);
                    const double new_cost = costmap[curr_state_id] +
                        m_state_space->get_distance(
                            *curr_state_,
                            succesor_state);
                    if (costmap.find(succesor_id) == costmap.end() ||
                        costmap[succesor_id] > new_cost) {
                        m_child_to_parent_map[succesor_id] =
                            std::make_pair(curr_state_id, i);
                        costmap[succesor_id] = new_cost;
                        assert(Q.find(curr_state_id) == Q.end());
                        Q.erase(succesor_id);
                        Q.insert(succesor_id);
                    }
                }
            }
        }
        return false;
    }

    bool Dijkstra::is_goal(const int& curr_state_id) const {
        return curr_state_id  == m_goal_id || m_distance_metric->get_distance(
            *m_state_space->get_state(curr_state_id),
            *m_state_space->get_state(m_goal_id)) <= m_goal_tolerance;
    }

    void Dijkstra::get_successors(
        const statespace::SE2::State* curr_state,
        std::vector<statespace::SE2::State>* succesors) {
        for (int i = 0; i < m_action_space->size(); i++) {
            const actionspace::ActionSpace::Action* action =
                m_action_space->get_action(i);
            const auto action_ = static_cast<
                const actionspace::GenericActionSpace::Action*>(action);
            const model::DeterministicModel::DeterministicModelInput input(
                *action_);
            model::DeterministicModel::DeterministicModelOutput output;
            m_model->get_prediction(input, &output);
            aikido::statespace::SE2::State succesor;
            m_state_space->discrete_state_to_continuous(*curr_state, &succesor);
            auto curr_state_isometry = succesor.getIsometry();
            const double x = curr_state_isometry.translation()[0]
                + output.getX();
            const double y = curr_state_isometry.translation()[1]
                + output.getY();
            Eigen::Isometry2d t = Eigen::Isometry2d::Identity();
            const Eigen::Rotation2D<double> rot(output.getTheta());
            t.linear() = rot.toRotationMatrix();
            t.translation() = Eigen::Vector2d(x, y);
            succesor.setIsometry(t);
            statespace::SE2::State succesor_state_;
            m_state_space->continuous_state_to_discrete(
                succesor, &succesor_state_);
            succesors->push_back(succesor_state_);
        }
        std::reverse(succesors->begin(), succesors->end());
    }

    void Dijkstra::extract_sequence(std::vector<int>* actions) {
        if (m_goal_id == m_start_id) {
            return;
        }
        auto parent = m_child_to_parent_map.find(m_goal_id);
        while (parent != m_child_to_parent_map.end()) {
            actions->push_back(parent->second.second);
            parent = m_child_to_parent_map.find(parent->second.first);
        }
        std::reverse(actions->begin(), actions->end());
    }

}  // namespace planner
}  // namespace libcozmo
