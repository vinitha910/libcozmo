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
        ChildToParentMap child_to_parent_map;
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
                extract_sequence(
                    child_to_parent_map, m_start_id, m_goal_id, actions);
                return true;
            }
            const statespace::StateSpace::State* curr_state =
                m_state_space->get_state(curr_state_id);
            for (int i = 0; i < m_action_space->size(); i++) {
                const actionspace::ActionSpace::Action* action =
                    m_action_space->get_action(i);
                const auto action_ = static_cast<
                    const actionspace::GenericActionSpace::Action*>(action);
                const auto curr_state_ =
                    static_cast<const statespace::SE2::State*>(curr_state);
                const model::DeterministicModel::ModelInput input(*action_);
                model::DeterministicModel::ModelOutput* output =
                    static_cast<model::DeterministicModel::ModelOutput*>(
                        m_model->get_prediction(input));
                aikido::statespace::SE2::State succesor_state;
                get_succ(
                    *curr_state_,
                    &succesor_state,
                    output->getX(),
                    output->getY(),
                    output->getTheta());
                statespace::SE2::State succesor_state_;
                m_state_space->continuous_state_to_discrete(
                    succesor_state, &succesor_state_);
                if (m_state_space->is_valid_state(succesor_state_)) {
                    const int succesor_id =
                        m_state_space->get_or_create_state(succesor_state_);
                    const double new_cost = costmap[curr_state_id] +
                        m_state_space->get_distance(
                            *curr_state_,
                            succesor_state_);
                    if (costmap.find(succesor_id) == costmap.end() ||
                        costmap[succesor_id] > new_cost) {
                        child_to_parent_map[succesor_id] =
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

    bool Dijkstra::is_goal(const int& curr_state_id) {
        return curr_state_id  == m_goal_id || m_se2->get_distance(
            *m_state_space->get_state(curr_state_id),
            *m_state_space->get_state(m_goal_id)) <= m_threshold;
    }

    void Dijkstra::get_succ(
        const statespace::SE2::State& state_,
        aikido::statespace::SE2::State* succesor,
        const double& x,
        const double& y,
        const double& theta) {
        m_state_space->discrete_state_to_continuous(state_, succesor);
        auto curr_state_isometry = succesor->getIsometry();
        double x_ = curr_state_isometry.translation()[0] + x;
        double y_ = curr_state_isometry.translation()[1] + y;
        Eigen::Isometry2d t = Eigen::Isometry2d::Identity();
        const Eigen::Rotation2D<double> rot(theta);
        t.linear() = rot.toRotationMatrix();
        t.translation() = Eigen::Vector2d(x_, y_);
        succesor->setIsometry(t);
    }

    void Dijkstra::extract_sequence(
        const ChildToParentMap& child_to_parent_map,
        const int& start_id,
        const int& goal_id,
        std::vector<int>* actions) {
        if (goal_id == start_id) {
            return;
        }
        auto parent = child_to_parent_map.find(goal_id);
        while (parent != child_to_parent_map.end()) {
            actions->push_back(parent->second.second);
            parent = child_to_parent_map.find(parent->second.first);
        }
        std::reverse(actions->begin(), actions->end());
    }

}  // namespace planner
}  // namespace libcozmo
