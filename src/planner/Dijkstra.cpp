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
// CONSEpriority_queueUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
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
        auto state_ = m_state_space->get_state(start_id);
        if (state_ == nullptr) {
            return false;
        }

        if (m_state_space->is_valid_state(*state_)) {
            m_start_id = start_id;
            return true;
        }
        return false;
    }

    bool Dijkstra::set_goal(const int& goal_id) {
        auto state_ = m_state_space->get_state(goal_id);
        if (state_ == nullptr) {
            return false;
        }
        if (m_state_space->is_valid_state(*state_)) {
            m_goal_id = goal_id;
            return true;
        }
        return false;
    }

    bool Dijkstra::solve(std::vector<int>* actions) {
        // Check that goal and start ID have been set
        if (m_goal_id == -1 || m_start_id == -1) {
            return false;
        }

        // Initializing the priority priority_queueueue and cost map
        CostMap costmap;
        CostMapComparator comparator(costmap);
        std::set<int, CostMapComparator> priority_queue(comparator);
        std::vector<int> path_ids;
        priority_queue.insert(m_start_id);
        costmap[m_start_id] = 0;

        // While the priority queue is not empty, search for a path from the
        // start state to the goal state
        while (!priority_queue.empty()) {
            int curr_state_id = *(priority_queue.begin());
            priority_queue.erase(priority_queue.begin());
            priority_queue.erase(curr_state_id);

            // Check if the goal condition has been met
            if (is_goal(curr_state_id)) {
                extract_action_sequence(actions);
                return true;
            }
            const statespace::StateSpace::State* curr_state =
                m_state_space->get_state(curr_state_id);
            
            // Find successor states to the current state
            std::vector<int> succesor_states;
            get_successors(*curr_state, &succesor_states);
            for (int i = 0; i < succesor_states.size(); i++) {
                // auto successor_state = stasspace.get_state(id);
                auto succesor_state = succesor_states[i];
                if (m_state_space->is_valid_state(*succesor_state)) {
                    const int succesor_id =
                        m_state_space->get_or_create_state(*succesor_state);
                    const double new_cost = costmap[curr_state_id] +
                        m_state_space->get_distance(
                            *curr_state,
                            *succesor_state);

                    // Update the cost map if a successor state has not been
                    // explored before, or if it has lower cost than previously
                    // explored path
                    if (costmap.find(succesor_id) == costmap.end() ||
                        costmap[succesor_id] > new_cost) {

                        // Add (parent ID, action ID) pair to child to parent
                        // map if the explored succsor is being added
                        m_child_to_parent_map[succesor_id] =
                            std::make_pair(curr_state_id, i);
                        costmap[succesor_id] = new_cost;
                        assert(priority_queue.find(curr_state_id) ==
                            priority_queue.end());
                        priority_queue.erase(succesor_id);
                        priority_queue.insert(succesor_id);
                    }
                }
            }
        }
        // If all possible states are explored with no solution, return false
        return false;
    }

    bool Dijkstra::is_goal(const int& curr_state_id) const {
        return curr_state_id  == m_goal_id || m_distance_metric->get_distance(
            *m_state_space->get_state(curr_state_id),
            *m_state_space->get_state(m_goal_id)) <= m_goal_tolerance;
    }

    void Dijkstra::get_successors(
        const statespace::StateSpace::State& state_,
        std::vector<int>* succesors) {
        for (int i = 0; i < m_action_space->size(); i++) {
            const actionspace::ActionSpace::Action* action =
                m_action_space->get_action(i);
            // Given model input, get successor state in continuous space
            Eigen::VectorXd state_vector = state_.vector();
            Eigen::VectorXd output(state_vector.size());
            m_model->predict_state(state_.vector(), action->vector(), &output);
            
            /*
            int successor_id = get_or create .. (output);
            if (id valid) {push back to vector}
            
            */


            // Append discretized successor state to vector of valid successors
            statespace::StateSpace::State& succesor_state_ = state_;
            succesor_state_.from_vector(output);
            succesors->push_back(&succesor_state_);
        }
        // Reverse the vector order as successors are appened in backwards order
        std::reverse(succesors->begin(), succesors->end());
    }

    void Dijkstra::extract_action_sequence(std::vector<int>* actions) {
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
