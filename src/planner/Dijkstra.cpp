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
#include <math.h>
#include <ros/ros.h>

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

    bool Dijkstra::solve(std::vector<int>* path) {
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
                m_goal_id = curr_state_id;
                extract_path(path);
                return true;
            }
            const statespace::StateSpace::State* curr_state =
                m_state_space->get_state(curr_state_id);
            
            const auto s = static_cast<const statespace::SE2::State*>(curr_state);
            // std::cout << "discrete" << std::endl;
            // std::cout << s->X() << " " << s->Y() << " " << s->Theta() << " " << s->Time() << std::endl;
            // Find successor states to the current state
            std::vector<int> succesor_states;
            get_successors(*curr_state, &succesor_states);
            for (int i = 0; i < succesor_states.size(); i++) {
                const int successor_id = succesor_states[i];
                const auto succesor_state = m_state_space->get_state(successor_id);
                const double new_cost = costmap[curr_state_id] +
                    m_state_space->get_distance(
                        *curr_state,
                        *succesor_state);

                // Update the cost map if a successor state has not been
                // explored before, or if it has lower cost than previously
                // explored path
                if (costmap.find(successor_id) == costmap.end() ||
                    costmap[successor_id] > new_cost) {

                    // Add (parent ID, action ID) pair to child to parent
                    // map if the explored succsor is being added
                    m_child_to_parent_map[successor_id] = curr_state_id;
                    costmap[successor_id] = new_cost;
                    assert(priority_queue.find(curr_state_id) ==
                        priority_queue.end());
                    priority_queue.erase(successor_id);
                    priority_queue.insert(successor_id);
                }
            }
        }
        // If all possible states are explored with no solution, return false
        return false;
    }

    bool Dijkstra::is_goal(const int& curr_state_id) const {
        const double dist = m_state_space->get_distance(
            *m_state_space->get_state(curr_state_id),
            *m_state_space->get_state(m_goal_id));
        // ROS_INFO("%d", dist);
        // getchar();
        return curr_state_id  == m_goal_id || dist <= m_goal_tolerance;
    }

    void Dijkstra::get_successors(
        const statespace::StateSpace::State& state_,
        std::vector<int>* succesors) {
        for (int i = 0; i < m_action_space->size(); i++) {
            const actionspace::ActionSpace::Action* action_ =
                m_action_space->get_action(i);
            const auto action = 
                static_cast<const actionspace::GenericActionSpace::Action*>(action_);
            
            Eigen::Vector4d state;
            m_state_space->discrete_state_to_continuous(state_, &state);

            // If the speed == 0 and action and current headings are different
            // then rotate in place
            if (action->m_speed == 0.0 && state[2] != action->m_heading) {
                 state[2] = action->m_heading;   
            }

            // If the action's heading and the current heading are not equal,
            // the action cannot be applied
            else if (action->m_speed != 0.0 && state[2] != action->m_heading) {
                continue;
            }
 
            // The headings match and there is a speed > 0
            else {
                const double distance = action->m_speed * action->m_duration;
                const double dx = distance * cos(action->m_heading);
                const double dy = distance * sin(action->m_heading);
                state[0] = state[0] + dx;
                state[1] = state[1] + dy;
            }

            state[3] = state[3] + action->m_duration;

            // std::cout << state[0] << " " << state[1] << " " << state[2] << " " << state[3] << std::endl;
            // getchar();

            Eigen::Vector4d discrete_state;
            m_state_space->continuous_state_to_discrete(state, &discrete_state);
            if (discrete_state.head<3>() == state_.vector().head<3>()) {
                continue;
            }
            // std::cout << "not same" << std::endl;
            succesors->push_back(m_state_space->get_or_create_state(state, true));
        }
    }

    void Dijkstra::extract_path(std::vector<int>* path_state_ids) {
        if (m_goal_id == m_start_id) {
            return;
        }
        path_state_ids->push_back(m_goal_id);
        int current_state_id = m_goal_id;
        while (current_state_id != m_start_id) {
            const auto parent = m_child_to_parent_map.find(current_state_id);
            if (parent == m_child_to_parent_map.end()) {
                break;
            }
            current_state_id = parent->second;
            path_state_ids->push_back(current_state_id);
        }
        std::reverse(path_state_ids->begin(), path_state_ids->end());
    }

}  // namespace planner
}  // namespace libcozmo
