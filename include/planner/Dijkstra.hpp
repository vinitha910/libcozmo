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

#ifndef LIBCOZMO_PLANNER_DIJKSTRA_HPP_
#define LIBCOZMO_PLANNER_DIJKSTRA_HPP_

#include <ros/ros.h>
#include <utility>
#include <unordered_map>
#include "actionspace/GenericActionSpace.hpp"
#include "statespace/SE2.hpp"
#include "model/DeterministicModel.hpp"
#include "planner/Planner.hpp"
#include "distance/distance.hpp"

namespace libcozmo {
namespace planner {

/// Maps state ID to it's appropiate cost
typedef std::unordered_map<int, double> CostMap;

/// Maps the child/successor state ID to its parent ID and to the action id
/// that is taken to get to the child state from parent state
typedef std::unordered_map<int, std::pair<
    int, int>> ChildToParentMap;

// Comparator orders state IDs based on their costs from start
class CostMapComparator {
 public:
    explicit CostMapComparator(const CostMap& cost_map): cost_map_(cost_map) {}

    bool operator()(const int& state_1,
                    const int& state_2) const {
        return cost_map_.find(state_1)->second <=
        cost_map_.find(state_2)->second;
    }

 private:
    const CostMap& cost_map_;
};

/// This class uses Dijkstra's algorithm to find sequence of actions
/// to get from start state to goal state
///
/// To search for a path the class implements an actionspace, a statespace,
/// and a model
class Dijkstra : public virtual Planner {
 public:
    Dijkstra(
        libcozmo::actionspace::ActionSpace* actionspace,
        libcozmo::statespace::StateSpace* statespace,
        libcozmo::model::Model* model,
        distance::Distance* distance,
        const double& threshold) : \
         m_action_space(actionspace), m_state_space(statespace), m_model(model),
         m_continuous_statespace(std::make_shared<aikido::statespace::SE2>()),
        m_distance_metric(aikido::distance::SE2(m_continuous_statespace)),
        m_se2(distance),
        m_threshold(threshold)  {
        m_start_id = -1;
        m_goal_id = -1;
     }
    ~Dijkstra() {}

    /// Documentation inherited
    bool set_start(const int& start_id) override;

    /// Documentation inherited
    bool set_goal(const int& goal_id) override;

    /// Documentation inherited
    bool solve(std::vector<
        int>* actions) override;

 private:
    /// Extracts sequence of actions from start to goal
    ///
    /// \param child_to_parent_map Maps state ID -> (parent ID, action)
    /// \param start_id Starting The ID of the start state
    /// \param goal_id ID The ID of the goal state
    /// \param[out] actions Vector of action IDs
    void extract_sequence(
        const ChildToParentMap& child_to_parent_map,
        const int& start_id,
        const int& goal_id,
        std::vector<int> *actions);

    /// Finds succestor state given input state and delta x, y, theta
    ///
    /// \param state_ The current state (in continuous space)
    /// \param x Change in x distance (mm)
    /// \param y Change in y distance (mm)
    /// \param theta Change in theta (radians)
    /// \param[out] successor The succesor state
    void get_succ(
        const statespace::SE2::State& state_,
        aikido::statespace::SE2::State* succesor,
        const double& x,
        const double& y,
        const double& theta);

    /// Check whether the solver has reached the goal, which its defintion
    /// varies based on distance metric
    ///
    /// \param curr_state_id The ID of current state
    /// \return True if goal condition is met; false otherwise
    bool is_goal(const int& curr_state_id);

    libcozmo::actionspace::ActionSpace* m_action_space;
    libcozmo::statespace::StateSpace* m_state_space;
    int m_start_id;
    int m_goal_id;
    libcozmo::model::Model* m_model;
    std::shared_ptr<aikido::statespace::SE2> m_continuous_statespace;
    aikido::distance::SE2 m_distance_metric;
    distance::Distance* m_se2;
    const double m_threshold;
};

}  // namespace planner
}  // namespace libcozmo

#endif
