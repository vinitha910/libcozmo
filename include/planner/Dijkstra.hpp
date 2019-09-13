//////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019, Brian Lee, Vinitha Ranganeni
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
#include "actionspace/ActionSpace.hpp"
#include "statespace/StateSpace.hpp"
#include "model/Model.hpp"
#include "planner/Planner.hpp"
#include "model/DeterministicModel.hpp"
#include "statespace/SE2.hpp"
#include "actionspace/generic_action_space.hpp"
#include "distance/SE2.hpp"

namespace libcozmo {
namespace planner {

/// Maps state ID to it's appropiate cost
typedef std::unordered_map<int, double> CostMap;

/// Maps the child/successor state ID to the parent ID and the action id
/// that is taken to get to the child state from parent state
typedef std::unordered_map<int, std::pair<
    int, int>> ChildToParentMap;

// Comparator used to order the states based on their costs
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

/// Uses Dijkstra's algorithm to find optimial sequence of actions
/// to get from specified start state to goal state
///
/// Actionspace handles possible set of actions at given time
/// Statespace handles states present in the representation
/// Transition model output successor state given input state and action.
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

    /// Document inherited
    bool set_start(const int& start_id) override;

    /// Document inherited
    bool set_goal(const int& goal_id) override;

    /// Document inherited
    bool solve(std::vector<
        int>* actions) override;

 private:

    /// Extracts sequence of actions from start to goal
    ///
    /// \param child_to_parent_map Maps state ID -> (parent ID, action)
    /// \param start_id Starting ID
    /// \param goal_id ID Goal ID
    /// \param[out] path_actions Vector of actions
    void extract_path(
        const ChildToParentMap& child_to_parent_map,
        const int& start_id,
        const int& goal_id,
        std::vector<int> *path_actions);

    /// Finds succestor state in continuous space
    ///
    /// \param state_ Current state
    /// \param successor The succesor state
    /// \param x Delta x
    /// \param y Delta y
    /// \param theta Delta theta
    void get_succ(
        const statespace::SE2::State& state_,
        aikido::statespace::SE2::State* succesor,
        const double& x,
        const double& y,
        const double& theta);

    bool is_goal(int curr_state_id);

    /// Action Space
    libcozmo::actionspace::ActionSpace* m_action_space;
    /// State Space
    libcozmo::statespace::StateSpace* m_state_space;
    /// Starting ID
    int m_start_id;
    /// Goal ID
    int m_goal_id;
    /// Transition model
    libcozmo::model::Model* m_model;
    std::shared_ptr<aikido::statespace::SE2> m_continuous_statespace;
    aikido::distance::SE2 m_distance_metric;
    distance::Distance* m_se2;
    const double m_threshold;
};

}  // namespace planner
}  // namespace libcozmo

#endif
