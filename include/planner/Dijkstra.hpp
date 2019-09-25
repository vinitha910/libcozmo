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

#include <memory>
#include "actionspace/GenericActionSpace.hpp"
#include "statespace/SE2.hpp"
#include "model/DeterministicModel.hpp"
#include "planner/Planner.hpp"
#include "distance/distance.hpp"

namespace libcozmo {
namespace planner {

/// This class uses Dijkstra's algorithm to find a sequence of actions
/// to get from start state to goal state
class Dijkstra : public virtual Planner {
 public:
    /// Constructs a planner with given actionspace, statespace, model
    ///
    /// Constructs distance metric and goal tolerance
    ///
    /// \param actionspace Defines the set of all possible actions
    /// \param statespace Defines the set of all possible states
    /// \param model A model that has learnt f : (state, action) -> state'
    /// \param distance_metric The metric used to calculate the distance between
    ///     two states
    /// \param goal_tolerance The threshold for being considered at the goal
    Dijkstra(
        std::shared_ptr<actionspace::ActionSpace> actionspace,
        std::shared_ptr<statespace::StateSpace> statespace,
        std::shared_ptr<model::Model> model,
        std::shared_ptr<distance::Distance> distance_metric,
        const double& goal_tolerance) : \
        m_action_space(actionspace),
        m_state_space(statespace),
        m_model(model),
        m_distance_metric(distance_metric),
        m_goal_tolerance(goal_tolerance)  {
        m_start_id = -1;
        m_goal_id = -1;
     }
    ~Dijkstra() {}

    /// Documentation inherited
    bool set_start(const int& start_id) override;

    /// Documentation inherited
    bool set_goal(const int& goal_id) override;

    /// Documentation inherited
    bool solve(std::vector<int>* actions) override;

 private:
    /// Extracts sequence of actions from start to goal
    ///
    /// \param[out] actions Vector of action IDs
    void extract_action_sequence(std::vector<int> *actions);

    /// Finds all successor states given input state
    ///
    /// \param state_ The current state (in continuous space)
    /// \param[out] successors Vector of succesor states
    void get_successors(
        const statespace::SE2::State* state_,
        std::vector<statespace::SE2::State>* succesors);

    /// Check whether the solver has reached the goal (i.e. is within the
    /// specified goal tolerance)
    ///
    /// \param curr_state_id The ID of current state
    /// \return True if goal condition is met; false otherwise
    bool is_goal(const int& curr_state_id) const;

    std::shared_ptr<actionspace::ActionSpace> m_action_space;
    std::shared_ptr<statespace::StateSpace> m_state_space;
    int m_start_id;
    int m_goal_id;
    std::shared_ptr<model::Model> m_model;
    std::shared_ptr<distance::Distance> m_distance_metric;
    const double m_goal_tolerance;
    ChildToParentMap m_child_to_parent_map;
};

}  // namespace planner
}  // namespace libcozmo

#endif
