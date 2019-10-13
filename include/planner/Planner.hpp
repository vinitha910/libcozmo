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

#ifndef LIBCOZMO_PLANNER_PLANNER_HPP_
#define LIBCOZMO_PLANNER_PLANNER_HPP_

#include <utility>
#include <unordered_map>

namespace libcozmo {
namespace planner {

/// Maps state ID to it's appropiate cost
typedef std::unordered_map<int, double> CostMap;

/// Maps the child/successor state ID to its respective (parent ID, action ID)
/// pair
typedef std::unordered_map<int, std::pair<
    int, int>> ChildToParentMap;

/// Comparator orders state IDs based on their costs-to-come from the start state
class CostMapComparator {
 public:
    explicit CostMapComparator(const CostMap& cost_map): cost_map_(cost_map) {}

    bool operator()(const int& state_1, const int& state_2) const {
        return cost_map_.find(state_1)->second <=
        cost_map_.find(state_2)->second;
    }

 private:
    const CostMap& cost_map_;
};

/// This class plans a sequence of actions required to get from the start to
/// goal state. The planning method varies based on the derived class.
///
/// While searching, the class utilizes an actionspace to get set of
/// actions to explore, statespace to build a graph representation, and
/// a model to generate successors.
///
/// For efficiency the planner handles all states and actions by their IDs.
class Planner {
 public:
    /// Sets start ID
    ///
    /// \param start_id The starting state ID
    /// \return true if start_id valid; false otherwise
    virtual bool set_start(const int& start_id) = 0;

    /// Sets goal ID
    ///
    /// \param goal_id The goal state ID
    /// \return true if goal_id valid; false otherwise
    virtual bool set_goal(const int& goal_id) = 0;

    /// Finds a sequence of actions from the start to goal state
    ///
    /// \param[out] path Vector of action IDs
    /// \return true if solution is found; false otherwise
    virtual bool solve(std::vector<int>* actions) = 0;
};

}  // namespace planner
}  // namespace libcozmo

#endif
