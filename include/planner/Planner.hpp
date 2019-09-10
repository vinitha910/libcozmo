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

namespace libcozmo {
namespace planner {


/// Plans sequence of actions required to reach from start to goal
/// A strategy of finding such path should be determined as a derived class
/// e.g. Dijkstra
///
/// Actionspace handles possible set of actions at given time
/// Statespace handles states present in the representation
/// Transition model outputs successor state given a state and an action.
class Planner
{
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

    /// Solves given search problem and finds path from start to goal
    ///
    /// \param path Vector of ID pairs
    /// \return true if path is found; false otherwise
    virtual bool solve(
        std::vector<int>* actions) = 0;
};

}  // namespace planner
}  // namespace libcozmo

#endif
