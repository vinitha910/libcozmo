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

#include <gtest/gtest.h>
#include "planner/Dijkstra.hpp"
#include "distance/SE2.hpp"

namespace libcozmo {
namespace planner {
namespace test {

TEST(DijkstraTest, StartIsGoalTest) {
    /// Checking solver's base case where start is the same as goal
    auto type_ = libcozmo::model::DeterministicModel::DeterministicModelType();
    libcozmo::statespace::SE2 state_space(10, 4);
    int start = state_space.get_or_create_state(
        libcozmo::statespace::SE2::State(0, 0, 0));
    libcozmo::actionspace::GenericActionSpace action_space(
        std::vector<double>{0.1},
        std::vector<double>{0.1},
        4);
    libcozmo::model::DeterministicModel model;
    auto se2 = distance::SE2(std::make_shared<statespace::SE2>(0.1, 4));
    model.load_model(&type_);
    libcozmo::planner::Dijkstra m_solver(
        &action_space, &state_space, &model, &se2, 0.01);
    std::vector<int> actions;
    EXPECT_TRUE(m_solver.set_start(start));
    EXPECT_TRUE(m_solver.set_goal(start));
    bool solved  = m_solver.solve(&actions);
    EXPECT_TRUE(solved);
}

TEST(DijkstraTest, SimpleSolverTestCozmo) {
    /// Checking solver in a medium-sized problem
    auto type_ = libcozmo::model::DeterministicModel::DeterministicModelType();
    libcozmo::statespace::SE2 state_space(10, 4);
    int start = state_space.get_or_create_state(
        libcozmo::statespace::SE2::State(0, 0, 0));
    int goal = state_space.get_or_create_state(
        libcozmo::statespace::SE2::State(35, 30, 3));
    libcozmo::actionspace::GenericActionSpace action_space(
        std::vector<double>{0, 150, 200, 250},
        std::vector<double>{1},
        4);
    libcozmo::model::DeterministicModel model;
    model.load_model(&type_);
    auto distance_metric =
        distance::SE2(std::make_shared<statespace::SE2>(10, 4));
    libcozmo::planner::Dijkstra m_solver(
        &action_space, &state_space, &model, &distance_metric, 1);
    std::vector<int> actions;
    EXPECT_TRUE(m_solver.set_start(start));
    EXPECT_TRUE(m_solver.set_goal(goal));
    bool solved  = m_solver.solve(&actions);
    EXPECT_TRUE(solved);
}

}  // namespace test
}  // namespace planner
}  // namespace libcozmo

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    return result;
}
