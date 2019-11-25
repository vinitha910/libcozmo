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

class DijkstraTest: public ::testing::Test {
 public:
    DijkstraTest() : m_solver(create_planner(&m_start, &m_goal)) {}

    ~DijkstraTest() {}

    /// Construct a dijkstra planner
    ///
    /// \param[out] start The ID of start state
    /// \param[out] goal The ID of goal state
    Dijkstra create_planner(int* start, int* goal) {
        std::shared_ptr<actionspace::GenericActionSpace> GAS_ptr =
            std::make_shared<actionspace::GenericActionSpace>(
                std::vector<double>{0, 150, 200, 250},
                std::vector<double>{1},
                4); 
        
        // Create statespace and add start/goal
        std::shared_ptr<statespace::SE2> SE2_ptr =
            std::make_shared<statespace::SE2>(10, 4);
        *start = SE2_ptr->get_or_create_state(
            libcozmo::statespace::SE2::State(0, 0, 0));
        *goal = SE2_ptr->get_or_create_state(
            libcozmo::statespace::SE2::State(35, 30, 3));

        // Get shared pointer of model, actionspace, and distance metric for
        // construction

        const std::shared_ptr<model::GPRModel> model_ptr =
            std::make_shared<model::GPRModel>(std::make_shared<ScikitLearnFramework>("/home/joonh/cozmo_ws/src/libcozmo/tests/model/SampleGPRModel.pkl"));
        const std::shared_ptr<distance::SE2> distance_ptr =
            std::make_shared<distance::SE2>(SE2_ptr);

        // Construct and return instance of a solver
        libcozmo::planner::Dijkstra m_solver(
            GAS_ptr,
            SE2_ptr,
            model_ptr,
            distance_ptr,
            1.0);

        return m_solver;    
    }

    libcozmo::planner::Dijkstra m_solver;
    int m_start;
    int m_goal;
};

TEST_F(DijkstraTest, StartIsGoalTest) {
    /// Checking solver's base case where start is the same as goal
    std::vector<int> actions;
    EXPECT_TRUE(m_solver.set_start(m_start));
    EXPECT_TRUE(m_solver.set_goal(m_start));
    bool solved  = m_solver.solve(&actions);
    EXPECT_TRUE(solved);
}

TEST_F(DijkstraTest, SimpleSolverTestCozmo) {
    /// Checking solver in a medium-sized problem
    std::vector<int> actions;
    EXPECT_TRUE(m_solver.set_start(m_start));
    EXPECT_TRUE(m_solver.set_goal(m_goal));
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
