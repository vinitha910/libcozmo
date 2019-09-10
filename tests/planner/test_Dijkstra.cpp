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

namespace libcozmo {
namespace planner {
namespace test {

class DijkstraTest: public ::testing::Test {
 public:
    DijkstraTest() : \
        m_handle(),
        m_publisher(m_handle.advertise<libcozmo::ActionMsg>("Action", 10)) {}

    void SetUp() {}

    void TearDown() {}

    ros::Publisher* get_publisher() { return &m_publisher; }

    ~DijkstraTest() {}

    ros::NodeHandle m_handle;
    ros::Publisher m_publisher;
    // libcozmo::statespace::SE2 m_state_space;
    // libcozmo::actionspace::GenericActionSpace m_action_space;
    // libcozmo::model::DeterministicModel m_model;

};

/// Test very simple case of 2 states, with supposedly only one action required
TEST_F(DijkstraTest, StartIsGoalTest) {	

    libcozmo::statespace::SE2 state_space(0.1, 4);
    int start = state_space.get_or_create_state(
        libcozmo::statespace::SE2::State(0, 0, 0));
    libcozmo::actionspace::GenericActionSpace action_space(
        std::vector<double>{0.1},
        std::vector<double>{0.1},
        4);
    libcozmo::model::DeterministicModel model;
    model.load_model(new libcozmo::model::DeterministicModel::ModelType(0.1));
    libcozmo::planner::Dijkstra m_solver(&action_space, &state_space, &model);
    std::vector<int> actions;
    EXPECT_TRUE(m_solver.set_start(start));
    EXPECT_TRUE(m_solver.set_goal(start));
    bool solved  = m_solver.solve(&actions);
    EXPECT_TRUE(solved);
}

TEST_F(DijkstraTest, SimpleSolverTestCozmo) {
    libcozmo::statespace::SE2 state_space(0.1, 4);
    int start = state_space.get_or_create_state(
        libcozmo::statespace::SE2::State(0, 0, 0));
    int goal = state_space.get_or_create_state(
        libcozmo::statespace::SE2::State(50,30, 2));
    
    libcozmo::actionspace::GenericActionSpace action_space(
        std::vector<double>{0, 5, 10, 15, 20},
        std::vector<double>{1, 2, 3},
        4);

    libcozmo::model::DeterministicModel model;
    model.load_model(new libcozmo::model::DeterministicModel::ModelType(0.1));

    libcozmo::planner::Dijkstra m_solver(&action_space, &state_space, &model);
    std::vector<int> actions;
    EXPECT_TRUE(m_solver.set_start(start));
    EXPECT_TRUE(m_solver.set_goal(goal));
    bool solved  = m_solver.solve(&actions);
    EXPECT_TRUE(solved);
    std::cout<<"There are "<<actions.size()<<" actions \n";
    //Now that it is solved, let's push the actions all to cozmo
    ros::Publisher publisher = *get_publisher();
    for (int i = 0; i < actions.size(); i++) {
        bool sent = false;
        if (i  == 0) {
            // sent = action_space.publish_action(actions[i], publisher);
            sent = action_space.publish_action(actions[i], publisher);
        } else {
            sent = action_space.publish_action(actions[i], publisher);    
        }
        
        if (sent) {
            std::cout<<"msg published: action id "<<actions[i]<< " \n";
            std::cout<<"speed: "<<static_cast<libcozmo::actionspace::GenericActionSpace::Action*>(action_space.get_action(actions[i]))->m_speed<<"\n";
            std::cout<<"heading: "<<static_cast<libcozmo::actionspace::GenericActionSpace::Action*>(action_space.get_action(actions[i]))->m_heading<<"\n";
            std::cout<<"duration: "<<static_cast<libcozmo::actionspace::GenericActionSpace::Action*>(action_space.get_action(actions[i]))->m_duration<<"\n";
        }
    }
}

// TEST_F(DijkstraTest, MediumSolverTestCozmo) {
//     libcozmo::statespace::SE2 state_space(1.0, 4);
//     int start = state_space.get_or_create_state(
//         libcozmo::statespace::SE2::State(0, 0, 0));
//     int goal = state_space.get_or_create_state(
//         libcozmo::statespace::SE2::State(2, 11, 3));
    
//     libcozmo::actionspace::GenericActionSpace action_space(
//         std::vector<double>{0, 0.1, 0.25, 0.5, 0.75, 1},
//         std::vector<double>{0, 1},
//         8);

//     libcozmo::model::DeterministicModel model;
//     libcozmo::model::DeterministicModel::ModelType model_type(
//             Eigen::Vector3d(0, 0, 0),
//             0,
//             Eigen::Vector3d(0, 0, 0),
//             0);
//     model.load_model(&model_type);

//     libcozmo::planner::Dijkstra m_solver(&action_space, &state_space, &model);
//     std::vector<int> actions;
//     EXPECT_TRUE(m_solver.set_start(start));
//     EXPECT_TRUE(m_solver.set_goal(goal));
//     bool solved  = m_solver.solve(&actions);
//     EXPECT_TRUE(solved);
//     std::cout<<"There are "<<actions.size()<<" actions \n";
//     //Now that it is solved, let's push the actions all to cozmo
//     ros::Publisher publisher = *get_publisher();
//     for (int i = 0; i < actions.size(); i++) {
//         bool sent = action_space.publish_action(actions[i], publisher);
//         if (sent) {
//             std::cout<<"msg published \n";
//         }
//     }
// }


    // m_state_space.get_or_create_state(libcozmo::statespace::SE2::state(0,0,0));
	// run planner
	// Given set of actions, publish it to cozmo sequentially

}  // namespace test
}  // namespace planner
}  // namespace libcozmo

int main(int argc, char **argv) {
    ros::init(argc, argv, "Dijkstra");
    ::testing::InitGoogleTest(&argc, argv);
    // ros::AsyncSpinner spinner(1);
    // spinner.start();
    int result = RUN_ALL_TESTS();
    // spinner.stop();
    return result;
}