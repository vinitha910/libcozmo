////////////////////////////////////////////////////////////////////////////////
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

#include <gtest/gtest.h>
#include "statespace/SE2.hpp"

namespace libcozmo {
namespace statespace {
namespace test {

class SE2StatespaceTest: public ::testing::Test {
 public:
    SE2StatespaceTest() : \
        statespace(0.1, 8), resolution(0.1), num_theta_vals(8) {}

    void SetUp() {
        SE2::State state_1(3, 2, 1);
        statespace.get_or_create_state(&state_1);

        SE2::State state_2(1, 3, 3);
        statespace.get_or_create_state(&state_2);
    }

    void TearDown() {}

    ~SE2StatespaceTest()  {}

    SE2 statespace;
    aikido::statespace::SE2 continuous_statespace;
    const double resolution;
    const int num_theta_vals;
};

TEST_F(SE2StatespaceTest, GetsOrCreatesDiscreteState) {
    // Cheking existing states IDs are returned correctly
    SE2::State state_1(3, 2, 1);
    EXPECT_EQ(0, statespace.get_or_create_state(&state_1));
    SE2::State state_2(1, 3, 3);
    EXPECT_EQ(1, statespace.get_or_create_state(&state_2));

    // Checking that new state is created and appropiate ID is returned
    SE2::State state_3(1, 1, 1);
    EXPECT_EQ(2, statespace.get_or_create_state(&state_3));
}

TEST_F(SE2StatespaceTest, GetsOrCreatesContinuousState) {
    // Checking that new state is created and appropiate ID is returned
    aikido::statespace::SE2::State state;
    continuous_statespace.expMap(Eigen::Vector3d(1, 1, 1), &state);
    EXPECT_EQ(2, statespace.get_or_create_state(&state));
}

TEST_F(SE2StatespaceTest, DiscreteToContinuousStateConversion) {
    SE2::State in_state(1, 2, 1);
    aikido::statespace::SE2::State out_state;
    statespace.discrete_state_to_continuous(&in_state, &out_state);

    Eigen::VectorXd log_state;
    continuous_statespace.logMap(&out_state, log_state);
    EXPECT_DOUBLE_EQ(log_state[0], 0.15);
    EXPECT_DOUBLE_EQ(log_state[1], 0.25);
    EXPECT_DOUBLE_EQ(log_state[2], M_PI/4);
}

TEST_F(SE2StatespaceTest, ContinuousToDiscreteStateConversion) {
    aikido::statespace::SE2::State in_state;
    continuous_statespace.expMap(Eigen::Vector3d(0.17, 0.257, M_PI/5), &in_state);
    
    SE2::State out_state;
    statespace.continuous_state_to_discrete(&in_state, &out_state);

    EXPECT_DOUBLE_EQ(out_state.x, 1);
    EXPECT_DOUBLE_EQ(out_state.y, 2);
    EXPECT_DOUBLE_EQ(out_state.theta, 1);
}

// // Check state added correctly
// TEST_F(SE2StatespaceTest, UnitTest1) {
//     Eigen::Vector3i expected(3, 2, 1);
//     Eigen::Vector3i state;
//     statespace.get_coord_from_state_id(0, &state);
//     EXPECT_EQ(expected[0], state[0]);
//     EXPECT_EQ(expected[1], state[1]);
//     EXPECT_EQ(expected[2], state[2]);
//     expected << 1, 3, 3;
//     statespace.get_coord_from_state_id(1, &state);
//     EXPECT_EQ(expected[0], state[0]);
//     EXPECT_EQ(expected[1], state[1]);
//     EXPECT_EQ(expected[2], state[2]);
// }

// // Check distnguishing valid_state
// TEST_F(SE2StatespaceTest, UnitTest2) {
//     EXPECT_EQ(true, statespace.is_valid_state(Eigen::Vector3i(15, 2, 1)));
//     EXPECT_EQ(false, statespace.is_valid_state(Eigen::Vector3i(-1, -2, -5)));
//     EXPECT_EQ(false, statespace.is_valid_state(Eigen::Vector3i(0, 150, -1)));
//     EXPECT_EQ(true, statespace.is_valid_state(Eigen::Vector3i(1, 2, 1)));
//     EXPECT_EQ(true, statespace.is_valid_state(Eigen::Vector3i(9, 8, 2)));
// }

// // Check continuous --> discrete pose (SE2 input)
// TEST_F(SE2StatespaceTest, UnitTest3) {
//     aikido::statespace::SE2::State continuous_state;
//     Eigen::Isometry2d t = Eigen::Isometry2d::Identity();
//     const Eigen::Rotation2D<double> rot(M_PI);
//     t.linear() = rot.toRotationMatrix();
//     Eigen::Vector2d trans;
//     trans << 5.4, 2.0;
//     t.translation() = trans;
//     continuous_state.setIsometry(t);
//     Eigen::Vector3i pose = 
//         statespace.continuous_state_to_discrete(continuous_state);
//     EXPECT_EQ(54, pose.x());
//     EXPECT_EQ(20, pose.y());
//     double angle = 2;
//     EXPECT_EQ(angle, pose[2]);
// }

// // Check path generation
// TEST_F(SE2StatespaceTest, UnitTest4) {
//     int myints[] = {0, 1};
//     std::vector<int> state_ids;
//     for (int i = 0; i < 2; i++) {
//         state_ids.push_back(myints[i]);
//     }
//     std::vector<Eigen::Vector3i> path_coordinates;
//     std::vector<Eigen::Vector3i> expected;
//     expected.push_back(Eigen::Vector3i(3, 2, 1));
//     expected.push_back(Eigen::Vector3i(1, 3, 3));
//     statespace.get_path_states(state_ids, &path_coordinates);
//     EXPECT_EQ(expected[0][0], path_coordinates[0][0]);
//     EXPECT_EQ(expected[0][1], path_coordinates[0][1]);
//     EXPECT_EQ(expected[0][2], path_coordinates[0][2]);
//     EXPECT_EQ(expected[1][0], path_coordinates[1][0]);
//     EXPECT_EQ(expected[1][1], path_coordinates[1][1]);
//     EXPECT_EQ(expected[1][2], path_coordinates[1][2]);
// }

// // Check create_state and get_or_create_state
// TEST_F(SE2StatespaceTest, UnitTest5) {
//     statespace.get_or_create_new_state(Eigen::Vector3i(1, 3, 3));
//     EXPECT_EQ(2, statespace.get_num_states());
// }

}  // namespace test
}  // namspace statespace
}  // namespace libcozmo

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}