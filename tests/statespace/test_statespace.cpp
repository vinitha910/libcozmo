////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019, Vinitha Ranganeni, Brian Lee
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
#include "statespace/statespace.hpp"

class myTestFixture1: public ::testing::Test {
 public:
    myTestFixture1() : \
    myStateSpace(0.1, 4) {}

    void SetUp() {
        myStateSpace.create_new_state(Eigen::Vector3i(3, 2, 1));
        myStateSpace.create_new_state(Eigen::Vector3i(1, 3, 3));
    }

    void TearDown() {}

    ~myTestFixture1()  {}
    libcozmo::statespace::Statespace myStateSpace;
};

// Check state added correctly
TEST_F(myTestFixture1, UnitTest1) {
    Eigen::Vector3i expected(3, 2, 1);
    Eigen::Vector3i state;
    myStateSpace.get_coord_from_state_id(0, &state);
    EXPECT_EQ(expected[0], state[0]);
    EXPECT_EQ(expected[1], state[1]);
    EXPECT_EQ(expected[2], state[2]);
    expected << 1, 3, 3;
    myStateSpace.get_coord_from_state_id(1, &state);
    EXPECT_EQ(expected[0], state[0]);
    EXPECT_EQ(expected[1], state[1]);
    EXPECT_EQ(expected[2], state[2]);
}

// Check distnguishing valid_state
TEST_F(myTestFixture1, UnitTest2) {
    EXPECT_EQ(true, myStateSpace.is_valid_state(Eigen::Vector3i(15, 2, 1)));
    EXPECT_EQ(false, myStateSpace.is_valid_state(Eigen::Vector3i(-1, -2, -5)));
    EXPECT_EQ(false, myStateSpace.is_valid_state(Eigen::Vector3i(0, 150, -1)));
    EXPECT_EQ(true, myStateSpace.is_valid_state(Eigen::Vector3i(1, 2, 1)));
    EXPECT_EQ(true, myStateSpace.is_valid_state(Eigen::Vector3i(9, 8, 2)));
}

// Check continuous --> discrete pose (SE2 input)
TEST_F(myTestFixture1, UnitTest3) {
    aikido::statespace::SE2::State continuous_state;
    Eigen::Isometry2d t = Eigen::Isometry2d::Identity();
    const Eigen::Rotation2D<double> rot(M_PI);
    t.linear() = rot.toRotationMatrix();
    Eigen::Vector2d trans;
    trans << 5.4, 2.0;
    t.translation() = trans;
    continuous_state.setIsometry(t);
    Eigen::Vector3i pose = myStateSpace.continuous_state_to_discrete(
        continuous_state);
    EXPECT_EQ(54, pose.x());
    EXPECT_EQ(20, pose.y());
    double angle = 2;
    EXPECT_EQ(angle, pose[2]);
}

// Check path generation
TEST_F(myTestFixture1, UnitTest4) {
    int myints[] = {0, 1};
    std::vector<int> state_ids;
    for (int i = 0; i < 2; i++) {
        state_ids.push_back(myints[i]);
    }
    std::vector<Eigen::Vector3i> path_coordinates;
    std::vector<Eigen::Vector3i> expected;
    expected.push_back(Eigen::Vector3i(3, 2, 1));
    expected.push_back(Eigen::Vector3i(1, 3, 3));
    myStateSpace.get_path_states(state_ids, &path_coordinates);
    EXPECT_EQ(expected[0][0], path_coordinates[0][0]);
    EXPECT_EQ(expected[0][1], path_coordinates[0][1]);
    EXPECT_EQ(expected[0][2], path_coordinates[0][2]);
    EXPECT_EQ(expected[1][0], path_coordinates[1][0]);
    EXPECT_EQ(expected[1][1], path_coordinates[1][1]);
    EXPECT_EQ(expected[1][2], path_coordinates[1][2]);
}

// Check create_state and get_or_create_state
TEST_F(myTestFixture1, UnitTest5) {
    myStateSpace.get_or_create_new_state(Eigen::Vector3i(1, 3, 3));
    EXPECT_EQ(2, myStateSpace.get_num_states());
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
