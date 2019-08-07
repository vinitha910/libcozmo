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
#include "action_space/generic_action_space.hpp"
#include <iostream>

class GenericActionFixture: public ::testing::Test {
 public:
    GenericActionFixture() : \
        m_actionspace(0, 1, 0, 1, 2, 2, 4),
        m_action(1.5, 4.0, Eigen::Vector2d(0, 1)) {}

    void SetUp() {

        // myStateSpace.create_new_state(Eigen::Vector3i(3, 2, 1));
        // myStateSpace.create_new_state(Eigen::Vector3i(1, 3, 4));
    }

    void TearDown() {}

    ~GenericActionFixture()  {}
    // const double m_check;
    // libcozmo::statespace::Statespace myStateSpace;
    libcozmo::actionspace::GenericAction m_action;
    libcozmo::actionspace::GenericActionSpace m_actionspace;
};

// Check action
TEST_F(GenericActionFixture, ActionSimilarityTest) {
    libcozmo::actionspace::GenericAction other_action(2.5, 3.0, Eigen::Vector2d(1, 0));
    EXPECT_EQ(2, m_action.action_similarity(other_action));
}

// Check actions generated, along with get_action
TEST_F(GenericActionFixture, ActionGenerationTest) {
    libcozmo::actionspace::GenericAction* action = m_actionspace.get_action(0);

    EXPECT_NEAR(0, action->m_speed, 0.00001);
    EXPECT_NEAR(0, action->m_duration, 0.00001);
    EXPECT_NEAR(1, action->m_direction[0], 0.00001);
    EXPECT_NEAR(0, action->m_direction[1], 0.00001);

    action = m_actionspace.get_action(15);
    // EXPECT_NEAR(1, action->m_speed, 0.00001);
    // EXPECT_NEAR(1, action->m_duration, 0.00001);
    // EXPECT_NEAR(0, action->m_direction[0], 0.00001);
    // EXPECT_NEAR(-1, action->m_direction[1], 0.00001);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
