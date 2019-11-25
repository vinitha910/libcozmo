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
#include "actionspace/GenericActionSpace.hpp"

class GenericActionFixture: public ::testing::Test {
 public:
    GenericActionFixture() : \
        m_actionspace(std::vector<double>{0, 1}, std::vector<double>{0, 1}, 4),
        m_action(1.5, 4.0, M_PI) {}

    ~GenericActionFixture()  {}
    libcozmo::actionspace::GenericActionSpace::Action m_action;
    libcozmo::actionspace::GenericActionSpace m_actionspace;
};

/// Tests action similarity
TEST_F(GenericActionFixture, ActionSimilarityTest) {
    double similarity = 0;
    ASSERT_TRUE(m_actionspace.action_similarity(0, 12, &similarity));
    EXPECT_NEAR(sqrt(2), similarity, 0.0001);
}

/// Check actions generated, along with get_action
/// Testing get_action also tests is_valid_action_id
TEST_F(GenericActionFixture, ActionGenerationTest) {
    libcozmo::actionspace::GenericActionSpace::Action* action =
        static_cast<libcozmo::actionspace::GenericActionSpace::Action*>(
            m_actionspace.get_action(0));
    EXPECT_NEAR(0, action->m_speed, 0.00001);
    EXPECT_NEAR(0, action->m_duration, 0.00001);
    EXPECT_NEAR(0, action->m_heading, 0.00001);

    action = static_cast<libcozmo::actionspace::GenericActionSpace::Action*>(
        m_actionspace.get_action(15));
    EXPECT_NEAR(1, action->m_speed, 0.00001);
    EXPECT_NEAR(1, action->m_duration, 0.00001);
    EXPECT_NEAR(M_PI * 3.0 / 2.0, action->m_heading, 0.00001);
}

/// Check out of range handling for action_similarity method
TEST_F(GenericActionFixture, ActionSimilarityOOR) {
    double similarity = 0.0;
    EXPECT_FALSE(m_actionspace.action_similarity(0, 61, &similarity));
    EXPECT_FALSE(m_actionspace.action_similarity(0, -5, &similarity));
}

/// Check out of range handling for get_action method
TEST_F(GenericActionFixture, GetActionOOR) {
    EXPECT_TRUE(m_actionspace.get_action(61) == nullptr);
    EXPECT_TRUE(m_actionspace.get_action(-1) == nullptr);
}

/// Check size method
TEST_F(GenericActionFixture, GetSize) {
    EXPECT_EQ(m_actionspace.size(), 16);
}

/// Check generated action vector
TEST_F(GenericActionFixture, ActionVectorTest) {
    libcozmo::actionspace::GenericActionSpace::Action* action =
        static_cast<libcozmo::actionspace::GenericActionSpace::Action*>(
            m_actionspace.get_action(15));
    Eigen::VectorXd action_vector = action->vector();
    EXPECT_NEAR(1, action_vector[0], 0.00001);
    EXPECT_NEAR(1, action_vector[1], 0.00001);
    EXPECT_NEAR(M_PI * 3.0 / 2.0, action_vector[2], 0.00001);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    int ret =  RUN_ALL_TESTS();
    return ret;
}
