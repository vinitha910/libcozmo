////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019, Brian Lee, Eric Pan, Vinitha Ranganeni
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
#include "actionspace/ObjectOrientedActionSpace.hpp"
#include "utils/utils.hpp"

class OOActionSpaceFixture: public ::testing::Test {
 public:
    OOActionSpaceFixture() : \
         m_actionspace(
            libcozmo::utils::linspace(0.0, 5.0, 3.0),
            std::vector<double>{4.0, 1.1},
            Eigen::Vector2d(6.0, 3.1),
            Eigen::Vector2d(5.0, 2.1),
            5) {
            object_state = create_object_state();
            }

    aikido::statespace::SE2::State create_object_state() {
        Eigen::Isometry2d transform(Eigen::Isometry2d::Identity());
        transform.linear() = Eigen::Rotation2Dd(M_PI/4).matrix();
        transform.translation() = Eigen::Vector2d(15, 15);
        aikido::statespace::SE2::State state;
        state.setIsometry(transform);
        return state;
    }

    libcozmo::actionspace::ObjectOrientedActionSpace m_actionspace;
    aikido::statespace::SE2::State object_state;
};

/// Check that correct amount of actions were generated
/// 4 sides, 5 edge offset, 3 speed --> 60 actions total
TEST_F(OOActionSpaceFixture, ActionSpaceSizeTest) {
    EXPECT_EQ(60, m_actionspace.size());
}

/// Check that valid/invalid IDs are handled correctly
TEST_F(OOActionSpaceFixture, IsValidActionIDTest) {
    EXPECT_FALSE(m_actionspace.is_valid_action_id(-1));
    EXPECT_TRUE(m_actionspace.is_valid_action_id(10));
}

TEST_F(OOActionSpaceFixture, ActionSimilarityTest) {
    double similarity;
    bool result = m_actionspace.action_similarity(0, 4, &similarity);
    ASSERT_TRUE(result);
    EXPECT_NEAR(2.549, similarity, 0.001);

    result = m_actionspace.action_similarity(72, 1, &similarity);
    ASSERT_FALSE(result);
}

/// Check that actions on opposite sides of the object have the same speed,
/// aspect ratio, and edge offset but heading offsets that differ by pi
TEST_F(OOActionSpaceFixture, UniqueActionTest) {
    for (int i = 0; i < m_actionspace.size()/2; ++i) {
        const auto action1 =
            static_cast<
                libcozmo::actionspace::
                    ObjectOrientedActionSpace::Action*>(
                    m_actionspace.get_action(i));
        const auto action2 =
            static_cast<
                libcozmo::actionspace::
                    ObjectOrientedActionSpace::Action*>(
                    m_actionspace.get_action(i + m_actionspace.size()/2));

        EXPECT_EQ(action1->speed(), action2->speed());
        EXPECT_EQ(action1->aspect_ratio(), action2->aspect_ratio());
        EXPECT_EQ(action1->edge_offset(), action2->edge_offset());
        EXPECT_EQ(action1->heading_offset(), action2->heading_offset() - M_PI);
    }
}

TEST_F(OOActionSpaceFixture, ActionGenerationTest) {
    // Tests a valid action
    libcozmo::actionspace::ObjectOrientedActionSpace::Action* action =
        static_cast<
            libcozmo::actionspace::ObjectOrientedActionSpace::Action*>(
            m_actionspace.get_action(4));
    EXPECT_EQ(2.5, action->speed());
    EXPECT_NEAR(0, action->heading_offset(), 0.001);
    EXPECT_EQ(-0.5, action->edge_offset());
    EXPECT_NEAR(4, action->aspect_ratio(), 0.01);

    // Tests nullptr action
    action = static_cast<
        libcozmo::actionspace::ObjectOrientedActionSpace::Action*>(
        m_actionspace.get_action(-1));
    EXPECT_TRUE(action == nullptr);
}

TEST_F(OOActionSpaceFixture, GetObjectOrientedActionTest) {
    libcozmo::actionspace::ObjectOrientedActionSpace::CozmoAction
        action(0.0, Eigen::Vector3d(0, 0, 0));
    m_actionspace.get_generic_to_object_oriented_action(
        4,
        object_state,
        &action);
    EXPECT_NEAR(action.start_pose().x(), 8.989592, 0.001);
    EXPECT_NEAR(action.start_pose().y(), 12.525126, 0.001);
    EXPECT_NEAR(action.start_pose().z(), M_PI/4, 0.001);

    m_actionspace.get_generic_to_object_oriented_action(
        47,
        object_state,
        &action);
    EXPECT_NEAR(action.start_pose().x(), 15.70710, 0.001);
    EXPECT_NEAR(action.start_pose().y(), 11.32304, 0.001);
    EXPECT_NEAR(action.start_pose().z(), 3 * M_PI/2 + M_PI/4, 0.001);
}

TEST_F(OOActionSpaceFixture, ActionVectorTest) {
    // Tests action vector generation for Generic Action
    libcozmo::actionspace::ObjectOrientedActionSpace::Action* action =
        static_cast<
            libcozmo::actionspace::ObjectOrientedActionSpace::Action*>(
            m_actionspace.get_action(4));
    Eigen::VectorXd action_vector = action->vector();
    EXPECT_NEAR(2.5, action_vector[0], 0.00001);
    EXPECT_NEAR(4, action_vector[1], 0.00001);
    EXPECT_NEAR(-0.5, action_vector[2], 0.00001);
    EXPECT_NEAR(0, action_vector[3], 0.00001);
}

TEST_F(OOActionSpaceFixture, CozmoActionVectorTest) {
    // Tests action vector generation for OOAction
    libcozmo::actionspace::ObjectOrientedActionSpace::
        CozmoAction action(3.0, Eigen::Vector3d{1, 2, 3});
    Eigen::VectorXd action_vector = action.vector();
    EXPECT_NEAR(3.0, action_vector[0], 0.00001);
    EXPECT_NEAR(1, action_vector[1], 0.00001);
    EXPECT_NEAR(2, action_vector[2], 0.00001);
    EXPECT_NEAR(3, action_vector[3], 0.00001);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    return ret;
}
