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

namespace as = libcozmo::actionspace;

class SimpleOOActionFixture: public ::testing::Test {
 public:
    SimpleOOActionFixture() : \
         m_actionspace(
            libcozmo::utils::linspace(0.0, 5.0, 3.0),
            std::vector<double>{4.0, 1.1},
            40,
            5),
         m_handle(),
         m_action_publisher(
            m_handle.advertise<libcozmo::ObjectOrientedAction>("Action", 10)),
         m_action_subscriber(
            m_handle.subscribe(
                "Action",
                10,
                &SimpleOOActionFixture::Callback,
                this)),
         msg_recieved(false) {}

    libcozmo::ObjectOrientedAction* get_action_msg() {
        return &msg;
    }

    void WaitForMessage() const {
        while (!msg_recieved) {
            ros::spinOnce();
        }
    }

    as::ObjectOrientedActionSpace m_actionspace;
    ros::NodeHandle m_handle;
    ros::Publisher m_action_publisher;
    ros::Subscriber m_action_subscriber;
    libcozmo::ObjectOrientedAction msg;
    bool msg_recieved;

 private:
    void Callback(const libcozmo::ObjectOrientedAction& event) {
        msg.speed = event.speed;
        msg.duration = event.duration;
        msg.x = event.x;
        msg.y = event.y;
        msg.theta = event.theta;
        msg_recieved = true;
    }
};

TEST_F(SimpleOOActionFixture, ActionSpaceSizeTest) {
    /// Check that correct amount of actions were generated
    /// 4 sides, 5 edge offset, 3 speed --> 60 actions total
    EXPECT_EQ(60, m_actionspace.size());
}

TEST_F(SimpleOOActionFixture, IsValidActionIDTest) {
    /// Check that valid/invalid IDs are handled correctly
    EXPECT_FALSE(m_actionspace.is_valid_action_id(-1));
    EXPECT_TRUE(m_actionspace.is_valid_action_id(10));
}

TEST_F(SimpleOOActionFixture, ActionGenerationTest) {
    // Tests a valid action
    as::ObjectOrientedActionSpace::GenericAction* action =
        static_cast<as::ObjectOrientedActionSpace::GenericAction*>(
            m_actionspace.get_action(4));
    EXPECT_EQ(2.5, action->getSpeed());
    EXPECT_NEAR(0, action->getHeadingOffset(), 0.001);
    EXPECT_EQ(0.5, action->getEdgeOffset());
    EXPECT_NEAR(4.0, action->getAspectRatio(), 0.01);

    // Tests nullptr action
    action = static_cast<as::ObjectOrientedActionSpace::GenericAction*>(
        m_actionspace.get_action(-1));
    EXPECT_TRUE(action == nullptr);
}

TEST_F(SimpleOOActionFixture, ActionSimilarityTest) {
    double similarity;
    bool result = m_actionspace.action_similarity(0, 4, &similarity);
    ASSERT_TRUE(result);
    EXPECT_NEAR(2.549, similarity, 0.001);

    result = m_actionspace.action_similarity(72, 1, &similarity);
    ASSERT_FALSE(result);
}

TEST_F(SimpleOOActionFixture, PublishActionTest) {
    /// Check that action published with updated cube pose is correct
    Eigen::Isometry2d transform(Eigen::Isometry2d::Identity());
    transform.linear() = Eigen::Rotation2Dd(M_PI/2).matrix();
    transform.translation() = Eigen::Vector2d(15, 15);
    aikido::statespace::SE2::State continuous_state;
    continuous_state.setIsometry(transform);
    bool result = m_actionspace.publish_action(
        4,
        m_action_publisher,
        continuous_state);
    ASSERT_TRUE(result);
    WaitForMessage();
    libcozmo::ObjectOrientedAction* action = get_action_msg();
    EXPECT_NEAR(-45, action->x, 0.001);
    EXPECT_NEAR(35, action->y, 0.001);
    EXPECT_EQ(M_PI / 2, action->theta);
    EXPECT_EQ(2.5, action->speed);
    EXPECT_EQ(1, action->duration);
    msg_recieved = false;
    result = m_actionspace.publish_action(
        -1,
        m_action_publisher,
        continuous_state);
    ASSERT_FALSE(result);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tester");
    testing::InitGoogleTest(&argc, argv);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    return ret;
}
