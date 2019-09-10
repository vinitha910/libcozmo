////////////////////////////////////////////////////////////////////////////////
//// Copyright (c) 2019, Eric Pan, Vinitha Ranganeni
//// All rights reserved.
////
//// Redistribution and use in source and binary forms, with or without
//// modification, are permitted provided that the following conditions are met:
////
////     1. Redistributions of source code must retain the above copyright notice
////        this list of conditions and the following disclaimer.
////     2. Redistributions in binary form must reproduce the above copyright
////        notice, this list of conditions and the following disclaimer in the
////        documentation and/or other materials provided with the distribution.
////     3. Neither the name of the copyright holder nor the names of its
////        contributors may be used to endorse or promote products derived from
////        this software without specific prior written permission.
////
//// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include "actionspace/ObjectOrientedActionSpace.hpp"
#include "utils/utils.hpp"

namespace as = libcozmo::actionspace;

class SimpleOOActionFixture: public ::testing::Test {
public:
    SimpleOOActionFixture() : \
         m_actionspace(libcozmo::utils::linspace(0.0, 5.0, 3.0),
                       libcozmo::utils::linspace(0.0, 5.0, 3.0), 1),
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

    void SetUp() {
        m_actionspace.generate_actions(Eigen::Vector3d(100, 200, 2));
    }

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

class ComplexOOActionFixture: public ::testing::Test {
public:
    ComplexOOActionFixture() : \
         m_actionspace(libcozmo::utils::linspace(-M_PI, M_PI, 7.0),
                       libcozmo::utils::linspace(0.0, M_E, 11.0), 5) {}

    void SetUp() {
        m_actionspace.generate_actions(Eigen::Vector3d(100, 200, 2));
    }

    as::ObjectOrientedActionSpace m_actionspace;
};

TEST_F(SimpleOOActionFixture, ActionSpaceSizeTest) {
    EXPECT_EQ(36, m_actionspace.size());
}

TEST_F(SimpleOOActionFixture, IsValidActionIDTest) {
    EXPECT_FALSE(m_actionspace.is_valid_action_id(-1));
    EXPECT_TRUE(m_actionspace.is_valid_action_id(10));
}

TEST_F(SimpleOOActionFixture, ActionGenerationTest) {
    // Tests a valid action
    as::ObjectOrientedActionSpace::Action* action =
        static_cast<as::ObjectOrientedActionSpace::Action*>(m_actionspace.get_action(5));
    EXPECT_NEAR(124.969, action->getStartPose().x(), 0.001);
    EXPECT_NEAR(145.442, action->getStartPose().y(), 0.001);
    EXPECT_EQ(2, action->getStartPose()(2));
    EXPECT_EQ(2.5, action->getSpeed());
    EXPECT_EQ(5, action->getDuration());

    // Tests nullptr action
    action = static_cast<as::ObjectOrientedActionSpace::Action*>(m_actionspace.get_action(-1));
    EXPECT_TRUE(action == nullptr);
}

TEST_F(SimpleOOActionFixture, ActionSimilarityTest) {
    double similarity;
    bool result = m_actionspace.action_similarity(0, 1, &similarity);
    ASSERT_TRUE(result);
    EXPECT_EQ(2.5, similarity);

    result = m_actionspace.action_similarity(36, 36, &similarity);
    ASSERT_FALSE(result);
}

TEST_F(SimpleOOActionFixture, PublishActionTest) {
    bool result = m_actionspace.publish_action(5, m_action_publisher);
    ASSERT_TRUE(result);
    WaitForMessage();
    libcozmo::ObjectOrientedAction* action = get_action_msg();
    EXPECT_NEAR(124.969, action->x, 0.001);
    EXPECT_NEAR(145.442, action->y, 0.001);
    EXPECT_EQ(2, action->theta);
    EXPECT_EQ(2.5, action->speed);
    EXPECT_EQ(5, action->duration);
    msg_recieved = false;

    result = m_actionspace.publish_action(-1, m_action_publisher);
    ASSERT_FALSE(result);
}

TEST_F(SimpleOOActionFixture, OverwriteActionSpaceTest) {
    m_actionspace.generate_actions(Eigen::Vector3d(300, 200, 1));

    EXPECT_EQ(36, m_actionspace.size());

    as::ObjectOrientedActionSpace::Action* action =
        static_cast<as::ObjectOrientedActionSpace::Action*>(m_actionspace.get_action(25));
    EXPECT_NEAR(332.4181, action->getStartPose().x(), 0.0001);
    EXPECT_NEAR(250.4882, action->getStartPose().y(), 0.0001);
    EXPECT_NEAR(4.14159, action->getStartPose()(2), 0.00001);
    EXPECT_EQ(5, action->getSpeed());
    EXPECT_EQ(2.5, action->getDuration());
}

TEST_F(ComplexOOActionFixture, ActionSpaceSizeTest) {
    EXPECT_EQ(1540, m_actionspace.size());
}

TEST_F(ComplexOOActionFixture, ActionGenerationTest) {
    // Tests valid action
    as::ObjectOrientedActionSpace::Action* action =
        static_cast<as::ObjectOrientedActionSpace::Action*>(m_actionspace.get_action(525));
    EXPECT_NEAR(37.1192, action->getStartPose().x(), 0.0001);
    EXPECT_NEAR(193.217, action->getStartPose().y(), 0.001);
    EXPECT_NEAR(0.429204, action->getStartPose()(2), 0.000001);
    EXPECT_NEAR(2.0944, action->getSpeed(), 0.0001);
    EXPECT_NEAR(2.17463, action->getDuration(), 0.00001);

    // Tests nullptr action
    action = static_cast<as::ObjectOrientedActionSpace::Action*>(m_actionspace.get_action(1540));
    EXPECT_TRUE(action == nullptr);
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
