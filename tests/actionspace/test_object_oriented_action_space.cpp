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

    ~SimpleOOActionFixture()  {}

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

    void TearDown() {}

    ~ComplexOOActionFixture()  {}

    as::ObjectOrientedActionSpace m_actionspace;

};

TEST_F(SimpleOOActionFixture, ActionSpaceSizeTest) {
    EXPECT_EQ(36, m_actionspace.size());
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

    // Tests
    action = static_cast<as::ObjectOrientedActionSpace::Action*>(m_actionspace.get_action(-1));
    EXPECT_TRUE(action == nullptr);
}

TEST_F(SimpleOOActionFixture, IsValidActionIDTest) {
    EXPECT_FALSE(m_actionspace.is_valid_action_id(-1));
    EXPECT_FALSE(m_actionspace.is_valid_action_id(36));
    EXPECT_TRUE(m_actionspace.is_valid_action_id(0));
    EXPECT_TRUE(m_actionspace.is_valid_action_id(10));
}

TEST_F(SimpleOOActionFixture, ActionSimilarityTest) {
    double similarity;
    bool result = m_actionspace.action_similarity(0, 1, &similarity);
    ASSERT_TRUE(result);
    EXPECT_EQ(2.5, similarity);

    result = m_actionspace.action_similarity(0, 3, &similarity);
    ASSERT_TRUE(result);
    EXPECT_EQ(2.5, similarity);

    result = m_actionspace.action_similarity(0, 9, &similarity);
    ASSERT_TRUE(result);
    EXPECT_NEAR(84.867352, similarity, 0.00001);

    result = m_actionspace.action_similarity(5, 16, &similarity);
    ASSERT_TRUE(result);
    EXPECT_NEAR(84.940965, similarity, 0.00001);

    result = m_actionspace.action_similarity(-1, 0, &similarity);
    ASSERT_FALSE(result);

    result = m_actionspace.action_similarity(0, -1, &similarity);
    ASSERT_FALSE(result);

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

    result = m_actionspace.publish_action(10, m_action_publisher);
    ASSERT_TRUE(result);
    msg_recieved = false;
    WaitForMessage();
    action = get_action_msg();
    EXPECT_NEAR(45.4422, action->x, 0.0001);
    EXPECT_NEAR(175.031, action->y, 0.001);
    EXPECT_NEAR(0.429204, action->theta, 0.000001);
    EXPECT_EQ(0, action->speed);
    EXPECT_EQ(2.5, action->duration);
}

TEST_F(SimpleOOActionFixture, OverwriteActionSpaceTest) {
    m_actionspace.generate_actions(Eigen::Vector3d(100, 200, 2));

    EXPECT_EQ(36, m_actionspace.size());

}

TEST_F(SimpleOOActionFixture, OORObjectOrientationTest) {
    // Tests for Object Orientation < 0
    m_actionspace.generate_actions(
            Eigen::Vector3d(100, 200, 2 - (4 * M_PI)));

    as::ObjectOrientedActionSpace::Action* action =
        static_cast<as::ObjectOrientedActionSpace::Action*>(m_actionspace.get_action(5));
    EXPECT_NEAR(124.969, action->getStartPose().x(), 0.001);
    EXPECT_NEAR(145.442, action->getStartPose().y(), 0.001);
    EXPECT_EQ(2, action->getStartPose()(2));
    EXPECT_EQ(2.5, action->getSpeed());
    EXPECT_EQ(5, action->getDuration());

    // Tests for Object Orientation > 2pi
    m_actionspace.generate_actions(
        Eigen::Vector3d(100, 200, 2 + (4 * M_PI)));

    action = static_cast<as::ObjectOrientedActionSpace::Action*>(m_actionspace.get_action(5));
    EXPECT_NEAR(124.969, action->getStartPose().x(), 0.001);
    EXPECT_NEAR(145.442, action->getStartPose().y(), 0.001);
    EXPECT_EQ(2, action->getStartPose()(2));
    EXPECT_EQ(2.5, action->getSpeed());
    EXPECT_EQ(5, action->getDuration());
}

TEST_F(ComplexOOActionFixture, ActionSpaceSizeTest) {
    EXPECT_EQ(1540, m_actionspace.size());
}

TEST_F(ComplexOOActionFixture, ActionGenerationTest) {
    as::ObjectOrientedActionSpace::Action* action =
        static_cast<as::ObjectOrientedActionSpace::Action*>(m_actionspace.get_action(75));
    EXPECT_NEAR(88.5969, action->getStartPose().x(), 0.0001);
    EXPECT_NEAR(128.796, action->getStartPose().y(), 0.001);
    EXPECT_EQ(2, action->getStartPose()(2));
    EXPECT_NEAR(3.14159, action->getSpeed(), 0.00001);
    EXPECT_NEAR(2.44645, action->getDuration(), 0.00001);

    action = static_cast<as::ObjectOrientedActionSpace::Action*>(m_actionspace.get_action(150));
    EXPECT_NEAR(106.783, action->getStartPose().x(), 0.001);
    EXPECT_NEAR(137.119, action->getStartPose().y(), 0.001);
    EXPECT_EQ(2, action->getStartPose()(2));
    EXPECT_NEAR(3.14159, action->getSpeed(), 0.00001);
    EXPECT_NEAR(1.9028, action->getDuration(), 0.0001);

    action = static_cast<as::ObjectOrientedActionSpace::Action*>(m_actionspace.get_action(225));
    EXPECT_NEAR(124.969, action->getStartPose().x(), 0.001);
    EXPECT_NEAR(145.442, action->getStartPose().y(), 0.001);
    EXPECT_EQ(2, action->getStartPose()(2));
    EXPECT_NEAR(3.14159, action->getSpeed(), 0.00001);
    EXPECT_NEAR(1.35914, action->getDuration(), 0.00001);

    action = static_cast<as::ObjectOrientedActionSpace::Action*>(m_actionspace.get_action(300));
    EXPECT_NEAR(143.155, action->getStartPose().x(), 0.001);
    EXPECT_NEAR(153.765, action->getStartPose().y(), 0.001);
    EXPECT_EQ(2, action->getStartPose()(2));
    EXPECT_NEAR(3.14159, action->getSpeed(), 0.00001);
    EXPECT_NEAR(0.815485, action->getDuration(), 0.000001);

    action = static_cast<as::ObjectOrientedActionSpace::Action*>(m_actionspace.get_action(375));
    EXPECT_NEAR(161.341, action->getStartPose().x(), 0.001);
    EXPECT_NEAR(162.088, action->getStartPose().y(), 0.001);
    EXPECT_EQ(2, action->getStartPose()(2));
    EXPECT_NEAR(3.14159, action->getSpeed(), 0.00001);
    EXPECT_NEAR(0.271828, action->getDuration(), 0.000001);

    action = static_cast<as::ObjectOrientedActionSpace::Action*>(m_actionspace.get_action(450));
    EXPECT_NEAR(28.7963, action->getStartPose().x(), 0.0001);
    EXPECT_NEAR(211.403, action->getStartPose().y(), 0.001);
    EXPECT_NEAR(0.429204, action->getStartPose()(2), 0.000001);
    EXPECT_NEAR(2.0944, action->getSpeed(), 0.0001);
    EXPECT_NEAR(2.71828, action->getDuration(), 0.00001);

    action = static_cast<as::ObjectOrientedActionSpace::Action*>(m_actionspace.get_action(525));
    EXPECT_NEAR(37.1192, action->getStartPose().x(), 0.0001);
    EXPECT_NEAR(193.217, action->getStartPose().y(), 0.001);
    EXPECT_NEAR(0.429204, action->getStartPose()(2), 0.000001);
    EXPECT_NEAR(2.0944, action->getSpeed(), 0.0001);
    EXPECT_NEAR(2.17463, action->getDuration(), 0.00001);

    action = static_cast<as::ObjectOrientedActionSpace::Action*>(m_actionspace.get_action(600));
    EXPECT_NEAR(45.4422, action->getStartPose().x(), 0.0001);
    EXPECT_NEAR(175.031, action->getStartPose().y(), 0.001);
    EXPECT_NEAR(0.429204, action->getStartPose()(2), 0.000001);
    EXPECT_NEAR(2.0944, action->getSpeed(), 0.0001);
    EXPECT_NEAR(1.63097, action->getDuration(), 0.00001);

    action = static_cast<as::ObjectOrientedActionSpace::Action*>(m_actionspace.get_action(675));
    EXPECT_NEAR(53.7651, action->getStartPose().x(), 0.0001);
    EXPECT_NEAR(156.845, action->getStartPose().y(), 0.001);
    EXPECT_NEAR(0.429204, action->getStartPose()(2), 0.000001);
    EXPECT_NEAR(2.0944, action->getSpeed(), 0.0001);
    EXPECT_NEAR(1.08731, action->getDuration(), 0.00001);

    action = static_cast<as::ObjectOrientedActionSpace::Action*>(m_actionspace.get_action(750));
    EXPECT_NEAR(62.088, action->getStartPose().x(), 0.0001);
    EXPECT_NEAR(138.659, action->getStartPose().y(), 0.001);
    EXPECT_NEAR(0.429204, action->getStartPose()(2), 0.000001);
    EXPECT_NEAR(2.0944, action->getSpeed(), 0.0001);
    EXPECT_NEAR(0.543656, action->getDuration(), 0.00001);
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
