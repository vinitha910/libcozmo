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
#include <iostream>
#include "action_space/generic_action_space.hpp"

class GenericActionFixture: public ::testing::Test {
 public:
    GenericActionFixture() : \
        m_handle(),
        m_actionspace(0, 1, 0, 1, 2, 2, 4),
        m_action(1.5, 4.0, Eigen::Vector2d(0, 1)),
        m_action_publisher(m_handle.advertise<libcozmo::ActionMsg>("Action", 10)),
        m_action_subscriber(m_handle.subscribe("Action", 10, &GenericActionFixture::Callback, this)) {}

    void SetUp() {
        while(!IsReady()) {
            ros::spinOnce();
        }
    }

    void TearDown() {
    }

    bool IsReady() {
        return (m_action_publisher.getNumSubscribers() > 0) &&
            (m_action_subscriber.getNumPublishers() > 0);
    }

    void Callback(const libcozmo::ActionMsg& event) {
        msg = event;
    }

    void Publish(const int id) {
        m_actionspace.publish_action(id, m_action_publisher);
    }

    libcozmo::ActionMsg get_action_msg() {
        return msg;
    }
    ~GenericActionFixture()  {}
    libcozmo::actionspace::GenericActionSpace::Action m_action;
    libcozmo::actionspace::GenericActionSpace m_actionspace;
    ros::NodeHandle m_handle;
    ros::Publisher m_action_publisher;
    ros::Subscriber m_action_subscriber;
    libcozmo::ActionMsg msg;
};

// Check action similarity works
TEST_F(GenericActionFixture, ActionSimilarityTest) {
    EXPECT_EQ(2, m_actionspace.action_similarity(0, 15));
}

// Check actions generated, along with get_action
TEST_F(GenericActionFixture, ActionGenerationTest) {
    libcozmo::actionspace::GenericActionSpace::Action action(-1, -1, Eigen::Vector2d(-1, -1));
    ASSERT_TRUE(m_actionspace.get_action(0, &action));
    EXPECT_NEAR(0, action.m_speed, 0.00001);
    EXPECT_NEAR(0, action.m_duration, 0.00001);
    EXPECT_NEAR(1, action.m_direction[0], 0.00001);
    EXPECT_NEAR(0, action.m_direction[1], 0.00001);

    ASSERT_TRUE(m_actionspace.get_action(15, &action));
    EXPECT_NEAR(1, action.m_speed, 0.00001);
    EXPECT_NEAR(1, action.m_duration, 0.00001);
    EXPECT_NEAR(0, action.m_direction[0], 0.00001);
    EXPECT_NEAR(-1, action.m_direction[1], 0.00001);
}

// Check out of range exception handling for action_similarity and get_action
TEST_F(GenericActionFixture, OORExceptionTest) {
    // EXPECT_THROW(m_actionspace.action_similarity(0, 61), std::out_of_range);
    // EXPECT_THROW(m_actionspace.action_similarity(0, -5), std::out_of_range);
    // EXPECT_THROW(m_actionspace.get_action(61), std::out_of_range);
    // EXPECT_THROW(m_actionspace.get_action(-1), std::out_of_range);
}

// Action publisher
TEST_F(GenericActionFixture, PublishActionTest) {
    // Publish(0);
    // libcozmo::ActionMsg msg = get_action_msg();
    // EXPECT_NEAR(0, msg.speed, 0.00001);
    // EXPECT_NEAR(0, msg.duration, 0.00001);
    // EXPECT_NEAR(0, msg.heading, 0.00001);

    // Publish(15);
    // msg = get_action_msg();
    // EXPECT_NEAR(1, msg.speed, 0.00001);
    // EXPECT_NEAR(1, msg.duration, 0.00001);
    // EXPECT_NEAR(1, msg.heading, 0.00001);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");
    testing::InitGoogleTest(&argc, argv);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    int ret =  RUN_ALL_TESTS();
    spinner.stop();
    return ret;
}
