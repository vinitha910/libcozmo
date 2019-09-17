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
#include "actionspace/GenericActionSpace.hpp"

class GenericActionFixture: public ::testing::Test {
 public:
    GenericActionFixture() : \
        m_handle(),
        m_msg_received(false),
        m_actionspace(std::vector<double>{0, 1}, std::vector<double>{0, 1}, 4),
        m_action(1.5, 4.0, M_PI),
        m_action_publisher(m_handle.advertise<libcozmo::ActionMsg>(
            "Action", 10)),
        m_action_subscriber(m_handle.subscribe(
            "Action", 10, &GenericActionFixture::Callback, this)) {}
    
    /// Necessary step because it takes a while for the subscriber
    /// and publisher to start up.
    void SetUp() {
        while(!IsReady()) {
            ros::spinOnce();
        }
    }

    bool IsReady() {
        return (m_action_publisher.getNumSubscribers() > 0) &&
               (m_action_subscriber.getNumPublishers() > 0);
    }

    void Callback(const libcozmo::ActionMsg& event) {
        m_msg.speed = event.speed;
        m_msg.duration = event.duration;
        m_msg.heading = event.heading;
        m_msg_received = true;
    }

    bool Publish(const int id) {
        return m_actionspace.publish_action(id, m_action_publisher);
    }

    void WaitForMessage() {
        while (m_msg_received == false) {
            ros::spinOnce();
        }
        m_msg_received = false;
    }

    libcozmo::ActionMsg* GetActionMsg() {
        return &m_msg;
    }

    ~GenericActionFixture()  {}
    libcozmo::actionspace::GenericActionSpace::Action m_action;
    libcozmo::actionspace::GenericActionSpace m_actionspace;
    ros::NodeHandle m_handle;
    ros::Publisher m_action_publisher;
    ros::Subscriber m_action_subscriber;
    libcozmo::ActionMsg m_msg;
    bool m_msg_received;
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
    EXPECT_TRUE(m_actionspace.size() == 16);
}

/// Action publisher
/// Test first and last action generated
TEST_F(GenericActionFixture, PublishActionTest) {
    Publish(0);
    WaitForMessage();
    libcozmo::ActionMsg* msg = GetActionMsg();
    EXPECT_NEAR(0, msg->speed, 0.00001);
    EXPECT_NEAR(0, msg->duration, 0.00001);
    EXPECT_NEAR(0, msg->heading, 0.00001);

    Publish(15);
    WaitForMessage();
    msg = GetActionMsg();
    EXPECT_NEAR(1, msg->speed, 0.00001);
    EXPECT_NEAR(1, msg->duration, 0.00001);
    EXPECT_NEAR(M_PI * 3.0 / 2.0, msg->heading, 0.00001);

    EXPECT_TRUE(false == Publish(16));
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
