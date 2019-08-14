#include <gtest/gtest.h>
#include "actionspace/object_oriented_action_space.hpp"
#include "utils/utils.hpp"

// Tests all basic functionalities and properties of object oriented action space including:
//    size of action space
//    actions generated by action space
//    action similarity
//    exception handling for invalid arguments
//    message publisher
class SimpleOOActionFixture: public ::testing::Test {
public:
    SimpleOOActionFixture() : \
         subscribe_count(0),
         m_handle(),
         m_action_publisher(m_handle.advertise<libcozmo::ObjectOrientedAction>("Action", 10)),
         m_action_subscriber(m_handle.subscribe("Action", 10, &SimpleOOActionFixture::Callback, this)){}

    void SetUp() {
        m_actionspace.generate_actions(Eigen::Vector2d(100, 200), 2);
        while (!IsReady()) {
            ros::spinOnce();
        }
    }

    void TearDown() {}

    libcozmo::ObjectOrientedAction* get_action_msg() {
        return &msg;
    }

    void Publish(const int& id) const {
        m_actionspace.publish_action(id, m_action_publisher);
    }

    void WaitForMessage() const {
        boost::shared_ptr<const libcozmo::ObjectOrientedAction> msg =
            ros::topic::waitForMessage<libcozmo::ObjectOrientedAction>(
                "Action",
                ros::Duration(0.01));
    }

    ~SimpleOOActionFixture()  {}

    std::vector<double> speeds = libcozmo::utils::linspace(0.0, 5.0, 3.0);
    std::vector<double> durations = libcozmo::utils::linspace(0.0, 5.0, 3.0);
    libcozmo::actionspace::ObjectOrientedActionSpace m_actionspace{speeds, durations, 1};
    ros::NodeHandle m_handle;
    ros::Publisher m_action_publisher;
    ros::Subscriber m_action_subscriber;
    libcozmo::ObjectOrientedAction msg;
    int subscribe_count;

private:
    void Callback(const libcozmo::ObjectOrientedAction& event) {
        msg.speed = event.speed;
        msg.duration = event.duration;
        msg.x = event.x;
        msg.y = event.y;
        msg.theta = event.theta;
        subscribe_count++;
    }

    bool IsReady() const {
         return (m_action_publisher.getNumSubscribers() > 0) &&
             (m_action_subscriber.getNumPublishers() > 0);
    }

};

// Tests for object orientation < 0
class SimpleOOActionFixture2: public ::testing::Test {
public:
    SimpleOOActionFixture2() {}

    void SetUp( ) {
        m_actionspace.generate_actions(Eigen::Vector2d(100, 200), 2 - (4 * M_PI));
    }

    void TearDown( ) {}

    ~SimpleOOActionFixture2( )  {}

    std::vector<double> speeds = libcozmo::utils::linspace(0.0, 5.0, 3.0);
    std::vector<double> durations = libcozmo::utils::linspace(0.0, 5.0, 3.0);
    libcozmo::actionspace::ObjectOrientedActionSpace m_actionspace{speeds, durations, 1};

};

// Tests for object orientation > 2pi
class SimpleOOActionFixture3: public ::testing::Test {
public:
    SimpleOOActionFixture3() {}

    void SetUp( ) {
        m_actionspace.generate_actions(Eigen::Vector2d(100, 200), 2 + (4 * M_PI));
    }

    void TearDown( ) {}

    ~SimpleOOActionFixture3( )  {}

    std::vector<double> speeds = libcozmo::utils::linspace(0.0, 5.0, 3.0);
    std::vector<double> durations = libcozmo::utils::linspace(0.0, 5.0, 3.0);
    libcozmo::actionspace::ObjectOrientedActionSpace m_actionspace{speeds, durations, 1};

};

class ComplexOOActionFixture: public ::testing::Test {
public:
    ComplexOOActionFixture() {}

    void SetUp( ) {
        m_actionspace.generate_actions(Eigen::Vector2d(100, 200), 2);
    }

    void TearDown( ) {}

    ~ComplexOOActionFixture( )  {}

    std::vector<double> speeds = libcozmo::utils::linspace(-M_PI, M_PI, 7.0);
    std::vector<double> durations = libcozmo::utils::linspace(0.0, M_E, 11.0);
    libcozmo::actionspace::ObjectOrientedActionSpace m_actionspace{speeds, durations, 5};

};

TEST_F(SimpleOOActionFixture, ActionSpaceSizeTest) {
    EXPECT_EQ(36, m_actionspace.size());
}

TEST_F(SimpleOOActionFixture, ActionGenerationTest) {
    libcozmo::actionspace::ObjectOrientedActionSpace::ObjectOrientedAction* action = m_actionspace.get_action(5);
    EXPECT_NEAR(124.969, action->start_pos.x(), 0.001);
    EXPECT_NEAR(145.442, action->start_pos.y(), 0.001);
    EXPECT_EQ(2, action->theta);
    EXPECT_EQ(2.5, action->speed);
    EXPECT_EQ(5, action->duration);

    action = m_actionspace.get_action(10);
    EXPECT_NEAR(45.4422, action->start_pos.x(), 0.0001);
    EXPECT_NEAR(175.031, action->start_pos.y(), 0.001);
    EXPECT_NEAR(0.429204, action->theta, 0.000001);
    EXPECT_EQ(0, action->speed);
    EXPECT_EQ(2.5, action->duration);

    action = m_actionspace.get_action(20);
    EXPECT_NEAR(75.0312, action->start_pos.x(), 0.0001);
    EXPECT_NEAR(254.558, action->start_pos.y(), 0.001);
    EXPECT_NEAR(5.14159, action->theta, 0.00001);
    EXPECT_EQ(0, action->speed);
    EXPECT_EQ(5, action->duration);

    action = m_actionspace.get_action(30);
    EXPECT_NEAR(154.558, action->start_pos.x(), 0.001);
    EXPECT_NEAR(224.969, action->start_pos.y(), 0.001);
    EXPECT_NEAR(3.57079, action->theta, 0.00001);
    EXPECT_EQ(2.5, action->speed);
    EXPECT_EQ(0, action->duration);
}

TEST_F(SimpleOOActionFixture, ActionSimilarityTest) {
    EXPECT_EQ(2.5, m_actionspace.action_similarity(0, 1));
    EXPECT_EQ(2.5, m_actionspace.action_similarity(0, 3));
    EXPECT_NEAR(84.867352, m_actionspace.action_similarity(0, 9), 0.00001);
    EXPECT_NEAR(84.940965, m_actionspace.action_similarity(5, 16), 0.00001);
}

TEST_F(SimpleOOActionFixture, OORExceptionTest) {
    EXPECT_THROW(m_actionspace.action_similarity(0, -1), std::out_of_range);
    EXPECT_THROW(m_actionspace.action_similarity(0, 36), std::out_of_range);
    EXPECT_THROW(m_actionspace.action_similarity(-1, 0), std::out_of_range);
    EXPECT_THROW(m_actionspace.action_similarity(36, 0), std::out_of_range);
    EXPECT_THROW(m_actionspace.get_action(-1), std::out_of_range);
    EXPECT_THROW(m_actionspace.get_action(36), std::out_of_range);
}

TEST_F(SimpleOOActionFixture, PublishActionTest) {
    Publish(5);
    WaitForMessage();
    libcozmo::ObjectOrientedAction* action = get_action_msg();
    ASSERT_EQ(1, subscribe_count);
    EXPECT_NEAR(124.969, action->x, 0.001);
    EXPECT_NEAR(145.442, action->y, 0.001);
    EXPECT_EQ(2, action->theta);
    EXPECT_EQ(2.5, action->speed);
    EXPECT_EQ(5, action->duration);

    Publish(10);
    WaitForMessage();
    action = get_action_msg();
    ASSERT_EQ(2, subscribe_count);
    EXPECT_NEAR(45.4422, action->x, 0.0001);
    EXPECT_NEAR(175.031, action->y, 0.001);
    EXPECT_NEAR(0.429204, action->theta, 0.000001);
    EXPECT_EQ(0, action->speed);
    EXPECT_EQ(2.5, action->duration);
}

TEST_F(SimpleOOActionFixture2, ActionGenerationTest) {
    libcozmo::actionspace::ObjectOrientedActionSpace::ObjectOrientedAction* action = m_actionspace.get_action(5);
    EXPECT_NEAR(124.969, action->start_pos.x(), 0.001);
    EXPECT_NEAR(145.442, action->start_pos.y(), 0.001);
    EXPECT_EQ(2, action->theta);
    EXPECT_EQ(2.5, action->speed);
    EXPECT_EQ(5, action->duration);

    action = m_actionspace.get_action(10);
    EXPECT_NEAR(45.4422, action->start_pos.x(), 0.0001);
    EXPECT_NEAR(175.031, action->start_pos.y(), 0.001);
    EXPECT_NEAR(0.429204, action->theta, 0.000001);
    EXPECT_EQ(0, action->speed);
    EXPECT_EQ(2.5, action->duration);

    action = m_actionspace.get_action(20);
    EXPECT_NEAR(75.0312, action->start_pos.x(), 0.0001);
    EXPECT_NEAR(254.558, action->start_pos.y(), 0.001);
    EXPECT_NEAR(5.14159, action->theta, 0.00001);
    EXPECT_EQ(0, action->speed);
    EXPECT_EQ(5, action->duration);

    action = m_actionspace.get_action(30);
    EXPECT_NEAR(154.558, action->start_pos.x(), 0.001);
    EXPECT_NEAR(224.969, action->start_pos.y(), 0.001);
    EXPECT_NEAR(3.57079, action->theta, 0.00001);
    EXPECT_EQ(2.5, action->speed);
    EXPECT_EQ(0, action->duration);
}

TEST_F(SimpleOOActionFixture3, ActionGenerationTest) {
    libcozmo::actionspace::ObjectOrientedActionSpace::ObjectOrientedAction* action = m_actionspace.get_action(5);
    EXPECT_NEAR(124.969, action->start_pos.x(), 0.001);
    EXPECT_NEAR(145.442, action->start_pos.y(), 0.001);
    EXPECT_EQ(2, action->theta);
    EXPECT_EQ(2.5, action->speed);
    EXPECT_EQ(5, action->duration);

    action = m_actionspace.get_action(10);
    EXPECT_NEAR(45.4422, action->start_pos.x(), 0.0001);
    EXPECT_NEAR(175.031, action->start_pos.y(), 0.001);
    EXPECT_NEAR(0.429204, action->theta, 0.000001);
    EXPECT_EQ(0, action->speed);
    EXPECT_EQ(2.5, action->duration);

    action = m_actionspace.get_action(20);
    EXPECT_NEAR(75.0312, action->start_pos.x(), 0.0001);
    EXPECT_NEAR(254.558, action->start_pos.y(), 0.001);
    EXPECT_NEAR(5.14159, action->theta, 0.00001);
    EXPECT_EQ(0, action->speed);
    EXPECT_EQ(5, action->duration);

    action = m_actionspace.get_action(30);
    EXPECT_NEAR(154.558, action->start_pos.x(), 0.001);
    EXPECT_NEAR(224.969, action->start_pos.y(), 0.001);
    EXPECT_NEAR(3.57079, action->theta, 0.00001);
    EXPECT_EQ(2.5, action->speed);
    EXPECT_EQ(0, action->duration);
}

TEST_F(ComplexOOActionFixture, ActionSpaceSizeTest) {
    EXPECT_EQ(1540, m_actionspace.size());
}

TEST_F(ComplexOOActionFixture, ActionGenerationTest) {
    libcozmo::actionspace::ObjectOrientedActionSpace::ObjectOrientedAction* action = m_actionspace.get_action(75);
    EXPECT_NEAR(88.5969, action->start_pos.x(), 0.0001);
    EXPECT_NEAR(128.796, action->start_pos.y(), 0.001);
    EXPECT_EQ(2, action->theta);
    EXPECT_NEAR(3.14159, action->speed, 0.00001);
    EXPECT_NEAR(2.44645, action->duration, 0.00001);

    action = m_actionspace.get_action(150);
    EXPECT_NEAR(106.783, action->start_pos.x(), 0.001);
    EXPECT_NEAR(137.119, action->start_pos.y(), 0.001);
    EXPECT_EQ(2, action->theta);
    EXPECT_NEAR(3.14159, action->speed, 0.00001);
    EXPECT_NEAR(1.9028, action->duration, 0.0001);

    action = m_actionspace.get_action(225);
    EXPECT_NEAR(124.969, action->start_pos.x(), 0.001);
    EXPECT_NEAR(145.442, action->start_pos.y(), 0.001);
    EXPECT_EQ(2, action->theta);
    EXPECT_NEAR(3.14159, action->speed, 0.00001);
    EXPECT_NEAR(1.35914, action->duration, 0.00001);

    action = m_actionspace.get_action(300);
    EXPECT_NEAR(143.155, action->start_pos.x(), 0.001);
    EXPECT_NEAR(153.765, action->start_pos.y(), 0.001);
    EXPECT_EQ(2, action->theta);
    EXPECT_NEAR(3.14159, action->speed, 0.00001);
    EXPECT_NEAR(0.815485, action->duration, 0.000001);

    action = m_actionspace.get_action(375);
    EXPECT_NEAR(161.341, action->start_pos.x(), 0.001);
    EXPECT_NEAR(162.088, action->start_pos.y(), 0.001);
    EXPECT_EQ(2, action->theta);
    EXPECT_NEAR(3.14159, action->speed, 0.00001);
    EXPECT_NEAR(0.271828, action->duration, 0.000001);

    action = m_actionspace.get_action(450);
    EXPECT_NEAR(28.7963, action->start_pos.x(), 0.0001);
    EXPECT_NEAR(211.403, action->start_pos.y(), 0.001);
    EXPECT_NEAR(0.429204, action->theta, 0.000001);
    EXPECT_NEAR(2.0944, action->speed, 0.0001);
    EXPECT_NEAR(2.71828, action->duration, 0.00001);

    action = m_actionspace.get_action(525);
    EXPECT_NEAR(37.1192, action->start_pos.x(), 0.0001);
    EXPECT_NEAR(193.217, action->start_pos.y(), 0.001);
    EXPECT_NEAR(0.429204, action->theta, 0.000001);
    EXPECT_NEAR(2.0944, action->speed, 0.0001);
    EXPECT_NEAR(2.17463, action->duration, 0.00001);

    action = m_actionspace.get_action(600);
    EXPECT_NEAR(45.4422, action->start_pos.x(), 0.0001);
    EXPECT_NEAR(175.031, action->start_pos.y(), 0.001);
    EXPECT_NEAR(0.429204, action->theta, 0.000001);
    EXPECT_NEAR(2.0944, action->speed, 0.0001);
    EXPECT_NEAR(1.63097, action->duration, 0.00001);

    action = m_actionspace.get_action(675);
    EXPECT_NEAR(53.7651, action->start_pos.x(), 0.0001);
    EXPECT_NEAR(156.845, action->start_pos.y(), 0.001);
    EXPECT_NEAR(0.429204, action->theta, 0.000001);
    EXPECT_NEAR(2.0944, action->speed, 0.0001);
    EXPECT_NEAR(1.08731, action->duration, 0.00001);

    action = m_actionspace.get_action(750);
    EXPECT_NEAR(62.088, action->start_pos.x(), 0.0001);
    EXPECT_NEAR(138.659, action->start_pos.y(), 0.001);
    EXPECT_NEAR(0.429204, action->theta, 0.000001);
    EXPECT_NEAR(2.0944, action->speed, 0.0001);
    EXPECT_NEAR(0.543656, action->duration, 0.00001);
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
