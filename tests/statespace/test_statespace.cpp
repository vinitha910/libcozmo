#include "statespace/statespace.hpp"
#include <gtest/gtest.h>
#include<iostream>

class myTestFixture1: public ::testing::Test { 
    public: 
        myTestFixture1( ) : \
        width(10),
        height(10),
        res(10),
        num_theta_vals(4),
        myStateSpace(width, height, res, num_theta_vals) { 
            // initialization code here

        } 

        void SetUp( ) { 
            // code here will execute just before the test ensues 
            // Add some example states
            const int& x_start = 0;
            const int& y_start = 0;
            const int& w_start = 0;
            const int& x_goal = 5;
            const int& y_goal = 5;
            const int& w_goal = 3;
            myStateSpace.set_start_state(x_start, y_start, w_start);
            myStateSpace.set_goal_state(x_goal, y_goal, w_goal);
            myStateSpace.create_new_state(1, 2, 1);
            myStateSpace.create_new_state(1, 3, 4);
        }

        void TearDown( ) { 
            // code here will be called just after the test completes
            // ok to through exceptions from here if need be
            //    is_valid_state //wrong state
        }

        ~myTestFixture1( )  { 
            // cleanup any pending stuff, but no exceptions allowed
        }
   // put in any custom data members that you need 
   //the member variables for statespace here
    const int& width;
    const int& height;
    const int& res;
    const int& num_theta_vals;
    libcozmo::statespace::Statespace myStateSpace;
};

// TEST (EqualityTest, ZeroShouldEqualZero)
// {
//     ASSERT_EQ(0, 0);
// }

// Declare tests

// Check state_id conversion
TEST_F (myTestFixture1, UnitTest1) { 
    int expected = 123;
    EXPECT_EQ(expected, myStateSpace.get_state_id(3, 2, 1));
}

// Check distnguishing valid_state 
TEST_F (myTestFixture1, UnitTest2) { 
    EXPECT_EQ(false, myStateSpace.is_valid_state(15, 2, 1));
    EXPECT_EQ(false, myStateSpace.is_valid_state(-1, -2, -5));
    EXPECT_EQ(false, myStateSpace.is_valid_state(0, 150, -1));
    EXPECT_EQ(true, myStateSpace.is_valid_state(1, 2, 1));
    EXPECT_EQ(true, myStateSpace.is_valid_state(9, 8, 2));
}

// Check discrete --> continuous pose
TEST_F (myTestFixture1, UnitTest3) { 
    //EXPECT_EQ(0, myStateSpace.continuous_pose_to_discrete(0.25));
    Eigen::Vector3d pose = myStateSpace.discrete_pose_to_continuous(4, 2, 3);
    EXPECT_EQ(5, pose[0]);
    EXPECT_EQ(3, pose[1]);
    double angle = 3 / 2.0 * M_PI;
    EXPECT_EQ(angle, pose[2]);
}

// Check continuous --> discrete pose
// TEST_F (myTestFixture1, UnitTest4) {
    //EXPECT_EQ(0.0, myStateSpace.discrete_angle_to_continuous(0));
// }

// Check state_id to pose conversion
// Eigen does some weird stuff, keeps converting .w() to diff value
// e.g. 1 --> 4
// TEST_F (myTestFixture1, UnitTest5) {
//     Eigen::Vector3i state;
//     myStateSpace.get_coord_from_state_id(123, state);
//     EXPECT_EQ(3, state.x());
//     EXPECT_EQ(2, state.y());
//     EXPECT_EQ(1, state.w());
// }



// TEST_F (myTestFixture1, UnitTest6) {
// }

//    get_or_create_new_state
            //    get_or_create_new_state
            //    get_path_coordinates
            //    discrete_angle_to_continuous
            //    continuous_position_to_discrete
            //    discrete_position_to_continuous
            //    discrete_pose_to_continuous
            //    continuous_pose_to_discrete
            //    continuous_pose_to_discrete
            //    is_valid_state //correct state
                // get_distance



int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

