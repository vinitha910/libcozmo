#include "statespace/statespace.hpp"
#include <gtest/gtest.h>

class myTestFixture1: public ::testing::Test { 
    public: 
        myTestFixture1( ) { 
            // initialization code here
            
                const int& width = 10;
                const int& height = 10;
                const double& res = 1;
                const int& num_theta_vals = 4;

                libcozmo::statespace::Statespace myStateSpace(width, height, res, num_theta_vals); 
        } 

        void SetUp( ) { 
            // code here will execute just before the test ensues 
            // Add some example states
            const int& x_start = 0;
            const int& y_start = 0;
            const int& w_start = 0;
            const int& x_goal = 5;
            const int& y_goal = 5;
            const int& z_goal = 3;
            mystateSpace.set_start_state(x_start, y_start, w_start);
            mystateSpace.set_goal_state(x_goal, y_goal, w_goal);
            mystateSpace.create_new_state()
            mystateSpace.create_new_state()
            //    get_or_create_new_state
            //    get_or_create_new_state
            //    get_path_coordinates
            //    get_state_id
            //    get_coord_from_state_id
            //    normalize_angle_rad
            //    discrete_angle_to_continuous
            //    continuous_angle_to_discrete
            //    continuous_position_to_discrete
            //    discrete_position_to_continuous
            //    discrete_pose_to_continuous
            //    continuous_pose_to_discrete
            //    continuous_pose_to_discrete
            //    is_valid_state //correct state
                // get_distance
        }

        void TearDown( ) { 
            // code here will be called just after the test completes
            // ok to through exceptions from here if need be
            //    is_valid_state //wrong state
        }

        // ~myTestFixture1( )  { 
        //     // cleanup any pending stuff, but no exceptions allowed
        // }
   // put in any custom data members that you need 
   //the member variables for statespace here
};

// TEST (EqualityTest, ZeroShouldEqualZero)
// {
//     ASSERT_EQ(0, 0);
// }

// Declare test
TEST_F (myTestFixture1, UnitTest1) { 
    ASSERT_EQ(0, 0);
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

