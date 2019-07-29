#include "statespace/statespace.hpp"
#include <gtest/gtest.h>

class myTestFixture1: public ::testing::Test { 
    public: 
    //     myTestFixture1( ) : \
    //     width(10),
    //     height(10),
    //     res(10),
    //     num_theta_vals(4),
    //     myStateSpace(width, height, res, num_theta_vals) { 
    //     } 

    //     void SetUp( ) { 
    //         const int& x_start = 0;
    //         const int& y_start = 0;
    //         const int& w_start = 0;
    //         const int& x_goal = 5;
    //         const int& y_goal = 5;
    //         const int& w_goal = 3;
    //         myStateSpace.set_start_state(x_start, y_start, w_start);
    //         myStateSpace.set_goal_state(x_goal, y_goal, w_goal);
    //         myStateSpace.create_new_state(3, 2, 1);
    //         myStateSpace.create_new_state(1, 3, 4);
    //     }

    //     void TearDown( ) { 
    //     }

    //     ~myTestFixture1( )  { 
    //     }
    // const int& width;
    // const int& height;
    // const int& res;
    // const int& num_theta_vals;
    // libcozmo::statespace::Statespace myStateSpace;
};

// // Check state_id conversion
// TEST_F (myTestFixture1, UnitTest1) { 
//     int expected = 123;
//     EXPECT_EQ(expected, myStateSpace.get_state_id(3, 2, 1));
// }

// // Check distnguishing valid_state 
// TEST_F (myTestFixture1, UnitTest2) { 
//     EXPECT_EQ(false, myStateSpace.is_valid_state(15, 2, 1));
//     EXPECT_EQ(false, myStateSpace.is_valid_state(-1, -2, -5));
//     EXPECT_EQ(false, myStateSpace.is_valid_state(0, 150, -1));
//     EXPECT_EQ(true, myStateSpace.is_valid_state(1, 2, 1));
//     EXPECT_EQ(true, myStateSpace.is_valid_state(9, 8, 2));
// }

// // Check discrete --> continuous pose
// TEST_F (myTestFixture1, UnitTest3) { 
//     //EXPECT_EQ(0, myStateSpace.continuous_pose_to_discrete(0.25));
//     Eigen::Vector3d pose = myStateSpace.discrete_pose_to_continuous(4, 2, 3);
//     EXPECT_EQ(4, pose[0]);
//     EXPECT_EQ(2, pose[1]);
//     double angle = 3 / 2.0 * M_PI;
//     EXPECT_EQ(angle, pose[2]);
// }

// // Check continuous --> discrete pose (double input)
// TEST_F (myTestFixture1, UnitTest4) { 
//     Eigen::Vector3i pose = myStateSpace.continuous_pose_to_discrete(5.4, 2.0, M_PI);
//     EXPECT_EQ(5, pose[0]);
//     EXPECT_EQ(2, pose[1]);
//     double angle = 2;
//     EXPECT_EQ(angle, pose[2]);
// }

// // Check continuous --> discrete pose (SE2 input)
// TEST_F (myTestFixture1, UnitTest5) { 
//     aikido::statespace::SE2::State continuous_state;
//     Eigen::Isometry2d t = Eigen::Isometry2d::Identity();
//     const Eigen::Rotation2D<double> rot(M_PI);
//     t.linear() = rot.toRotationMatrix();
//     Eigen::Vector2d trans;
//     trans << 5.4, 2.0;
//     t.translation() = trans;
//     continuous_state.setIsometry(t);
//     Eigen::Vector3i pose = myStateSpace.continuous_pose_to_discrete(continuous_state);
//     EXPECT_EQ(5, pose[0]);
//     EXPECT_EQ(2, pose[1]);
//     double angle = 2;
//     EXPECT_EQ(angle, pose[2]);
// }

// // Check state_id to pose conversion
// TEST_F (myTestFixture1, UnitTest6) { 
//     Eigen::Vector3i pose;
//     myStateSpace.get_coord_from_state_id(123, pose);
//     EXPECT_EQ(3, pose[0]);
//     EXPECT_EQ(2, pose[1]);
//     EXPECT_EQ(1, pose[2]);
// }

// // Check path generation 
// TEST_F (myTestFixture1, UnitTest7) {

//     int myints[] = {123, 259};
//     std::vector<int> state_ids;
//     for (int i = 0; i < 2; i++) 
//         state_ids.push_back(myints[i]); 
//     std::vector<Eigen::Vector3i> path_coordinates;
//     std::vector<Eigen::Vector3i> expected;
//     expected.push_back(Eigen::Vector3i(3, 2, 1));
//     expected.push_back(Eigen::Vector3i(9, 5, 2));
//     myStateSpace.get_path_coordinates(state_ids, &path_coordinates);
//     EXPECT_EQ(expected[0][0], path_coordinates[0][0]);
//     EXPECT_EQ(expected[0][1], path_coordinates[0][1]);
//     EXPECT_EQ(expected[0][2], path_coordinates[0][2]);
//     EXPECT_EQ(expected[1][0], path_coordinates[1][0]);
//     EXPECT_EQ(expected[1][1], path_coordinates[1][1]);
//     EXPECT_EQ(expected[1][2], path_coordinates[1][2]);
// }

// // Check create_state and get_or_create_state
// TEST_F (myTestFixture1, UnitTest8) {

//     // since 2 states were added in intialization,
//     // map should already have two states
//     EXPECT_EQ(2, myStateSpace.get_map_size());
//     // make sure the states are indeed correct
//     const int id = 123;
//     std::unordered_map<int,Eigen::Vector3i>::const_iterator itr;
//     myStateSpace.get_state(id, itr);
//     Eigen::Vector3i state = itr->second;
    
//     EXPECT_EQ(3, state.x());
//     EXPECT_EQ(2, state.y());
//     EXPECT_EQ(1, state.z());

//     // try to add existing state
//     myStateSpace.get_or_create_new_state(1, 3, 4);
//     EXPECT_EQ(2, myStateSpace.get_map_size());
// }

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

