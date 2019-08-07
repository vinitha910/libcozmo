#include <gtest/gtest.h>
#include "action_space/cozmo_actions.hpp"

class myTestFixture1: public ::testing::test { 
public: 
   myTestFixture1() : \
   myActionSpace() { 
       // initialization code here
   } 

   void SetUp( ) { 
       // code here will execute just before the test ensues 
       myActionSpace.generate_actions(10, 100, 5, 10, 100, 5, 1, 5, 5);
   }

   void TearDown( ) { 
       // code here will be called just after the test completes
       // ok to through exceptions from here if need be
   }

   ~myTestFixture1( )  { 
       // cleanup any pending stuff, but no exceptions allowed
       GenericActionSpace myActionSpace;
   }

   // put in any custom data members that you need 
};

TEST_F(myTestFixture1, UnitTest1) {
    EXPECT_EQ(176, myActionSpace.get_action_space().size());
}
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

/**
 * Things to check
 * ----------------
 *  Gen actions ->
 *  -1, 0, 1 means [neg, 0, pos]
 *  -1, 0, 1 for linear and angular velocities; 0, 1 for durations and samples
 *
 *  lin         ang     duration    samples
 *  ---         ---     --------    -------
 *  -1 to -1    1 to 1   1 to 1       3
 *  -1 to 0
 *  0 to 1
 *  1 to 1
 *
 *  repeat for ang
 *
 *  1 to 1      1 to 1   0 to 1       3
 *
 *                                    2, 5
 *  size of action space
 *  individual values
 *
 *  OO -> check for different pose positions
 *  and check the offsets are all correct
 *
 *
 *
 *
 *
 *
 */
