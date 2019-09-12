////////////////////////////////////////////////////////////////////////////////
<<<<<<< HEAD
// Copyright (c) 2019, Vinitha Ranganeni, Brian Lee
=======
// Copyright (c) 2019, Brian Lee, Vinitha Ranganeni
>>>>>>> a7fffa012325937d70466fdf4b5b0938ff1a8d82
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
#include "statespace/SE2.hpp"
#include "distance/SE2.hpp"
#include "distance/translation.hpp"
#include "distance/orientation.hpp"

namespace libcozmo {
namespace distance {
namespace test {

<<<<<<< HEAD
class DistanceTest: public ::testing::Test {
 public:
    DistanceTest() : \
        m_statespace(statespace::SE2(0.1, 8)),
        m_se2(distance::SE2(&m_statespace)),
        m_trans(distance::Translation(&m_statespace)),
        m_orient(distance::Orientation(&m_statespace)) {}

    void SetUp() {
        m_id_1 =
            m_statespace.get_or_create_state(statespace::SE2::State(0, 0, 0));
        m_id_2 =
            m_statespace.get_or_create_state(statespace::SE2::State(1, 3, 1));
    }

    ~DistanceTest() {}

    statespace::SE2 m_statespace;
    distance::SE2 m_se2;
    distance::Translation m_trans;
    distance::Orientation m_orient;
    int m_id_1;
    int m_id_2;
};

TEST_F(DistanceTest, TestSE2) {
    // Cheking SE2 distance calculated correctly
    EXPECT_NEAR(
        sqrt(pow(0.1,2) + pow(0.3,2) + pow(M_PI/4, 2)), 
        m_se2.get_distance(
            *m_statespace.get_state(m_id_1),
            *m_statespace.get_state(m_id_2)),
        0.01);
}

TEST_F(DistanceTest, TestTranslation) {
    // Cheking translation calculated correctly
    EXPECT_NEAR(
        sqrt(pow(0.1,2) + pow(0.3,2)), 
        m_trans.get_distance(
            *m_statespace.get_state(m_id_1),
            *m_statespace.get_state(m_id_2)),
        0.01);
}

TEST_F(DistanceTest, TestOrientation) {
    // Cheking orientation calculated correctly
    EXPECT_NEAR(
        M_PI/4, 
        m_orient.get_distance(
            *m_statespace.get_state(m_id_1),
            *m_statespace.get_state(m_id_2)),
=======
TEST(DistanceTest, TestSE2) {
    // Checking SE2 distance calculated correctly
    const auto statespace = std::make_shared<statespace::SE2>(0.1, 8);
    distance::SE2 se2 = distance::SE2(statespace);
    EXPECT_NEAR(
        sqrt(pow(0.1, 2) + pow(0.3, 2) + pow(M_PI / 2, 2)),
        se2.get_distance(
            statespace::SE2::State(0, 0, 1), statespace::SE2::State(1, 3, 3)),
        0.01);
}

TEST(DistanceTest, TestTranslation) {
    // Checking translation calculated correctly
    const auto statespace = std::make_shared<statespace::SE2>(0.1, 8);
    distance::Translation trans = distance::Translation(statespace);
    EXPECT_NEAR(
        sqrt(pow(0.1, 2) + pow(0.3, 2)),
        trans.get_distance(
            statespace::SE2::State(0, 0, 1),
            statespace::SE2::State(1, 3, 3)),
        0.01);
}

TEST(DistanceTest, TestOrientation) {
    // Checking orientation calculated correctly
    const auto statespace = std::make_shared<statespace::SE2>(0.1, 8);
    distance::Orientation orient = distance::Orientation(statespace);
    EXPECT_NEAR(
        M_PI / 2,
        orient.get_distance(
            statespace::SE2::State(0, 0, 1),
            statespace::SE2::State(1, 3, 3)),
>>>>>>> a7fffa012325937d70466fdf4b5b0938ff1a8d82
        0.01);
}

}  // namespace test
<<<<<<< HEAD
}  // namspace distance
=======
}  // namespace distance
>>>>>>> a7fffa012325937d70466fdf4b5b0938ff1a8d82
}  // namespace libcozmo

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
<<<<<<< HEAD
}
=======
}
>>>>>>> a7fffa012325937d70466fdf4b5b0938ff1a8d82
