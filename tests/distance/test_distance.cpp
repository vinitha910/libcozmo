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
#include "statespace/SE2.hpp"
#include "distance/SE2.hpp"
#include "distance/translation.hpp"
#include "distance/orientation.hpp"

namespace libcozmo {
namespace distance {
namespace test {

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
        0.01);
}

}  // namespace test
}  // namespace distance
}  // namespace libcozmo

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
