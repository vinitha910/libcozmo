////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019,  Brian Lee, Vinitha Ranganeni
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
#include "utils/utils.hpp"

namespace libcozmo {
namespace utils {
namespace test {

/// Check that Euclidean distance with double is correct
TEST(TestSuite, EuclideanDoubleTest) {
    std::vector<double> vector1{10.0, 16.4, 5.7};
    std::vector<double> vector2{100.0, 15.2, 0.0};
    double result = utils::euclidean_distance(vector1, vector2);
    EXPECT_NEAR(90.1883, result, 0.0001);
}

/// Check that Euclidean distance with int is correct
TEST(TestSuite, EuclideanIntTest) {
    std::vector<int> vector1{10, 16, 5};
    std::vector<int> vector2{100, 15, 0};
    double result = utils::euclidean_distance(vector1, vector2);
    EXPECT_NEAR(90.1443, result, 0.0001);
}

/// Check that Euclidean distance with float is correct
TEST(TestSuite, EuclideanFloatTest) {
    std::vector<float> vector1{-16.75, 5.51, 0.49};
    std::vector<float> vector2{100.0, 15.2, 0.00};
    double result = utils::euclidean_distance(vector1, vector2);
    EXPECT_NEAR(117.1525, result, 0.0001);
}

}  // namespace test
}  // namespace utils
}  // namespace libcozmo

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
