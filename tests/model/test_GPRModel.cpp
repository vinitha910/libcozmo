////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019,  Vinitha Ranganeni
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

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <stdexcept>
#include <Python.h>
#include "model/GPRModel.hpp"
#include "model/ScikitLearnFramework.hpp"
#include "statespace/SE2.hpp"

namespace libcozmo {
namespace model {
namespace test {

// Check model inference is correct
TEST(GPRModelTest, LoadModelTest) {
    Py_Initialize();
    auto framework = std::make_shared<ScikitLearnFramework>("SampleGPRModel.pkl");
    auto statespace = std::make_shared<aikido::statespace::SE2>();
    auto model = GPRModel(framework, statespace);

    GPRModel::ModelInput input = 
        GPRModel::ModelInput(30.0, -1.0, 1.0, Eigen::Vector2d(0,1));
    GPRModel::ModelOutput output;
    model.inference(input, &output);
    EXPECT_NEAR(0.0978534, output.distance, 0.001);
    EXPECT_NEAR(-0.0001391, output.dtheta, 0.001);
    Py_Finalize();
}

TEST(GPRModelTest, IncorrectLoadModelTest) {
    Py_Initialize();
    try {
        auto framework = ScikitLearnFramework("");
    } catch(std::invalid_argument const& error) {
        EXPECT_EQ(error.what(), 
            std::string("[ScikitLearnFramework] Invalid model_path"));
    }
    Py_Finalize();
}

// Todo test predict state function
// TEST(GPRModelTest, GetPredictedStateTest) {
//     Py_Initialize();
//     auto framework = std::make_shared<ScikitLearnFramework>("SampleGPRModel.pkl");
//     auto statespace = std::make_shared<aikido::statespace::SE2>();
//     auto model = GPRModel(framework, statespace);

//     GPRModel::ModelInput input = 
//         GPRModel::ModelInput(30.0, -1.0, 1.0, Eigen::Vector2d(0,1));


//     Py_Finalize();
// }

}  // namespace test
}  // namespace model
}  // namespace libcozmo

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    const auto results = RUN_ALL_TESTS();
    return results;
}