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

namespace libcozmo {
namespace model {
namespace test {

class GPRModelTest: public ::testing::Test {
 public:
    GPRModelTest() :
        m_model(create_model()) {}

    ~GPRModelTest() {}

    GPRModel create_model() {
        auto framework =
            std::make_shared<ScikitLearnFramework>("/home/joonh/cozmo_ws/src/libcozmo/tests/model/SampleGPRModel.pkl");
        return GPRModel(framework);
    }

    GPRModel m_model;
};


TEST_F(GPRModelTest, ModelPredictionTest) {
    Eigen::VectorXd model_input(4);
    model_input << 30.0, 1.0, -1.0, 0;
    Eigen::VectorXd state_input(3);
    state_input << 1, 1, 0;
    Eigen::VectorXd state_output(3);

    EXPECT_TRUE(m_model.predict_state(model_input, state_input, &state_output));

    double x = 1 + 0.0978534 * cos(-0.0001391);
    double y = 1 + 0.0978534 * sin(-0.0001391);
    double theta = -0.0001391;

    EXPECT_NEAR(x, state_output[0], 0.001);
    EXPECT_NEAR(y, state_output[1], 0.001);
    EXPECT_NEAR(theta, state_output[2], 0.001);
}

}  // namespace test
}  // namespace model
}  // namespace libcozmo

int main(int argc, char **argv) {
    Py_Initialize();
    ::testing::InitGoogleTest(&argc, argv);
    const auto results = RUN_ALL_TESTS();
    Py_Finalize();
    return results;
}
