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

class GPRModelTest: public ::testing::Test {
 public:
    GPRModelTest() :
        m_model(create_model()) {}

    ~GPRModelTest() {}

    GPRModel create_model() {
        auto framework =
            std::make_shared<ScikitLearnFramework>("SampleGPRModel.pkl");
        auto statespace = std::make_shared<aikido::statespace::SE2>();
        return GPRModel(framework, statespace);
    }

    GPRModel m_model;
};

TEST_F(GPRModelTest, ModelInferenceTest) {
    GPRModel::ModelInput input =
        GPRModel::ModelInput(30.0, -1.0, 1.0, Eigen::Vector2d(0, 1));
    GPRModel::ModelOutput output;
    m_model.inference(input, &output);
    EXPECT_NEAR(0.0978534, output.distance, 0.001);
    EXPECT_NEAR(-0.0001391, output.dtheta, 0.001);
}

TEST_F(GPRModelTest, GetPredictedStateTest) {
    GPRModel::ModelInput input =
        GPRModel::ModelInput(30.0, -1.0, 1.0, Eigen::Vector2d(3.5, 5.3));

    aikido::statespace::SE2::State in;
    Eigen::Isometry2d t = Eigen::Isometry2d::Identity();
    const Eigen::Rotation2D<double> rot(M_PI/4);
    t.linear() = rot.toRotationMatrix();
    t.translation() = Eigen::Vector2d(2.0, 3.0);
    in.setIsometry(t);

    aikido::statespace::SE2::State out;
    m_model.predict_state(input, in, &out);

    const auto transform = out.getIsometry();
    Eigen::Rotation2Dd rotation = Eigen::Rotation2Dd::Identity();
    rotation.fromRotationMatrix(transform.rotation());

    EXPECT_NEAR(2.05392324, transform.translation().x(), 0.001);
    EXPECT_NEAR(3.08165519, transform.translation().y(), 0.001);
    EXPECT_NEAR(0.78525906, rotation.angle(), 0.001);
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
