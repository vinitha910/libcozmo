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

#include <gtest/gtest.h>
#include "model/GPRModel.hpp"
#include "model/ScikitLearnFramework.hpp"
#include <Python.h>

namespace libcozmo {
namespace model {
namespace test {

class GPRModelTest: public ::testing::Test {
 public:
    GPRModelTest() :
        m_framework(std::make_shared<ScikitLearnModel>("/home/vinitha/workspaces/curiosity_ws/src/curiosity_project/python/models/novel_uninformed/angle_regressor_12.pkl")),
        m_model(m_framework) {
    }

    ~GPRModelTest() { }

    std::shared_ptr<ScikitLearnModel> m_framework;
    GPRModel m_model;
};

/// Check that model without anything loaded simply handles nullptr
TEST_F(GPRModelTest, LoadModelTest) {
    GPRModel::ModelInput input =
        GPRModel::ModelInput(10, 0.75, 1);
    GPRModel::ModelOutput output;
    m_model.inference(input, &output);
    EXPECT_NEAR(0.00012, output.distance, 0.01);
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