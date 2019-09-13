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
#include "model/DeterministicModel.hpp"

namespace AS = libcozmo::actionspace;

namespace libcozmo {
namespace model {
namespace test {

class DeterministicModelTest: public ::testing::Test {
 public:
    DeterministicModelTest() :\
    	m_type(),
    	m_model() {}

    void SetUp() {}

    void TearDown() {}

    ~DeterministicModelTest() {}

    bool load_model() {
    	return m_model.load_model(&m_type);
    }

    DeterministicModel m_model;
    DeterministicModel::ModelType m_type;
};

/// Check that model without anything loaded simply handles nullptr,
/// and that Lin Reg weights & bias are updated correctly
TEST_F(DeterministicModelTest, LoadModelTest) {
	DeterministicModel::ModelInput input = DeterministicModel::ModelInput(
		AS::GenericActionSpace::Action(0, 0, 0));
	EXPECT_TRUE(m_model.get_prediction(input) == nullptr);
	EXPECT_TRUE(load_model());
}

/// Check that model outputs correct delta values for each action
TEST_F(DeterministicModelTest, GetPredictionTest) {
	load_model();
	DeterministicModel::ModelInput input1 = DeterministicModel::ModelInput(
		AS::GenericActionSpace::Action(0, 0, 0));
	DeterministicModel::ModelInput input2 = DeterministicModel::ModelInput(
		AS::GenericActionSpace::Action(50, 1, M_PI / 2.0));
	DeterministicModel::ModelInput input3 = DeterministicModel::ModelInput(
		AS::GenericActionSpace::Action(-50, 1, M_PI));

	DeterministicModel::ModelOutput output1 = 
		*static_cast<DeterministicModel::ModelOutput*>(
			m_model.get_prediction(input1));
	DeterministicModel::ModelOutput output2 = 
		*static_cast<DeterministicModel::ModelOutput*>(
			m_model.get_prediction(input2));
	DeterministicModel::ModelOutput output3 = 
		*static_cast<DeterministicModel::ModelOutput*>(
			m_model.get_prediction(input3));

	Eigen::Vector3d result1(output1.getX(), output1.getY(), output1.getTheta());

	EXPECT_NEAR(output1.getX(), 0, 0.001);
	EXPECT_NEAR(output1.getY(), 0, 0.001);
	EXPECT_NEAR(output1.getTheta(), 0, 0.001);
	EXPECT_NEAR(output2.getX(), 0, 0.001);
	EXPECT_NEAR(output2.getY(), 50, 0.001);
	EXPECT_NEAR(output2.getTheta(), M_PI / 2.0, 0.001);
	EXPECT_NEAR(output3.getX(), 50, 0.001);
	EXPECT_NEAR(output3.getY(), 0, 0.001);
	EXPECT_NEAR(output3.getTheta(), M_PI, 0.001);
}

}  // namespace test
}  // namespace model
}  // namespace libcozmo

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
