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

#include "model/DeterministicModel.hpp"
#include <sstream>
#include <iostream>
#include "aikido/statespace/SE2.hpp"
#include "utils/utils.hpp"

namespace libcozmo {
namespace model {

void LatticeGraphModel::get_successor_state(
    const WorldRepresentation::ModelInput& model_input_,
    const aikido::statespace::StateSpace::State& in_,
    aikido::statespace::StateSpace::State* out_) {
        
    // Casting to access attributes specific to LatticeGraph
    const ModelInput model_input =
        static_cast<const ModelInput&>(model_input_);
    const aikido::statespace::SE2::State in_state =
        static_cast<const aikido::statespace::SE2::State&>(in_);
    aikido::statespace::SE2::State succesor;
    
    // Get Isometry from current State
    auto curr_state_isometry = in_.getIsometry();

    // Apply actions to current state to get output state
    const double x = curr_state_isometry.translation()[0]
        + model_input.getX();
    const double y = curr_state_isometry.translation()[1]
        + model_input.getY();
    Eigen::Isometry2d t = Eigen::Isometry2d::Identity();
    const Eigen::Rotation2D<double> rot(model_input.getTheta());
    t.linear() = rot.toRotationMatrix();
    t.translation() = Eigen::Vector2d(x, y);
    succesor.setIsometry(t);
    *out_ = successor;
}

}  // namespace model
}  // namespace libcozmo
