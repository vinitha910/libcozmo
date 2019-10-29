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
#include "actionspace/GenericActionSpace.hpp"
#include "utils/utils.hpp"

namespace libcozmo {
namespace model {

void DeterministicModel::get_successor(
    const actionspace::ActionSpace::Action& input,
    const aikido::statespace::StateSpace::State& in_,
    aikido::statespace::StateSpace::State* out_) {

    // Casting to access attributes specific to LatticeGraph
    const actionspace::GenericActionSpace::Action& model_input =
        static_cast<const actionspace::GenericActionSpace::Action&>(input);
    const aikido::statespace::SE2::State in_state =
        static_cast<const aikido::statespace::SE2::State&>(in_);
    aikido::statespace::SE2::State* successor =
        static_cast<aikido::statespace::SE2::State*>(out_);

    // Get Isometry from current State
    auto curr_state_isometry = in_state.getIsometry();

    double angle = model_input.m_heading;
    double distance = model_input.m_speed * model_input.m_duration;

    const double delta_x = distance * cos(angle);
    const double delta_y = distance * sin(angle);

    // Apply actions to current state to get output state
    double x = curr_state_isometry.translation()[0] + delta_x;
    double y = curr_state_isometry.translation()[1] + delta_y;
    Eigen::Isometry2d t = Eigen::Isometry2d::Identity();
    const Eigen::Rotation2D<double> rot(angle);
    t.linear() = rot.toRotationMatrix();
    t.translation() = Eigen::Vector2d(x, y);
    successor->setIsometry(t);
    // *out_ = successor;
}

}  // namespace model
}  // namespace libcozmo
