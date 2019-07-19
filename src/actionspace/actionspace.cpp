////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019, Vinitha Ranganeni
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

#include "src/statespace/statespace.h"
#include <Eigen>
#include <assert.h>
#include <cmath>
#include <utility>
#include "aikido/distance/SE2.hpp"
#include "aikido/statespace/SE2.hpp"
#include "cozmo_description/cozmo.hpp"

namespace libcozmo {
namespace actionspace {
//using aikido::statespace::SE2;
using libcozmo::actionspace::Action;

GenericActionSpace::GenericActionSpace(const float& lin_min,
                                       const float& lin_max,
                                       const int& lin_samples,
                                       const float& ang_min,
                                       const float& ang_max,
                                       const int& ang_samples,
                                       const float& dur_min,
                                       const float& dur_max,
                                       const int& dur_samples,
                                       const std::string& mesh_dir) {
    m_lin_min = lin_min;
}

GenericActionSpace::applyAction(const int& actionIdx) {
    //
    Action act = m_actions[actionIdx];

    //get linear velocity, angular velocity, and duration
    float linvel = act.m_linvel;
    float angvel = act.m_angvel;
    float duration = act.m_duration;
    float left = linvel + angvel;
    float right = linvel - angvel;
    
    //apply given action to cozmo
    //cozmo.drive_wheels(left_wheel, right_wheel, duration=action.duration)
    
}

GenericActionSpace::generatAction() {

    // randomly generate different values of 
    // linear, angular velocity and duration
    // within the class's threshold set during
    // intialization

    

}

}  // namespace statespace
}  // namespace libcozmo

