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

#include "distance/SE2.hpp"

namespace libcozmo {
namespace distance {

    SE2::SE2(const std::shared_ptr<statespace::SE2> statespace)
        : m_statespace(statespace) {
        if (m_statespace == nullptr) {
            throw std::invalid_argument("statespace is a nullptr.");
        }
    }

    // double SE2::get_distance(
    //     const statespace::StateSpace::State& _state_1,
    //     const statespace::StateSpace::State& _state_2) const {
    //     return m_statespace->get_distance(_state_1, _state_2);
    // }

    double SE2::get_distance(
        const statespace::StateSpace::State& _state_1,
        const statespace::StateSpace::State& _state_2) const {
        ContinuousState continuous_state_1;
        discrete_state_to_continuous(_state_1, &continuous_state_1);
        ContinuousState continuous_state_2;
        discrete_state_to_continuous(_state_2, &continuous_state_2);
        return get_distance(continuous_state_1, continuous_state_2);
    }

    double SE2::get_distance(
        const statespace::StateSpace::ContinuousState& _state_1,
        const statespace::StateSpace::ContinuousState& _state_2) const {
        const ContinuousState state_1 =
            static_cast<const ContinuousState&>(_state_1);
        const ContinuousState state_2 =
            static_cast<const ContinuousState&>(_state_2);
        Eigen::Vector3d diff;
        diff.head<2>() = state_1.vector().head<2>() - state_2.vector().head<2>();
        diff[2] = normalize_angle_rad(state_1.vector()[2] - state_2.vector()[2]);
        return diff.norm();
    }

}  // namespace distance
}  // namespace libcozmo
