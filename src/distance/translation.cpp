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

#include <cmath>
#include "distance/translation.hpp"
#include "statespace/SE2.hpp"

namespace libcozmo {
namespace distance {

	double Translation::get_distance(
		const libcozmo::statespace::StateSpace::State& _state_1,
    	const libcozmo::statespace::StateSpace::State& _state_2) const {

		aikido::statespace::SE2::State continuous_state_1;
		m_statespace->
			discrete_state_to_continuous(_state_1, &continuous_state_1);
		aikido::statespace::SE2::State continuous_state_2;
		m_statespace->
			discrete_state_to_continuous(_state_2, &continuous_state_2);

		Eigen::Vector2d position_1(
			continuous_state_1.getIsometry().translation());
		Eigen::Vector2d position_2(
			continuous_state_2.getIsometry().translation());

		return sqrt(pow(position_1[0] - position_2[0], 2)
			+ pow(position_1[1] - position_2[1], 2));
	}

}  // namespace distance
}  // namespace libcozmo