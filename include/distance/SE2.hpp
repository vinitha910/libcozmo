//////////////////////////////////////////////////////////////////////////////
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

#ifndef LIBCOZMO_DISTANCE_SE2_HPP_
#define LIBCOZMO_DISTANCE_SE2_HPP_

#include <memory>
#include "statespace/SE2.hpp"
#include "distance/distance.hpp"

namespace libcozmo {
namespace distance {

/// SE2 Distance Metric
///
/// This class implements a distace metric between SE2 transformations.
class SE2 : public Distance {
 public:
    /// Constructs metric with given statespace
    ///
    /// \param statespace The statespace the metric operates in
    explicit SE2(const std::shared_ptr<statespace::SE2> statespace);

    ~SE2() {}

    /// Documentation inherited
    double get_distance(
        const statespace::StateSpace::State& _state_1,
        const statespace::StateSpace::State& _state_2) const override;

 private:
    const std::shared_ptr<statespace::SE2> m_statespace;
};

}  // namespace distance
}  // namespace libcozmo

#endif  // LIBCOZMO_DISTANCE_SE2_HPP_
