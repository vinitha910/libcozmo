////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019, Vinitha Ranganeni, Brian Lee
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

#ifndef INCLUDE_MDP_MDP_H_
#define INCLUDE_MDP_MDP_H_

// #include <boost/functional/hash.hpp>
#include <Eigen/Dense>
#include <vector>
// #include <utility>
// #include <unordered_map>
// #include "aikido/distance/SE2.hpp"

namespace libcozmo {
namespace mdp {

struct Action {
  Eigen::Vector2d velocity;
  Eigen::Vector2d position;
  double duration;
};

// This class implements the Markov Decision process
class MDP {
 public:
    // Constructor

    // \param discount_factor The discounting factor
    MDP(const int& discount_factor) : \ 
    gamma(discount_factor){}

    ~MDP() {}

    // Selects and executes a function
    // records the change in state given action

    void explore() const;

 private:

    std::vector<Action> generate_actions() const;

    // The discount factor
    double gamma;
};

}  // namespace MDP
}  // namespace libcozmo

#endif  // INCLUDE_MDP_MDP_H_
