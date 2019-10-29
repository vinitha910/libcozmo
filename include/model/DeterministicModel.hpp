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

#ifndef INCLUDE_MODEL_LATTICEGRAPHMODEL_HPP_
#define INCLUDE_MODEL_LATTICEGRAPHMODEL_HPP_

#include <Python.h>
#include <Eigen/Geometry>
#include <memory>
#include "model/WorldRepresentation.hpp"
#include "model/ModelFramework.hpp"
#include "model/ScikitLearnFramework.hpp"
#include "statespace/StateSpace.hpp"
#include "actionspace/GenericActionSpace.hpp"

namespace libcozmo {
namespace model {

/// Note: the model makes an assumption that input used is of generic
///       actionspace, and the outcome of an action is deterministic.
class DeterministicModel : public virtual WorldRepresentation {
 public:
    
    /// Constructs this class given the framework where the GPR was trained and
    /// the statespace in which the model operates
    DeterministicModel(
        const std::shared_ptr<actionspace::GenericActionSpace> actionspace,
        const std::shared_ptr<aikido::statespace::StateSpace> statespace) :
        m_actionspace(actionspace),
        m_statespace(statespace) {}

    ~DeterministicModel() = default;

    /// Documentation inherited
    void get_successor(
        const actionspace::ActionSpace::Action& input,
        const aikido::statespace::StateSpace::State& in_,
        aikido::statespace::StateSpace::State* out_) override;

 private:
    const std::shared_ptr<actionspace::GenericActionSpace> m_actionspace;
    const std::shared_ptr<aikido::statespace::StateSpace> m_statespace;
};

}  // namespace model
}  // namespace libcozmo

#endif  // INCLUDE_MODEL_GPRMODEL_HPP_