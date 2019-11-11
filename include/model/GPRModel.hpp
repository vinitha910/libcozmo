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

#ifndef INCLUDE_MODEL_GPRMODEL_HPP_
#define INCLUDE_MODEL_GPRMODEL_HPP_

#include <Python.h>
#include <Eigen/Geometry>
#include <memory>
#include "model/Model.hpp"
#include "model/ModelFramework.hpp"
#include "model/ScikitLearnFramework.hpp"
#include "statespace/StateSpace.hpp"

namespace libcozmo {
namespace model {

/// This model assumes it using SE2-based state vector as its state input and
/// object oriented-based action vector as its model input.
/// Addtionally, path to model.pkl passed in as construction argument
/// needs to be the absolute path.
class GPRModel : public virtual Model {
 public:
    /// Constructs this class given the framework where the GPR was trained in
    GPRModel(
        const std::shared_ptr<ModelFramework> framework) :
        m_framework(framework) {}

    ~GPRModel() = default;

    /// Documentation inherited
    /// Model input format in [speed, aspect_ratio, edge_offset, heading_offset]
    bool predict_state(
        const Eigen::VectorXd& model_input,
        const Eigen::VectorXd& state_input,
        Eigen::VectorXd* state_output) const override;

 private:
    const std::shared_ptr<ModelFramework> m_framework;
};

}  // namespace model
}  // namespace libcozmo

#endif  // INCLUDE_MODEL_GPRMODEL_HPP_
