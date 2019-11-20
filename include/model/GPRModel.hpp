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

namespace libcozmo {
namespace model {

/// This class implements a Gaussian Process Regressor(GPR), a type of
/// multi-dimensional linear regression model, from Scikit-Learn.
///
/// Given input vector action (a) : [speed, aspect ratio, edge offset], it
/// predicts outcome delta state (Δs) : [delta distance, delta theta].
/// such that the function can be defined as f: a--> Δs
class GPRModel : public virtual Model {
 public:
    /// Constructs this class given the framework where the GPR was trained and
    /// the statespace in which the model operates
    GPRModel(
        const std::shared_ptr<ModelFramework> framework) :
        m_framework(framework) {}

    ~GPRModel() = default;

    /// Documentation inherited
    /// Action takes in vector from ObjectOrientedActionSpace in
    /// format [speed, aspect_ratio, edge_offset, heading_offset],
    /// but only the first three elemnts are used by the model.
    /// State in SE2 format: [x, y, theta]
    bool predict_state(
        const Eigen::VectorXd& input_action,
        const Eigen::VectorXd& input_state,
        Eigen::VectorXd* output_state) const override;

 private:
    const std::shared_ptr<ModelFramework> m_framework;
};

}  // namespace model
}  // namespace libcozmo

#endif  // INCLUDE_MODEL_GPRMODEL_HPP_
