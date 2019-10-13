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

class GPRModel : public virtual Model {
 public:
    /// The input into the Guassian Process Regressor Model is the
    /// object-oriented action being executed by cozmo
    class ModelInput : public Model::ModelInput {
     public:
        explicit ModelInput(
            const double& speed,
            const double& edge_offset_ratio,
            const double& aspect_ratio,
            const Eigen::Vector2d& direction) :
            speed(speed),
            edge_offset_ratio(edge_offset_ratio),
            aspect_ratio(aspect_ratio),
            direction(direction) {}

        /// The speed in mm/s
        const double speed;

        /// Normalized distance from center of edge in range [-1, 1], where -1
        /// and 1 are the left and right corner of the object respectively
        const double edge_offset_ratio;

        /// Either the width or height in the aspect ratio of the object
        const double aspect_ratio;

        /// The direction vector of the action
        const Eigen::Vector2d direction;
    };

    /// The output of the Guassian Process Regressor Model is the delta state
    /// (i.e. the distance the object moved and the change in orientation)
    class ModelOutput : public Model::ModelOutput {
     public:
        explicit ModelOutput(const double& distance, const double& dtheta) :
            distance(distance), dtheta(dtheta) {}

        ModelOutput() : distance(0.0), dtheta(0.0) {}

        /// The distance the object moved after applying an action
        double distance;

        /// The change in orientation of the object after applying an action
        double dtheta;
    };

    /// Constructs this class given the framework where the GPR was trained and
    /// the statespace in which the model operates
    GPRModel(
        const std::shared_ptr<ModelFramework> framework,
        const std::shared_ptr<aikido::statespace::StateSpace> statespace) :
        m_framework(framework),
        m_statespace(statespace) {}

    ~GPRModel() = default;

    /// Documentation inherited
    void inference(
        const Model::ModelInput& input, Model::ModelOutput* output) override;

    /// Documentation inherited
    void predict_state(
        const Model::ModelInput& input,
        const aikido::statespace::StateSpace::State& in_,
        aikido::statespace::StateSpace::State* out_) override;

 private:
    const std::shared_ptr<ModelFramework> m_framework;
    const std::shared_ptr<aikido::statespace::StateSpace> m_statespace;
};

}  // namespace model
}  // namespace libcozmo

#endif  // INCLUDE_MODEL_GPRMODEL_HPP_
