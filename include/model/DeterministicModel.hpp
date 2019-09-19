////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019,  Brian Lee, Vinitha Ranganeni
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

#ifndef LIBCOZMO_MODEL_GENERICMODEL_HPP
#define LIBCOZMO_MODEL_GENERICMODEL_HPP

#include <Eigen/Dense>
#include "model/Model.hpp"
#include "actionspace/GenericActionSpace.hpp"
namespace libcozmo {
namespace model {

/// Given an action(ModelInput), this model predicts the delta SE2 state
/// (ModelOutput)
class DeterministicModel : public virtual Model {
 public:
    class DeterministicModelType : public Model::ModelType {
     public:
        DeterministicModelType() {}

        ~DeterministicModelType() = default;
    };

    class DeterministicModelInput : public Model::ModelInput {
     public:
        /// The ModelInput is an action defined by a speed, duration and heading
        explicit DeterministicModelInput(
            const actionspace::GenericActionSpace::Action& action) : \
            m_action(action) {}

        ~DeterministicModelInput() = default;

        double get_speed() const { return m_action.m_speed; }
        double get_duration() const { return m_action.m_duration; }
        double get_heading() const { return m_action.m_heading; }
     private:
        const actionspace::GenericActionSpace::Action m_action;
    };

    class DeterministicModelOutput : public Model::ModelOutput {
     public:
        /// The ModelOutput is the change in the SE2 state after applying the
        /// given action
        DeterministicModelOutput() {
            m_x = 0;
            m_y = 0;
            m_theta = 0;}
        DeterministicModelOutput(
            const double& x,
            const double& y,
            const double& theta) : \
            m_x(x), m_y(y), m_theta(theta) {}

        ~DeterministicModelOutput() = default;
        double getX() const { return m_x; }
        double getY() const { return m_y; }
        double getTheta() const { return m_theta; }

     private:
        double m_x;
        double m_y;
        double m_theta;
    };

    /// Identitiy constructor
    DeterministicModel(): m_model(nullptr) {}
    ~DeterministicModel() {}

    /// Documentation inherited
    bool load_model(Model::ModelType* model) override;

    /// Documentation inherited
    bool get_prediction(
        const Model::ModelInput& input,
        Model::ModelOutput* output) const override;

 private:
    Model::ModelType* m_model;
};

}  // namespace model
}  // namespace libcozmo

#endif  // LIBCOZMO_MODEL_GENERICMODEL_HPP