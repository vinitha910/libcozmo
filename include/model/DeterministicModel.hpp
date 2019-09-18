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

/// This model predicts Cozmo's end state after executing a given action
///
/// Given an action (ModelInput), the delta state (ModelOutput) is predicted
/// by taking the action's speed and duration to calculate the change in
/// distance with respect to Cozmo's x & y axis, and the executed action's
/// heading as the output heading.
class DeterministicModel : public virtual Model {
 public:
    class ModelType : public Model::ModelType {
     public:
        ModelType() {}

        ~ModelType() = default;
    };

    class ModelInput : public Model::ModelInput {
     public:
        /// Constructs input with speed, duration, and heading from given action
        explicit ModelInput(
            const actionspace::GenericActionSpace::Action& action) : \
            m_action(action) {}

        ~ModelInput() = default;

        double get_speed() const { return m_action.m_speed; }
        double get_duration() const { return m_action.m_duration; }
        double get_heading() const { return m_action.m_heading; }
     private:
        const actionspace::GenericActionSpace::Action m_action;
    };

    class ModelOutput : public Model::ModelOutput {
     public:
        /// Constructs output from predicted delta distance and final heading
        /// after executing given action
        ModelOutput(const double& x, const double& y, const double& theta) : \
            m_x(x), m_y(y), m_theta(theta) {}

        ~ModelOutput() = default;
        double getX() const { return m_x; }
        double getY() const { return m_y; }
        double getTheta() const { return m_theta; }

     private:
        const double m_x;
        const double m_y;
        const double m_theta;
    };

    /// Identitiy constructor
    DeterministicModel(): m_model(nullptr) {}
    ~DeterministicModel() {}

    /// Documentation inherited
    bool load_model(Model::ModelType* model) override;

    /// Documentation inherited
    Model::ModelOutput* get_prediction(const Model::ModelInput& input) override;

 private:
    Model::ModelType* m_model;
};

}  // namespace model
}  // namespace libcozmo

#endif  // LIBCOZMO_MODEL_GENERICMODEL_HPP
