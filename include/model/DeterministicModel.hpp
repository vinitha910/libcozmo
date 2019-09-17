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

/// Deterministic model to predict Cozmo's movement
///
/// Delta state calculated by calculating predicted change in cozmo's position
/// via distance change in x,y respectively, with its new heading predicted as
/// the heading of given action
///
/// Given action, delta x, y, theta is calculated
class DeterministicModel : public virtual Model {
 public:
    class ModelType : public Model::ModelType {
     public:
        ModelType() {}

        ~ModelType() = default;
    };

    class ModelInput : public Model::ModelInput {
     public:
        /// Constructs input with given parameters
        explicit ModelInput(
            const actionspace::GenericActionSpace::Action& action) : \
            m_action(action) {}

        ~ModelInput() = default;

        double get_speed() { return m_action.m_speed; }
        double get_duration() { return m_action.m_duration; }
        double get_heading() { return m_action.m_heading; }
     private:
        const actionspace::GenericActionSpace::Action m_action;
    };

    class ModelOutput : public Model::ModelOutput {
     public:
        /// Constructs output with given parameters
        ModelOutput(const double& x, const double& y, const double& theta) : \
            m_x(x), m_y(y), m_theta(theta) {}

        ~ModelOutput() = default;
        double getX() { return m_x; }
        double getY() { return m_y; }
        double getTheta() { return m_theta; }

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
    Model::ModelOutput* get_prediction(const Model::ModelInput& input) override;

 private:
    Model::ModelType* m_model;
};

}  // namespace model
}  // namespace libcozmo

#endif  // LIBCOZMO_MODEL_GENERICMODEL_HPP
