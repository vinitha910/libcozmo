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

class LatticeGraphModel : public virtual WorldRepresentation {
 public:
    /// The input into LatticeGraphModel for Generic action handling
    /// Actions are performed w.r.t. to Cozmo
    class ModelInput : public WorldRepresentation::ModelInput {
     public:
        explicit ModelInput(
            const double& speed,
            const double& direction) :
            m_speed(speed),
            m_direction(direction) {}
        
        double get_speed() { return m_speed; }
        double get_heading() { return m_direction; }

        /// The speed (mm/s)
        const double m_speed;

        /// The direction angle of action (radians)
        const double m_direction;
    };

    /// Do I need this for LatticeGraph?
    /// The output of the Guassian Process Regressor Model is the delta state
    /// (i.e. the distance the object moved and the change in orientation)
    class ModelOutput : public WorldRepresentation::ModelOutput {
     public:
        explicit ModelOutput(const double& distance, const double& dtheta) :
            m_distance(distance), m_dtheta(dtheta) {}

        ModelOutput() : m_distance(0.0), m_dtheta(0.0) {}

        /// The distance the object moved after applying an action
        double m_distance;

        /// The change in orientation of the object after applying an action
        double m_dtheta;
    };

    /// Constructs this class given the framework where the GPR was trained and
    /// the statespace in which the model operates
    LatticeGraphModel(
        const std::shared_ptr<actionspace::GenericActionSpace> actionspace,
        const std::shared_ptr<aikido::statespace::StateSpace> statespace) :
        m_actionspace(framework),
        m_statespace(statespace) {}

    ~LatticeGraphModel() = default;

    /// Documentation inherited
    void get_successor(
        const WorldRepresentation::ModelInput& input,
        const aikido::statespace::StateSpace::State& in_,
        aikido::statespace::StateSpace::State* out_) override;

 private:
    const std::shared_ptr<actionspace::GenericActionSpace> m_actionspace;
    const std::shared_ptr<aikido::statespace::StateSpace> m_statespace;
};

}  // namespace model
}  // namespace libcozmo

#endif  // INCLUDE_MODEL_GPRMODEL_HPP_