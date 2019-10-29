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

#include "model/GPRModel.hpp"
#include <sstream>
#include <iostream>
#include "aikido/statespace/SE2.hpp"
#include "actionspace/ObjectOrientedActionSpace.hpp"
#include "utils/utils.hpp"

namespace libcozmo {
namespace model {

void GPRModel::inference(
    const ModelInput& input, ModelOutput* output_) {

    PyObject* p_list = PyList_New(3);
    PyList_SetItem(p_list, 0, Py_BuildValue("f", input.speed));
    PyList_SetItem(p_list, 1, Py_BuildValue("f", input.edge_offset_ratio));
    PyList_SetItem(p_list, 2, Py_BuildValue("f", input.aspect_ratio));

    PyObject* p_input = PyList_New(1);
    PyList_SetItem(p_input, 0, p_list);

    PyObject* p_inference_fn =
        PyObject_GetAttrString(m_framework->get_module(), "inference");
    PyObject* p_args = PyTuple_Pack(2, m_framework->get_model(), p_input);
    PyObject* p_result = PyObject_CallObject(p_inference_fn, p_args);

    ModelOutput* output = static_cast<ModelOutput*>(output_);
    output->distance = PyFloat_AsDouble(PyList_GetItem(p_result, 0));
    output->dtheta = PyFloat_AsDouble(PyList_GetItem(p_result, 1));
}

void GPRModel::get_successor(
    const actionspace::ActionSpace::Action& model_input_,
    const aikido::statespace::StateSpace::State& in_,
    aikido::statespace::StateSpace::State* out_) {
    
    // Get ModelInput from action
    const actionspace::ObjectOrientedActionSpace::GenericAction& action = 
        static_cast<const actionspace::ObjectOrientedActionSpace::GenericAction&>(
            model_input_);
    
    const double angle = action.heading_offset();

    ModelInput model_input(
        action.speed(),
        action.edge_offset(),
        action.aspect_ratio(),
        Eigen::Vector2d{cos(angle), sin(angle)}
        );

    ModelOutput output;
    inference(model_input, &output);
    const aikido::statespace::SE2::State in_state =
        static_cast<const aikido::statespace::SE2::State&>(in_);

    const Eigen::Vector2d direction_norm = model_input.direction.normalized();
    const auto transform = in_state.getIsometry();
    const double x =
        transform.translation().x() + output.distance * direction_norm.x();
    const double y =
        transform.translation().y() + output.distance * direction_norm.y();

    // Calculate new rotation
    Eigen::Rotation2Dd rotation = Eigen::Rotation2Dd::Identity();
    rotation.fromRotationMatrix(transform.rotation());
    const double theta =
        utils::angle_normalization(rotation.angle() + output.dtheta);

    // Set out state
    aikido::statespace::SE2::State* out_state =
        static_cast<aikido::statespace::SE2::State*>(out_);
    Eigen::Isometry2d t = Eigen::Isometry2d::Identity();
    const Eigen::Rotation2D<double> rot(theta);
    t.linear() = rot.toRotationMatrix();
    t.translation() = Eigen::Vector2d(x, y);
    out_state->setIsometry(t);
}

}  // namespace model
}  // namespace libcozmo
