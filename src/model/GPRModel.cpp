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
#include "utils/utils.hpp"

namespace libcozmo {
namespace model {

bool GPRModel::predict_state(
        const Eigen::VectorXd& input_action,
        const Eigen::VectorXd& input_state,
        Eigen::VectorXd* output_state) const {
    if (input_action.size() != 4 || input_state.size() != 3) {
        return false;
    }

    // Get speed, edge_offset, aspect_ratio into Python list
    PyObject* p_list = PyList_New(3);
    PyList_SetItem(p_list, 0, Py_BuildValue("f", input_action[0]));
    PyList_SetItem(p_list, 1, Py_BuildValue("f", input_action[2]));
    PyList_SetItem(p_list, 2, Py_BuildValue("f", input_action[1]));

    PyObject* p_input = PyList_New(1);
    PyList_SetItem(p_input, 0, p_list);

    // Get model inference
    // Model output is [distance, dtheta]
    PyObject* p_inference_fn =
        PyObject_GetAttrString(m_framework->get_module(), "inference");
    PyObject* p_args = PyTuple_Pack(2, m_framework->get_model(), p_input);
    PyObject* p_result = PyObject_CallObject(p_inference_fn, p_args);

    double distance = PyFloat_AsDouble(PyList_GetItem(p_result, 0));
    double dtheta = PyFloat_AsDouble(PyList_GetItem(p_result, 1));
    double x = input_state[0] + distance * cos(dtheta);
    double y = input_state[1] + distance * sin(dtheta);
    double theta = input_state[2] + dtheta;

    Eigen::Vector3d state(x, y, theta);
    *output_state = state;
    return true;
}

}  // namespace model
}  // namespace libcozmo
