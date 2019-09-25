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

#include "model/ScikitLearnFramework.hpp"
#include <sstream>
#include <iostream>
#include <stdexcept>

namespace libcozmo {
namespace model {

ScikitLearnFramework::ScikitLearnFramework(const std::string& model_path) {
    if (!initialize(model_path)) {
        throw std::invalid_argument(
            "[ScikitLearnFramework] Invalid model_path");
    }
}

ScikitLearnFramework::~ScikitLearnFramework() {
    Py_DecRef(p_model);
    Py_DecRef(p_module);
}

bool ScikitLearnFramework::initialize(const std::string& model_path) {
    std::stringstream buf;
    buf << "import _pickle as pickle" << std::endl
        << "def load_model(filename):" << std::endl
        << "    return pickle.load(open(filename, 'rb'))" << std::endl
        << "def inference(model, input):" << std::endl
        << "    return model.predict(input).tolist()[0]" << std::endl;

    // Compile python code
    PyObject* p_compiled_fn =
        Py_CompileString(buf.str().c_str(), "" , Py_file_input);

    // Create module for python code
    if (p_compiled_fn != NULL) {
        p_module =
            PyImport_ExecCodeModule("scikit_learn_module", p_compiled_fn);

        // Get load_model function
        if (p_module != NULL) {
            PyObject* p_load_model_fn =
                PyObject_GetAttrString(p_module, "load_model");

            // Pack arguments and load model
            if (p_load_model_fn != NULL) {
                PyObject *p_file = Py_BuildValue("s", model_path.c_str());
                PyObject *p_args = PyTuple_Pack(1, p_file);
                p_model = PyObject_CallObject(p_load_model_fn, p_args);
                Py_DecRef(p_args);
                Py_DecRef(p_file);
            }

            Py_DecRef(p_load_model_fn);
            Py_DecRef(p_compiled_fn);

            return true;
        }
    }
    return false;
}

}  // namespace model
}  // namespace libcozmo
