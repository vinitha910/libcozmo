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

#ifndef INCLUDE_MODEL_MODEL_HPP_
#define INCLUDE_MODEL_MODEL_HPP_

#include "aikido/statespace/StateSpace.hpp"

namespace libcozmo {
namespace model {

/// Abstract class for all model types. When using this class or its derived
/// classes in a script you must wrap the code with Py_Initialize() and
/// Py_Finalize();
class Model {
 public:
    // /// Base class for model input
    // class ModelInput;

    // /// Base class for model output
    // class ModelOutput;

    // /// Get the model's predicted output given then input
    // ///
    // /// \param input The model input, dependent on model type
    // /// \param[out] output The ouput after running inference
    // virtual void inference(const ModelInput& input, ModelOutput* output) = 0;

    /// Get the output state given the input and the current state
    ///
    /// \param in The in/current state
    /// \param[out] The predicted output state given the model input
    virtual void predict_state(
        const ModelInput& input,
        const aikido::statespace::StateSpace::State& in_,
        aikido::statespace::StateSpace::State* out_) = 0;
};

/// Class for handling input of the model; varies based on model
class Model::ModelInput {
 protected:
    /// This is a base class that should only be used in derived classes
    ModelInput() = default;

    ~ModelInput() = default;
};

/// Class for handling output of the model; varies based on model type
class Model::ModelOutput {
 protected:
    /// This is a base class that should only be used in derived classes
    ModelOutput() = default;

    ~ModelOutput() = default;
};

}  // namespace model
}  // namespace libcozmo

#endif  // INCLUDE_MODEL_MODEL_HPP_
