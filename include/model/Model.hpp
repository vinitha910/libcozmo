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

#ifndef LIBCOZMO_MODEL_MODEL_HPP_
#define LIBCOZMO_MODEL_MODEL_HPP_

namespace libcozmo {
namespace model {

class Model {
 public:
    /// Base class for all models to be loaded
    class ModelType;

    /// Base class for model input
    class ModelInput;

    /// Base class for model output
    class ModelOutput;

    /// Loads given Model
    ///
    /// \param model The model to load
    /// \return true if model succesfully loaded, else false
    virtual bool load_model(Model::ModelType* model) = 0;

    /// Predict delta state
    ///
    /// \param input The model input, varies based on model type
    /// \return model output, varies based on model type
    virtual bool get_prediction(
        const ModelInput& input,
        Model::ModelOutput* output) const = 0;
};

/// This class specifies the type of Model, particularly the framework used to
/// create the Model (e.g. scikit-learn, pytorch, etc)
class Model::ModelType {
 protected:
    /// This is a base class that should only be used in derived classes
    ModelType() = default;

    ~ModelType() = default;
};

/// Class for handling input of the model; varies based on model type
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

#endif  // LIBCOZMO_MODEL_MODEL_HPP_