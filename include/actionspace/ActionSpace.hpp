//////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019, Brian Lee, Vinitha Ranganeni
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

#ifndef INCLUDE_MODEL_DETERMINISTICMODEL_HPP_
#define INCLUDE_MODEL_DETERMINISTICMODEL_HPP_

#include "aikido/distance/SE2.hpp"

namespace libcozmo {
namespace actionspace {

class ActionSpace {
 public:
    /// Base class for all Actions
    class Action;

    /// Calculates similarity between two actions
    /// Similarity metric varies based on action definition
    ///
    /// \param action_id1, actionid2 IDs of actions to compare
    /// \param[out] similarity value
    /// \return True if calculation successful; false otherwise
    virtual bool action_similarity(
        const int& action_id1,
        const int& action_id2,
        double* similarity) const = 0;

    /// Gets pointer to action with the given ID if action exists; if not,
    /// returns null pointer
    ///
    /// \param action_id The action ID
    /// \return Pointer to action
    virtual Action* get_action(const int& action_id) const = 0;

    /// Publishes action msg for given action ID
    ///
    /// \param action_id Action ID
    /// \param publisher ROS publisher
    /// \param _state The aikido state of the cube
    /// \return True if publish successful; false otherwise;
    virtual bool publish_action(
        const int& action_id,
        const ros::Publisher& publisher,
        const aikido::statespace::StateSpace::State& _state) const = 0;

    /// Checks whether given action ID is valid
    ///
    /// \param action_id Action ID
    /// \return True if action ID valid; false otherwise
    virtual bool is_valid_action_id(const int& action_id) const = 0;

    /// Gets the total number of actions in the action space
    ///
    /// \return Number of actions
    virtual int size() const = 0;
};

class ActionSpace::Action {
 protected:
    // This is a base class that should only be used in derived classes.
    Action() = default;

    ~Action() = default;
};

}  // namespace actionspace
}  // namespace libcozmo

#endif  // INCLUDE_MODEL_DETERMINISTICMODEL_HPP_
