//////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019, Vinitha Ranganeni
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

#ifndef LIBCOZMO_ACTIONSPACE_ACTIONSPACE_HPP_
#define LIBCOZMO_ACTIONSPACE_ACTIONSPACE_HPP_

namespace libcozmo {
namespace actionspace {

class ActionSpace
{
 public:

    /// Base class for all Actions
    class Action;

    // Calculates and returns similarity with another action
    // Similarity is defined by the euclidean distance based on
    // their speed, duration, and direction (x,y calcualted separately)

    // \param action_id1, actionid2 The ID of two actions to compare
    virtual double action_similarity(
        const int& action_id1,
        const int& action_id2) const = 0;

    virtual Action* get_action(const int& action_id) const = 0;

    virtual void publish_action(const int& action_id) const = 0;
 private:
    virtual bool is_valid_action_id(const int& action_id) const = 0;
};

class ActionSpace::Action
{
 protected:
    // This is a base class that should only only be used in derived classes.
    Action() = default;

    ~Action() = default;
};

}  // namespace actionspace
}  // namespace libcozmo

#endif