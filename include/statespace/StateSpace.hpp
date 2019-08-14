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

#ifndef LIBCOZMO_STATESPACE_STATESPACE_HPP_
#define LIBCOZMO_STATESPACE_STATESPACE_HPP_

#include "aikido/distance/SE2.hpp"

namespace libcozmo {
namespace statespace {

class StateSpace
{
 public:

    /// Base class for all discrete states
    class State;

    /// Checks if given (discrete) state exists in the statespace; if not, 
    /// creates and adds the state to the statespace
    ///
    /// \param _state Input state (assumption: state is valid)
    /// \return state ID
    virtual int get_or_create_state(const State& _state) = 0;
    
    /// Checks if given (continuous) state exists in the statespace; if not, 
    /// creates and adds the state to the statespace
    ///
    /// \param _state Input state
    /// \return State ID
    virtual int get_or_create_state(
        const aikido::statespace::StateSpace::State& _state) = 0;

    /// Converts the given discrete state into a continuous state
    ///
    /// \param _state Input discrete state (assumption: state is valid)
    /// \param[out] _continuous_state Output continuous state
    virtual void discrete_state_to_continuous(
        const State& _state,
        aikido::statespace::StateSpace::State* _continuous_state) const = 0;

    /// Converts the given continuous state into a discrete state
    ///
    /// \param _state Input discrete state 
    /// \param[out] _discrete_state Output continuous state
    virtual void continuous_state_to_discrete(
        const aikido::statespace::StateSpace::State& _state, 
        State* _discrete_state) const = 0;

    /// Gets the state ID for the given state if the state exists in the
    /// statespace
    ///
    /// \param _state Input discrete state (assumption: state is valid)
    /// \param[out] _state_id State ID
    /// \return True if the state ID was found and false otherwise
    virtual bool get_state_id(const State& _state, int* _state_id) const = 0;

    /// Gets the state for the give ID if the state exists in the statespace
    ///
    /// \param _state_id The ID of the state
    /// \return Pointer to the discrete state
    virtual State* get_state(const int& _state_id) const = 0;

    /// Checks if the given state is a valid state; validity of state varies 
    /// based on state type
    ///
    /// \param _state Input discrete state
    /// \return True if the state if valid and false otherwise
    virtual bool is_valid_state(const State& _state) const = 0;

    /// Get the number of states in the statespace
    ///
    /// \return Number of states
    virtual int size() const = 0;

    /// Gets the distance between two SE2 states
    ///
    /// \param _state_1, _state_2 The discrete states to calculate the distance
    /// between; distance metric varies based on states (assumption: states are 
    /// valid)
    /// \return Distance between the states
    virtual double get_distance(
        const State& _state_1, const State& _state_2) const = 0;

    /// Gets the distance between two SE2 states
    ///
    /// \param _state_1, _state_2 The continuous states to calculate the 
    /// distance between; distance metric varies based on states
    /// \return Distance between the states
    virtual double get_distance(
        const aikido::statespace::StateSpace::State& _state_1,
        const aikido::statespace::StateSpace::State& _state_2) const = 0;

    /// Copies a discrete state
    /// 
    /// \param _source Input state (state to copy)
    /// \param[out] _destination Output state
    virtual void copy_state(const State& _source, State* _destination) const = 0;

 private:
    virtual State* create_state() = 0;
};


class StateSpace::State
{
 protected:
    // This is a base class that should only only be used in derived classes.
    State() = default;

    ~State() = default;
};

}  // namespace statespace
}  // namespace libcozmo

#endif