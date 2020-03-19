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

#ifndef INCLUDE_STATESPACE_STATESPACE_HPP_
#define INCLUDE_STATESPACE_STATESPACE_HPP_

#include "aikido/distance/SE2.hpp"

namespace libcozmo {
namespace statespace {

class StateSpace {
 public:
    /// Base class for all discrete states
    class State;

    /// Base class for all continuous states
    class ContinuousState;

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
        const ContinuousState& _state) = 0;

    /// Checks if given state via vector representation exists in the
    /// statespace; if not, creates and adds the state to the statespace
    /// Throws an exception if vector size does not match the exected size,
    /// which depends on derived class
    ///
    /// \param _state Vector representation
    /// \return State ID
    virtual int get_or_create_state(
        const Eigen::VectorXd& _state) = 0;

    /// Converts the given discrete state into a continuous state
    ///
    /// \param _state Input discrete state (assumption: state is valid)
    /// \param[out] _continuous_state Output continuous state
    virtual void discrete_state_to_continuous(
        const State& _state,
        ContinuousState* _continuous_state) const = 0;

    /// Converts the given continuous state into a discrete state
    ///
    /// \param _state Input discrete state
    /// \param[out] _discrete_state Output continuous state
    virtual void continuous_state_to_discrete(
        const ContinuousState& _state,
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

    /// Copies a discrete state
    ///
    /// \param _source Input state (state to copy)
    /// \param[out] _destination Output state
    virtual void copy_state(
        const State& _source,
        State* _destination) const = 0;

    /// Gets the resolution of statespace
    ///
    /// \return Resolution
    virtual double get_resolution() const = 0;

    /// Adds pairwise elements in the given vectors
    ///
    /// \param _vector_1 The first vector to add
    /// \param _vector_2 The second vector to add
    /// \param[out] _vector_out Output vector
    /// Throws an exception if the vector sizes are not equal
    virtual void add(
        const Eigen::VectorXd& _vector_1,
        const Eigen::VectorXd& _vector_2,
        Eigen::VectorXd* _vector_out) const {
        if (_vector_1.size() != _vector_2.size()) {
            std::stringstream msg;
            msg << "states do not have equal size: "
                << ", got " << _vector_1.size() <<
                    " and " << _vector_2.size() << ".\n";
            throw std::runtime_error(msg.str());
        }
        for (int i = 0; i < _vector_1.size(); i++) {
            (*_vector_out)[i] = _vector_1[i] + _vector_2[i];
        }
    }

 private:
    virtual State* create_state() = 0;
};


class StateSpace::State {
 public:
    /// Convert state to its vector representation
    ///
    /// \return Output state vector
    virtual Eigen::VectorXd vector() const = 0;

    /// Converts state value given vector representation
    /// Throws an exception if the state vector size is incorrect; allowed
    /// vector size depends on derived class
    ///
    /// \param state State vector
    virtual void from_vector(const Eigen::VectorXd& state) = 0;

    /// Equality operator
    virtual bool operator== (const State& state) const = 0;
 protected:
    // This is a base class that should only only be used in derived classes.
    State() = default;

    ~State() = default;
};

class StateSpace::ContinuousState {
    /// Convert state to its vector representation
    ///
    /// \return Output state vector
    virtual Eigen::VectorXd vector() const = 0;

    /// Converts state value given vector representation
    /// Throws an exception if the state vector size is incorrect; allowed
    /// vector size depends on derived class
    ///
    /// \param state State vector
    virtual void from_vector(const Eigen::VectorXd& state) = 0;

    /// Converts vector representation to its continuous state value
    /// virtual void to_vector(const ContinuousState& state) = 0;

    /// Equality operator
    virtual bool operator== (const ContinuousState& state) const = 0;
 protected:
    // This is a base class that should only only be used in derived classes.
    ContinuousState() = default;

    ~ContinuousState() = default;
};

}  // namespace statespace
}  // namespace libcozmo

#endif  // INCLUDE_STATESPACE_STATESPACE_HPP_
