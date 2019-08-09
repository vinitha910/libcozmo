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

#ifndef INCLUDE_STATESPACE_STATESPACE_H_
#define INCLUDE_STATESPACE_STATESPACE_H_

#include <boost/functional/hash.hpp>
#include <Eigen/Dense>
#include <vector>
#include <utility>
#include <unordered_map>
#include "aikido/distance/SE2.hpp"
#include "StateSpace.hpp"

namespace libcozmo {
namespace statespace {

// This class implements a discretized two-dimensional Special Euclidean
// group SE(2),i.e. the space of planar rigid body transformations.
class SE2 : public virtual StateSpace {
 public:
    class State : public StateSpace::State 
    {
     public:
        State() : x(0), y(0), theta(0) {};

        ~State() = default;

        explicit State(const int& x, const int& y, const int& theta) : \
            x(x), y(y), theta(theta) {};
        
        bool operator== (const State& state) const {
            return x == state.x && y == state.y && theta == state.theta;
        }

        friend std::size_t hash_value(const State& state) {
            std::size_t seed = 0;
            boost::hash_combine(seed, boost::hash_value(state.x));
            boost::hash_combine(seed, boost::hash_value(state.y));
            boost::hash_combine(seed, boost::hash_value(state.theta));
            return seed;
        }

     private:
        int x;
        int y;
        int theta;
    
        friend class SE2;
    };

    // Constructor

    // \param resolution_m Resolution of discretized state (m)
    // \param num_theta_vals Number of discretized theta values; Must be a 
    // power of 2
    SE2(
        const double& resolution_m,
        const int& num_theta_vals) : \
        m_resolution(resolution_m),
        m_num_theta_vals(num_theta_vals),
        m_statespace(std::make_shared<aikido::statespace::SE2>()),
        m_distance_metric(aikido::distance::SE2(m_statespace)) {}

    ~SE2() {}

    // Returns ID of the state with given pose if it exists,
    // Otherwise creates a new state and returns its ID

    // \param pose The discrete state
    int get_or_create_state(const StateSpace::State* _state) override;

    // Returns ID of the state with given transformation if it exists,
    // Otherwise creates a new state and returns its ID

    // \param state The SE2 state
    int get_or_create_state(
        const aikido::statespace::SE2::State* state,
        StateSpace::State* discrete_state);

    // Converts discretized state to continuous state

    // \param state The discrete state
    void discrete_state_to_continuous(
        const StateSpace::State* _state,
        aikido::statespace::SE2::State* state_continuous) const;

    // Converts continuous state to discretized state

    // \param state The SE2 state
    void continuous_state_to_discrete(
        const aikido::statespace::SE2::State* state, 
        StateSpace::State* discrete_state);

    // Returns true and fills state_id with corresponding ID if state exists 
    // and false otherwise

    // \param pose The discrete state
    // \param id The state id
    bool get_state_id(
        const StateSpace::State* _state, int* state_id) const override;

    // Returns true and fills state with corresponding state if state with 
    // given ID exists and false otherwise

    // \param state_id The ID of the state
    // \param state The discrete state
    bool get_state(const int& state_id, StateSpace::State* state) const override;

    // Return true if the state is valid and false otherwise

    // \param state The discretized state
    bool is_valid_state(const StateSpace::State* _state) const override;

    // Returns the number of currently existing states
    int get_num_states() const;

    // Returns distance between two SE2 states

    // \param state_1, state_2 The states to calculate the SE2 distance between
    double get_distance(
        const aikido::statespace::SE2::State* state_1,
        const aikido::statespace::SE2::State* state_2) const;

 private:
    /// Creates a new state and adds it to the statespace
    ///
    /// \return pointer to the state
    StateSpace::State* create_state() override;

    void copy_state(
        const StateSpace::State* _source, StateSpace::State* _destination) const;

    // Get normalized angle (radians) in [0, 2pi]

    // \param theta_rad Angle (radians)
    double normalize_angle_rad(const double& theta_rad) const;

    // Converts discrete angle to continuous (radians)

    // \param theta Discrete angle
    double discrete_angle_to_continuous(const int& theta) const;

    // Converts discrete angle to continuous (radians)

    // \param theta Continuous angle
    int continuous_angle_to_discrete(const double& theta) const;

    // Converts discrete position to continuous

    // \param position The discrete coordinates
    Eigen::Vector2d discrete_position_to_continuous(
        const Eigen::Vector2i& position) const;

    // Converts discrete position to continuous

    // \param position The discrete coordinates
    Eigen::Vector2i continuous_position_to_discrete(
        const Eigen::Vector2d& position) const;

    // Maps discrete state to state ID
    std::unordered_map<State, int, boost::hash<State>> m_state_to_id_map;

    // Vector of discrete states; index of state is the state ID.
    std::vector<State*> m_state_map;

    // Number of discretized theta values
    const int m_num_theta_vals;

    // Resolution of discrete state
    const double m_resolution;

    std::shared_ptr<aikido::statespace::SE2> m_statespace;
    aikido::distance::SE2 m_distance_metric;
};

}  // namespace statespace
}  // namespace libcozmo

#endif  // INCLUDE_STATESPACE_STATESPACE_H_

