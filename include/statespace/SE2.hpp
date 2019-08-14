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

#ifndef LIBCOZMO_STATESPACE_SE2_HPP_
#define LIBCOZMO_STATESPACE_SE2_HPP_

#include "StateSpace.hpp"
#include <boost/functional/hash.hpp>
#include <Eigen/Dense>
#include <vector>
#include <utility>
#include <unordered_map>

namespace libcozmo {
namespace statespace {

// This class implements a discretized two-dimensional Special Euclidean
// group SE(2), i.e. the space of planar rigid body transformations.
class SE2 : public virtual StateSpace {
 public:
    class State : public StateSpace::State 
    {
     public:
        /// Constructs identity state
        State() : x(0), y(0), theta(0) {};

        ~State() = default;

        /// Constructs state with given parameters
        explicit State(const int& x, const int& y, const int& theta) : \
            x(x), y(y), theta(theta) {};
        
        /// Equality operator
        bool operator== (const State& state) const {
            return x == state.x && y == state.y && theta == state.theta;
        }

        /// Custom state hash
        friend std::size_t hash_value(const State& state) {
            std::size_t seed = 0;
            boost::hash_combine(seed, boost::hash_value(state.x));
            boost::hash_combine(seed, boost::hash_value(state.y));
            boost::hash_combine(seed, boost::hash_value(state.theta));
            return seed;
        }

        int getX() const { return x; }
        int getY() const { return y; }
        int getTheta() const { return theta; } 

     private:
        int x;
        int y;
        int theta;

        friend class SE2;
    };

    /// Constructs a discretized SE2 state space 
    ///
    /// \param resolution_m Resolution of the environment (m)
    /// \param num_theta_vals Number of discretized theta values; Must be a 
    /// power of 2
    SE2(
        const double& resolution_m,
        const int& num_theta_vals) : \
        m_resolution(resolution_m),
        m_num_theta_vals(num_theta_vals),
        m_statespace(std::make_shared<aikido::statespace::SE2>()),
        m_distance_metric(aikido::distance::SE2(m_statespace)) {}

    ~SE2();

    /// Documentation inherited
    int get_or_create_state(const StateSpace::State& _state) override;

    /// Documentation inherited
    int get_or_create_state(
        const aikido::statespace::StateSpace::State& _state) override;
    
    /// Documentation inherited    
    void discrete_state_to_continuous(
        const StateSpace::State& _state,
        aikido::statespace::StateSpace::State* _continuous_state) const override;

    /// Documentation inherited
    void continuous_state_to_discrete(
        const aikido::statespace::StateSpace::State& _state, 
        StateSpace::State* _discrete_state) const override;

    /// Documentation inherited
    bool get_state_id(
        const StateSpace::State& _state, int* _state_id) const override;

    /// Documentation inherited
    StateSpace::State* get_state(const int& _state_id) const override;

    /// Documentation inherited
    /// State is valid if theta is in [0, num_theta_vals]
    bool is_valid_state(const StateSpace::State& _state) const override;

    /// Documentation inherited
    int size() const override;

    /// Documentation inherited
    double get_distance(
        const StateSpace::State& _state_1,
        const StateSpace::State& _state_2) const override;
    
    /// Documentation inherited
    double get_distance(
        const aikido::statespace::StateSpace::State& _state_1,
        const aikido::statespace::StateSpace::State& _state_2) const override;
    
    /// Documentation inherited
    void copy_state(
        const StateSpace::State& _source, 
        StateSpace::State* _destination) const override;

 private:
    /// Creates a new state and adds it to the statespace
    ///
    /// \return pointer to the state
    StateSpace::State* create_state() override;

    /// Gets normalized angle (radians) in [0, 2pi]
    ///
    /// \param theta_rad Angle (radians)
    /// \return normalized angle
    double normalize_angle_rad(const double& theta_rad) const;

    /// Converts discrete angle to continuous (radians)
    ///
    /// \param theta Discrete angle in [0, num_theta_vals]
    /// \return Continuous angle in [0, 2pi]
    double discrete_angle_to_continuous(const int& theta) const;

    /// Converts discrete angle to continuous (radians)
    ///
    /// \param theta Continuous angle (radians)
    /// \return Discrete angle in [0, num_theta_vals]
    int continuous_angle_to_discrete(const double& theta) const;

    /// Converts discrete position to continuous (x_m, y_m)
    ///
    /// \param position Discrete coordinates
    /// \return Continous coordinates in meters
    Eigen::Vector2d discrete_position_to_continuous(
        const Eigen::Vector2i& position) const;

    /// Converts continuous position (x_m, y_m) to discrete
    ///
    /// \param position Continuous coordinates in meters
    /// \return Discrete coordinates
    Eigen::Vector2i continuous_position_to_discrete(
        const Eigen::Vector2d& position) const;

    /// Maps discrete state (libcozmo::statespace::SE2::State) to state ID
    std::unordered_map<State, int, boost::hash<State>> m_state_to_id_map;

    /// Vector of discrete states (libcozmo::statespace::SE2::State)
    /// Index of state is the state ID
    std::vector<State*> m_state_map;

    /// Number of discretized theta values
    const int m_num_theta_vals;

    /// Resolution of environment (m)
    const double m_resolution;

    std::shared_ptr<aikido::statespace::SE2> m_statespace;
    aikido::distance::SE2 m_distance_metric;
};

}  // namespace statespace
}  // namespace libcozmo

#endif  // LIBCOZMO_STATESPACE_SE2_HPP_

