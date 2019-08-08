////////////////////////////////////////////////////////////////////////////////
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
#include "statehasher.hpp"

namespace libcozmo {
namespace statespace {

// This class implements a discretized two-dimensional Special Euclidean
// group SE(2),i.e. the space of planar rigid body transformations.
class Statespace {
 public:
    // Constructor

    // \param resolution_m Resolution of discretized state (m)
    // \param num_theta_vals Number of discretized theta values,
    // must be a power of 2
    Statespace(
        const double& resolution_m,
        const int& num_theta_vals) : \
        m_resolution(resolution_m),
        m_num_theta_vals(num_theta_vals),
        m_statespace(std::make_shared<aikido::statespace::SE2>()),
        m_distance_metric(aikido::distance::SE2(m_statespace)) {}

    ~Statespace() {}

    // Creates a new state given discretized coordinates
    // Returns the ID of the created state

    // \param state The discrete state
    int create_new_state(const Eigen::Vector3i& state);

    // Creates a new discrete state given a SE2 transformation
    // Returns the ID of the created state

    // \param state The SE2 state
    int create_new_state(
        const aikido::statespace::SE2::State& state);

    // Returns ID of the state with given pose if it exists,
    // Otherwise creates a new state and returns its ID

    // \param pose The discrete state
    int get_or_create_new_state(const Eigen::Vector3i& pose);

    // Returns ID of the state with given transformation if it exists,
    // Otherwise creates a new state and returns its ID

    // \param state The SE2 state
    int get_or_create_new_state(
        const aikido::statespace::SE2::State& state);

    // Fills the states vector with states corresponding to given set of IDs

    // \param state_ids The vector of state ids
    // \param states The vector of discretized poses
    void get_path_states(
        const std::vector<int>& state_ids,
        std::vector<Eigen::Vector3i>* states);

    // Converts discretized state to continuous state

    // \param state The discrete state
    aikido::statespace::SE2::State discrete_state_to_continuous(
        const Eigen::Vector3i state) const;

    // Converts continuous state to discretized state

    // \param state The SE2 state
    Eigen::Vector3i continuous_state_to_discrete(
        const aikido::statespace::SE2::State& state);

    // Returns true and fills state_id with corresponding ID if state exists 
    // and false otherwise

    // \param pose The discrete state
    // \param id The state id
    bool get_state_id(const Eigen::Vector3i& state, int* state_id);

    // Returns true and fills state with corresponding state if state with 
    // given ID exists and false otherwise

    // \param state_id The ID of the state
    // \param state The discrete state
    bool get_coord_from_state_id(
        const int& state_id,
        Eigen::Vector3i* state) const;

    // Return true if the state is valid and false otherwise

    // \param state The discretized state
    bool is_valid_state(const Eigen::Vector3i& state) const;

    // Returns the number of currently existing states
    int get_num_states() const;

    // Returns distance between two SE2 states

    // \param state_1, state_2 The two states to
    // calculate the distance between
    double get_distance(
        const aikido::statespace::SE2::State* state_1,
        const aikido::statespace::SE2::State* state_2) const;

 private:
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
    std::unordered_map<Eigen::Vector3i, int, StateHasher> m_state_to_id_map;

    // Vector of discrete states; index of state is the state ID.
    std::vector<Eigen::Vector3i> m_state_map;

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

