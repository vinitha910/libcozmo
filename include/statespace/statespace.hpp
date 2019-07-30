////////////////////////////////////////////////////////////////////////////////
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

#ifndef INCLUDE_STATESPACE_STATESPACE_H_
#define INCLUDE_STATESPACE_STATESPACE_H_

#include <Eigen/Dense>
#include <vector>
#include <utility>
#include <unordered_map>
#include "aikido/distance/SE2.hpp"
#include <boost/functional/hash.hpp>
namespace libcozmo {
namespace statespace {

// Custom Hash function for a discretized state
struct StateHasher {
    std::size_t operator()(const Eigen::Vector3i& state) const {
        using boost::hash_value;
        using boost::hash_combine;
        std::size_t seed = 0;
        hash_combine(seed,hash_value(state.x()));
        hash_combine(seed,hash_value(state.y()));
        hash_combine(seed,hash_value(state.z()));
        return seed;
    }
};

// This class implements the two-dimensional special Euclidean
// group SE(2),i.e. the space of planar rigid body transformations.
class Statespace {
 public:
    // Creates Statespace object

    // \param res Resolution of discretized state
    // \param num_theta_vals Number of discretized theta values
    Statespace(const double& res,
               const int& num_theta_vals) : \
               m_resolution(res),
               m_num_theta_vals(num_theta_vals)
               {}

    ~Statespace() {}

    // Creates a new state given discretized coordinates
    
    // \param state The discrete state
    Eigen::Vector3i create_new_state(const Eigen::Vector3i& state);

    // Creates a new state given a SE2 transformation

    // \param state The SE2 state in continuous value to discretize
    Eigen::Vector3i create_new_state(const aikido::statespace::SE2::State& state);

    // Returns a state with given pose if it exists,
    // Otherwise creates and returns a new state

    // \param pose The discrete state 
    Eigen::Vector3i get_or_create_new_state(const Eigen::Vector3i& pose);

    // Returns a state with given transformation if it exists,
    // Otherwise creates and returns a new state  

    // \param state The SE2 state                          
    Eigen::Vector3i get_or_create_new_state(
        const aikido::statespace::SE2::State& state);

    // Return coordinates for given set of states

    // \param path_state_ids The vector of state ids
    // \param path_coordinates The vector of discretized poses
    void get_path_coordinates(
        const std::vector<int>& path_state_ids,
        std::vector<Eigen::Vector3i> *path_coordinates);

    // Converts discretized state to continuous state

    // \param pose_discrete The discrete state
    aikido::statespace::SE2::State discrete_pose_to_continuous(
        const Eigen::Vector3i pose_discrete) const;

    // Converts continuous state to discretized state
    
    // \param state_continuous The SE2 state
    Eigen::Vector3i continuous_pose_to_discrete(
        const aikido::statespace::SE2::State& state_continuous);
    
    // Returns state ID of given pose
    
    // \param pose The discrete state
    // \param id The state id
    bool get_state_id(const Eigen::Vector3i& pose, int& id);

    // Gets discretized state given state ID
    // Return true if state with given state ID exists and false otherwise
    
    // \param state_id The ID of the state
    // \param state The discrete state
    bool get_coord_from_state_id(
        const int& state_id,
        Eigen::Vector3i& state) const;

    // Return true if the state is valid and false otherwise
    
    // \param state The discretized state
    bool is_valid_state(const Eigen::Vector3i& state) const;

    // Returns the number of currently existng states
    // Discrete state keyed to repsective id
    int get_num_states() const;

 private:
     // Get normalized angle (radians) in [0, 2pi]

     // \param theta_rad Angle (radians)
    double normalize_angle_rad(const double& theta_rad) const;

    // Converts continuous angle (radians) to discrete

    // \theta Normalized angle (radians)
    double discrete_angle_to_continuous(const int& theta) const;

    // Converts discrete angle to continuous (radians)

    // \param theta Discretizd angle
    int continuous_angle_to_discrete(const double& theta) const;

    // Converts discrete position to continuous

    // \param position The discrete coordinates
    Eigen::Vector2d discrete_position_to_continuous(
        const Eigen::Vector2i position) const;

    // Maps discrete state to state ID
    std::unordered_map<Eigen::Vector3i, int, StateHasher> m_state_to_id_map;

    // Vector of discrete states; index of state is the state ID.
    std::vector<Eigen::Vector3i> m_state_map;

    // Number of discretized theta values
    const int m_num_theta_vals;

    // Resolution of discrete state
    const double m_resolution;
};

}  // namespace statespace
}  // namespace libcozmo

#endif  // INCLUDE_STATESPACE_STATESPACE_H_

