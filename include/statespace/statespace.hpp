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

// Hashing for m_state_to_id_map as Eigen does not come with hashing function
struct StateHasher
{
    // Returns hashkey
    
    // \param state the discretized state to function as key for mapping its id
    std::size_t operator()(const Eigen::Vector3i& state) const
    {
        using boost::hash_value;
        using boost::hash_combine;
        std::size_t seed = 0;
        hash_combine(seed,hash_value(state.x()));
        hash_combine(seed,hash_value(state.y()));
        hash_combine(seed,hash_value(state.z()));
        return seed;
    }
};

class Statespace {
 public:
    // Creates Statespace object

    // \param res resolution to grid the envrionment with
    // \param num_theta_vals number of bins for theta to be discretized into
    Statespace(const double& res,
               const int& num_theta_vals) : \
               m_resolution(res),
               m_num_theta_vals(num_theta_vals)
               {}

    ~Statespace() {}

    // Creates new state with given width, height, and orientation
    
    // \param x, y, theta discrete values for state
    Eigen::Vector3i create_new_state(const int& x, const int& y, \
        const int& theta);

    // Creates new state given SE2 state instead

    // \param state SE2 state in continuous value to discretize
    Eigen::Vector3i create_new_state(const aikido::statespace::SE2::State& s);

    // Checks if state with given pose already exists
    // If not, creates new state

    // \param x the discrete x coordinate
    // \param y the discrete y coordinate
    // \param theta discrete angle 
    Eigen::Vector3i get_or_create_new_state(const int& x,
                                            const int& y,
                                            const int& theta);

    // Checks if state with given pose based on SE2 already exists
    // If not, creates new state  

    // \param state SE2 state                          
    Eigen::Vector3i get_or_create_new_state(\
        const aikido::statespace::SE2::State& s);

    // Fills in vector with coordinates of states

    // \param path_state_ids list of ids holding path
    // \param path_coordinates vector to fill in respective coordinates
    void get_path_coordinates(
        const std::vector<int>& path_state_ids,
        std::vector<Eigen::Vector3i> *path_coordinates);

    // Converts continuous pose(x,y,th) from relative discretized values
    // To real continuous values

    // \param pose_discrete discrete state vector
    Eigen::Vector3d discrete_pose_to_continuous(\
        const Eigen::Vector3i pose_discrete) const;

    // Converts continuous pose(x,y,th) from real continuous values
    // To relative discrete values on graph representation

    // \param pose_continuous continuous state vector
    Eigen::Vector3i continuous_pose_to_discrete(\
        Eigen::Vector3d pose_continuous) const;
    
    // Converts SE2 state, which holds continuous values
    // to discrete state for Statespace use
    
    // \param state_continuous SE2 state
    Eigen::Vector3i continuous_pose_to_discrete(
        const aikido::statespace::SE2::State& state_continuous);
    
    // Returns the state ID (1D representation) for the given (x, y) cell
    
    // \param pose the discrete state
    // \id the container for id found from pose
    bool get_state_id(const Eigen::Vector3i& pose, int& id);

    // Gets the coordinates for the given state ID and stores then in x and y
    // Return true if coordinates are valid and false otherwise
    
    // \param state_id The id of the state
    // \param state the vector to fill in found coordinates x,y,theta
    bool get_coord_from_state_id(const int& state_id, \
        Eigen::Vector3i& state) const;

    // Return true if the state is valid and false otherwise
    
    // \param state the discretized state
    bool is_valid_state(const Eigen::Vector3i& state) const;

    // Returns size of the unordered map holding states
    // Discrete state keyed to repsective id
    int get_num_states() const;

 private:
     // Get normalized raadian angle in [0, 2pi]

     // \param theta_rad angle in radian with no particular bounds
    double normalize_angle_rad(const double& theta_rad) const;

    // Convert normalized radian into int
    // that corresponds to the bin

    // \theta normalized angle in radian
    double discrete_angle_to_continuous(const int& theta) const;

    // Converts discrete angle to minimum value of the corresponding bin

    // \param theta discretizd angle
    int continuous_angle_to_discrete(const double& theta) const;

    // Converts meter value of input position in discretized value
    // Based on the grid width, height, and resolution

    // \param position the x,y coordinate vector
    Eigen::Vector2i continuous_position_to_discrete(\
        const Eigen::Vector2d position) const;

    // Dicrete value of position of input state into continuous values

    // \param position the discretized x,y coordinate vector
    Eigen::Vector2d discrete_position_to_continuous(\
        const Eigen::Vector2i position) const;

    // map holding respective index in container vector keyed by its state
    std::unordered_map<Eigen::Vector3i, int, StateHasher> m_state_to_id_map;

    // container vector of states created
    std::vector<Eigen::Vector3i> m_state_map;

    // number of bins to discretized theta into
    const int m_num_theta_vals;

    // unit to measure x,y in discretized envrionment
    const double m_resolution;
};

}  // namespace statespace
}  // namespace libcozmo

#endif  // INCLUDE_STATESPACE_STATESPACE_H_

