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
//#include "aikido/statespace/SE2.hpp"
//#include "cozmo_description/cozmo.hpp"

namespace libcozmo {
namespace statespace {

// This class implements a 2D grid-based graph representation
class Statespace {
 public:
    // Occupancy_grid A 1D vector containing either 1 for an occupied/obstacle
    // Cell or 0 for a free cell. The index is a 1D representation for an (x, y)
    // Width The width of the occupancy grid
    // Height The height of the occupancy grid
    Statespace(const int& width,
          const int& height,
          const double& res,
          const int& num_theta_vals) : \
          m_width(width),
          m_height(height),
          m_resolution(res),
          m_num_theta_vals(num_theta_vals),
          m_distance(std::make_shared<aikido::statespace::SE2>())
          {}

    ~Statespace() {}

    // Sets and returns the start state ID (m_start_id)
    // Returns -1 if the state is not valid
    int set_start_state(const int& x, const int& y, const int& theta);

    // Sets and returns the goal state ID (m_goal_id)
    // Returns -1 if the state is not valid
    int set_goal_state(const int& x, const int& y, const int& theta);

    // Creates new state with given width, height, and theta
    Eigen::Vector3i create_new_state(const int& x, const int& y, const int& theta);
    Eigen::Vector3i create_new_state(const aikido::statespace::SE2::State& s);

    // Creates new state with given width, height, and theta
    Eigen::Vector3i get_or_create_new_state(const int& x,
                                       const int& y,
                                       const int& theta);

    //overloading                                  
    Eigen::Vector3i get_or_create_new_state(const aikido::statespace::SE2::State& s);

    // Find the coordinates for the path given a vector containing all the
    // state IDs
    // now the path coordinates are (theta, (x,y)) wrapped as nested pair
    void get_path_coordinates(
        const std::vector<int>& path_state_ids,
        std::vector<Eigen::Vector3i> *path_coordinates);

 private:
    // Returns the state ID (1D representation) for the given (x, y) cell
    int get_state_id(const int& x,
                     const int& y,
                     const int& theta) const;

    // Gets the coordinates for the given state ID and stores then in x and y
    // Return true if coordinates are valid and false otherwise
    bool get_coord_from_state_id(const int& state_id, Eigen::Vector3i& state) const;

    // Return true if the state is valid and false otherwise
    bool is_valid_state(const int& x, const int& y, const int& theta) const;

    // Get normalized raadian angle in [0, 2pi]
    double normalize_angle_rad(const double& theta_rad) const;

    // Convert normalized radian into int
    // that corresponds to the bin
    double discrete_angle_to_continuous(const int& theta) const;

    // Converts discrete angle to minimum value of the corresponding bin
    int continuous_angle_to_discrete(const double& theta) const;

    // Converts meter value of input position in discretized value
    // Based on the grid width, height, and resolution
    Eigen::Vector2i continuous_position_to_discrete(const double& x_m,
                                                  const double& y_m) const;

    // Dicrete value of position of input state into continuous values
    Eigen::Vector2d discrete_position_to_continuous(const int& x,
                                                  const int& y) const;

    // Converts continuous pose(x,y,th) from relative discretized values
    // To real continuous values
    Eigen::Vector3d discrete_pose_to_continuous(const int& x,
                                              const int& y,
                                              const int& theta) const;

    // Converts continuous pose(x,y,th) from real continuous values
    // To relative discrete values
    Eigen::Vector3i continuous_pose_to_discrete(const double& x_m,
                                              const double& y_m,
                                              const double& theta_rad) const;
    
    //overloading
    Eigen::Vector3i continuous_pose_to_discrete(const aikido::statespace::SE2::State& state_continuous) ;

    // Computes distance between two states in SE2
    // Takes in two State objects
    double get_distance(const aikido::statespace::SE2::State& state_1,
                        const aikido::statespace::SE2::State& state_2) const;

    double get_distance(const Eigen::Vector3i& state_1,
                        const Eigen::Vector3i& state_2) const;

    std::unordered_map<int, Eigen::Vector3i> m_state_map;
    const int m_width;
    const int m_height;
    const int m_num_theta_vals;
    const double m_resolution;
    int m_start_id;
    int m_goal_id;
    const aikido::distance::SE2 m_distance;
};

}  // namespace statespace
}  // namespace libcozmo

#endif  // SRC_STATESPACE_STATESPACE_H_
