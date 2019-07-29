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

#include "statespace/statespace.hpp"
#include <ros/ros.h>
#include <assert.h>
#include <cmath>

namespace libcozmo {
namespace statespace {

// int Statespace::set_start_state(const int& x, const int& y, const int& theta) {
//     try {
//         if (is_valid_state(x, y, theta)) {
//             m_start_id = get_state_id(x, y, theta);
//             return m_start_id;
//         }
//     } catch (ros::Exception &e) {
//         ROS_ERROR("Error Message %s ", e.what());
//     }
// }

// int Statespace::set_goal_state(const int& x, const int& y, const int& theta) {
//     try {
//         if (is_valid_state(x, y, theta)) {
//             m_goal_id = get_state_id(x, y, theta);
//             return m_goal_id;
//         }
//     } catch (ros::Exception &e) {
//         ROS_ERROR("Error Message %s ", e.what());
//     }
// }

// Eigen::Vector3i Statespace::create_new_state(const int& x,
//                                              const int& y,
//                                              const int& theta) {
//     Eigen::Vector3i state(x, y, theta);
//     int state_id = get_state_id(x, y, theta);
//     m_state_map[state_id] = state;
//     return state;
// }

// Eigen::Vector3i Statespace::create_new_state(
//     const aikido::statespace::SE2::State& state_continuous) {
//     auto t = state_continuous.getIsometry();
//     Eigen::Rotation2D<double> rotation;
//     rotation.fromRotationMatrix(t.linear());
//     const double theta_rad = rotation.angle();
//     const Eigen::Vector2d position = t.translation();
//     int x = position[0];
//     int y = position[1];
//     int theta = continuous_angle_to_discrete(theta_rad);
//     Eigen::Vector3i state_discrete(x, y, theta);
//     int state_id = get_state_id(x, y, theta);
//     m_state_map[state_id] = state_discrete;
//     return state_discrete;
// }

// Eigen::Vector3i Statespace::get_or_create_new_state(const int& x,
//                                                const int& y,
//                                                const int& theta) {
//     int state_id = get_state_id(x, y, theta);
//     if (m_state_map.find(state_id) != m_state_map.end()) {
//         return m_state_map[state_id];
//     } else {
//         return create_new_state(x, y, theta);
//     }
// }

// Eigen::Vector3i Statespace::get_or_create_new_state(
//     const aikido::statespace::SE2::State& state_continuous) {
//     auto t = state_continuous.getIsometry();
//     Eigen::Rotation2D<double> rotation;
//     rotation.fromRotationMatrix(t.linear());
//     const double theta_rad = rotation.angle();
//     const Eigen::Vector2d position = t.translation();
//     int x = position[0];
//     int y = position[1];
//     int theta = continuous_angle_to_discrete(theta_rad);
//     int state_id = get_state_id(x, y, theta);
//     if (m_state_map.find(state_id) != m_state_map.end()) {
//         return m_state_map[state_id];
//     } else {
//         return create_new_state(x, y, theta);
//     }
// }

// void Statespace::get_path_coordinates(
//     const std::vector<int>& path_state_ids,
//     std::vector<Eigen::Vector3i> *path_coordinates) {
//     for (int i = 0; i < path_state_ids.size(); ++i) {
//         int state_id = path_state_ids[i];
//         Eigen::Vector3i pose;
//         if (get_coord_from_state_id(state_id, pose)) {
//             path_coordinates->push_back(create_new_state(pose[0], \
//             pose[1], pose[2]));
//         }
//     }
// }

// int Statespace::get_state_id(const int& x,
//                              const int& y,
//                              const int& theta) const {
//     //return (theta * m_width * m_height) + y * m_width + x;
//     Eigen::Vector3i pose(x, y, theta);
//     return m_state_to_id_map[pose];
// }

// bool Statespace::get_coord_from_state_id(const int& state_id,
//                                          Eigen::Vector3i& state) const {
//     int theta_val = state_id / (m_width * m_height);
//     int y_val = (state_id - theta_val * m_width * m_height) / m_width;
//     int x_val = state_id - y_val * m_width - theta_val * m_width * m_height;
//     state << x_val, y_val, theta_val;
//     return (is_valid_state(x_val, y_val, theta_val));
// }

bool Statespace::is_valid_state(const int& x,
                                const int& y,
                                const int& theta) const {

    if (!(theta >= 0 && theta < m_num_theta_vals)) {
        return false;
    }
    return true;
}

// double Statespace::get_distance(
//     const aikido::statespace::SE2::State& state_1,
//     const aikido::statespace::SE2::State& state_2) const{
//     return m_distance.distance(&state_1, &state_2);
// }

// double Statespace::normalize_angle_rad(const double& theta_rad) const {
//     assert(m_bins % 2 == 0);
//     double normalized_theta_rad = theta_rad;
//     if (abs(theta_rad) > 2.0 * M_PI) {
//         normalized_theta_rad = normalized_theta_rad - \
//         static_cast<int>(normalized_theta_rad / (2.0 * M_PI)) * 2.0 * M_PI;
//     }
//     if (theta_rad < 0) {
//         normalized_theta_rad += 2.0 * M_PI;
//     }
//     return normalized_theta_rad;
// }

// double Statespace::discrete_angle_to_continuous(const int& theta) const {
//     double theta_rad = theta * (2 * M_PI /m_num_theta_vals);
//     return theta_rad;
// }

// int Statespace::continuous_angle_to_discrete(const double& theta_rad) const {
//     double bin_size = 2.0 * M_PI / m_num_theta_vals;
//     int theta = \
//         normalize_angle_rad(theta_rad + bin_size / 2.0) / (2.0 * M_PI) \
//         * m_num_theta_vals;
//     return static_cast<int>(theta);
// }

// Eigen::Vector2i Statespace::continuous_position_to_discrete(
//                                         const double& x_m,
//                                         const double& y_m) const {
//     int x = static_cast<int>(floor(x_m / m_resolution));
//     int y = static_cast<int>(floor(y_m / m_resolution));
//     Eigen::Vector2i position(x, y);
//     return position;
// }

// Eigen::Vector2d Statespace::discrete_position_to_continuous(const int& x,
//                                                           const int& y) const {
//     double x_m = x * m_resolution + (m_resolution / 2);
//     double y_m = y * m_resolution + (m_resolution / 2);
//     Eigen::Vector2d positon(x_m, y_m);
//     return positon;
// }

// Eigen::Vector3d Statespace::discrete_pose_to_continuous(const int& x,
//                                                       const int& y,
//                                                       const int& theta) const {
//     Eigen::Vector2d position = discrete_position_to_continuous(x, y);
//     double theta_discrete = discrete_angle_to_continuous(theta);
//     Eigen::Vector3d pose(position[0], position[1], theta_discrete);
//     return pose;
// }

// Eigen::Vector3i Statespace::continuous_pose_to_discrete(
//                                                 const double& x_m,
//                                                 const double& y_m,
//                                                 const double& theta_rad) const {
//     Eigen::Vector2i position = continuous_position_to_discrete(x_m, y_m);
//     int theta  = continuous_angle_to_discrete(normalize_angle_rad(theta_rad));
//     Eigen::Vector3i pose(position[0], position[1], theta);
//     return pose;
// }

// Eigen::Vector3i Statespace::continuous_pose_to_discrete(
//     const aikido::statespace::SE2::State& state_continuous) {
//     return create_new_state(state_continuous);
// }

// int Statespace::get_map_size() const {
//     return m_state_map.size();
// }

// void Statespace::get_state(const int& state_id, \
// std::unordered_map<int, Eigen::Vector3i>::const_iterator& itr) {
//     auto element = m_state_map.find(state_id);
//     itr =  element;
// }

}  // namespace statespace
}  // namespace libcozmo

