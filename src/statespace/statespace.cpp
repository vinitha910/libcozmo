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

Eigen::Vector3i Statespace::create_new_state(const int& x, \
    const int& y, \
    const int& theta) {
    Eigen::Vector3i state(x, y, theta);
    m_state_map.push_back(state);
    m_state_to_id_map[state] = m_state_to_id_map.size() - 1;
    return state;
}

Eigen::Vector3i Statespace::create_new_state(\
    const aikido::statespace::SE2::State& state_continuous) {
    auto t = state_continuous.getIsometry();
    Eigen::Rotation2D<double> rotation;
    rotation.fromRotationMatrix(t.linear());
    const double theta_rad = rotation.angle();
    const Eigen::Vector2d position = t.translation();
    const int x = position[0];
    const int y = position[1];
    const int theta = continuous_angle_to_discrete(theta_rad);
    create_new_state(x, y, theta);
}

Eigen::Vector3i Statespace::get_or_create_new_state(const int& x, \
    const int& y,
    const int& theta) {
    const Eigen::Vector3i pose(x, y, theta);
    const auto state_id = m_state_to_id_map.find(pose);
    if (state_id != m_state_to_id_map.end()) {
        return m_state_map.at(state_id->second);
    } else {
        return create_new_state(x, y, theta);
    }
}

Eigen::Vector3i Statespace::get_or_create_new_state(\
    const aikido::statespace::SE2::State& state_continuous) {
    const auto t = state_continuous.getIsometry();
    Eigen::Rotation2D<double> rotation;
    rotation.fromRotationMatrix(t.linear());
    const double theta_rad = rotation.angle();
    const Eigen::Vector2d position = t.translation();
    const int x = position[0];
    const int y = position[1];
    const int theta = continuous_angle_to_discrete(theta_rad);
    return get_or_create_new_state(x, y, theta);
}

void Statespace::get_path_coordinates(\
    const std::vector<int>& path_state_ids, \
    std::vector<Eigen::Vector3i> *path_coordinates) {
    for (int i = 0; i < path_state_ids.size(); ++i) {
        const int state_id = path_state_ids[i];
        Eigen::Vector3i pose;
        if (get_coord_from_state_id(state_id, pose)) {
            path_coordinates->push_back(create_new_state(pose[0], \
                pose[1], pose[2]));
        }
    }
}

bool Statespace::get_state_id(const Eigen::Vector3i& pose, int& id) {
    const auto itr = m_state_to_id_map.find(pose);
    if (itr != m_state_to_id_map.end()) {
        id = itr->second;
        return true;
    }
    return false;
}

bool Statespace::get_coord_from_state_id(const int& state_id, \
    Eigen::Vector3i& state) const {
    state = m_state_map[state_id];
    return (is_valid_state(state));
}

bool Statespace::is_valid_state(const Eigen::Vector3i& state) const {
    const int theta = state.z();
    if (!(theta >= 0 && theta < m_num_theta_vals)) {
        return false;
    }
    return true;
}

double Statespace::normalize_angle_rad(const double& theta_rad) const {
    assert(m_bins % 2 == 0);
    double normalized_theta_rad = theta_rad;
    if (abs(theta_rad) > 2.0 * M_PI) {
        normalized_theta_rad = normalized_theta_rad - \
            static_cast<int>(normalized_theta_rad / (2.0 * M_PI)) * 2.0 * M_PI;
    }
    if (theta_rad < 0) {
        normalized_theta_rad += 2.0 * M_PI;
    }
    return normalized_theta_rad;
}

double Statespace::discrete_angle_to_continuous(const int& theta) const {
    const double theta_rad = theta * (2 * M_PI /m_num_theta_vals);
    return theta_rad;
}

int Statespace::continuous_angle_to_discrete(const double& theta_rad) const {
    const double bin_size = 2.0 * M_PI / static_cast<double>(m_num_theta_vals);
    const int theta = \
        normalize_angle_rad(theta_rad + bin_size / 2.0) / (2.0 * M_PI) \
        * static_cast<double>(m_num_theta_vals);
    return static_cast<int>(theta);
}

Eigen::Vector2i Statespace::continuous_position_to_discrete(\
    const Eigen::Vector2d position) const {
    const int x = static_cast<int>(floor(position.x() / \
        static_cast<double>(m_resolution)));
    const int y = static_cast<int>(floor(position.y() / \
        static_cast<double>(m_resolution)));
    const Eigen::Vector2i discrete_position(x, y);
    return discrete_position;
}

Eigen::Vector2d Statespace::discrete_position_to_continuous(\
    const Eigen::Vector2i position) const {
    const double x_m = position.x() * m_resolution + (m_resolution / 2.0);
    const double y_m = position.y() * m_resolution + (m_resolution / 2.0);
    const Eigen::Vector2d continuous_positon(x_m, y_m);
    return continuous_positon;
}

Eigen::Vector3d Statespace::discrete_pose_to_continuous(\
    const Eigen::Vector3i pose_discrete) const {
    const Eigen::Vector2i discrete_positon(pose_discrete.x(), \
        pose_discrete.y());
    const Eigen::Vector2d position = discrete_position_to_continuous(\
        discrete_positon);
    const double theta_discrete = discrete_angle_to_continuous(\
        pose_discrete.z());
    const Eigen::Vector3d pose(position[0], position[1], theta_discrete);
    return pose;
}

Eigen::Vector3i Statespace::continuous_pose_to_discrete(\
    const Eigen::Vector3d pose_continuous) const {
    const Eigen::Vector2d position_continuous(pose_continuous.x(), \
        pose_continuous.y());
    const Eigen::Vector2i position = continuous_position_to_discrete(\
        position_continuous);
    const int theta  = continuous_angle_to_discrete(normalize_angle_rad(\
        pose_continuous.z()));
    const Eigen::Vector3i pose(position[0], position[1], theta);
    return pose;
}

Eigen::Vector3i Statespace::continuous_pose_to_discrete(\
    const aikido::statespace::SE2::State& state_continuous) {
    const auto t = state_continuous.getIsometry();
    Eigen::Rotation2D<double> rotation;
    rotation.fromRotationMatrix(t.linear());
    const double theta_rad = rotation.angle();
    const Eigen::Vector2d position = t.translation();
    const int x = position[0];
    const int y = position[1];
    const int theta = continuous_angle_to_discrete(theta_rad);
    const Eigen::Vector3i pose(x, y, theta);
    return pose;
}

int Statespace::get_num_states() const {
    return m_state_map.size();
}

}  // namespace statespace
}  // namespace libcozmo

