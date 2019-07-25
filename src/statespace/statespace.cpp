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
//#include <Eigen/Dense>
#include <assert.h>
#include <cmath>
#include<ros/ros.h>
//#include "aikido/distance/SE2.hpp"
//#include "aikido/statespace/SE2.hpp"
//#include "cozmo_description/cozmo.hpp"

namespace libcozmo {
namespace statespace {

int Statespace::set_start_state(const int& x, const int& y, const int& theta) {
    
    try {
        if (is_valid_state(x, y, theta)) {
            m_start_id = get_state_id(x, y, theta);
            return m_start_id;
        }
    } catch (ros::Exception &e) {
        ROS_ERROR("Error Message %s ", e.what());
    }

}

int Statespace::set_goal_state(const int& x, const int& y, const int& theta) {
    
    try {
        if (is_valid_state(x, y, theta)) {
            m_goal_id = get_state_id(x, y, theta);
            return m_goal_id;
        }
    } catch (ros::Exception &e) {
        ROS_ERROR("Error Message %s ", e.what());
    }
}

Eigen::Vector3i Statespace::create_new_state(const int& x, const int& y, const int& theta) {
    Eigen::Vector3i state(x, y, theta);
    int state_id = get_state_id(x, y, theta);
    m_state_map[state_id] = state;

    return state;
}

Eigen::Vector3i Statespace::create_new_state(const aikido::statespace::SE2::State& state_continuous) {
    
    auto t = state_continuous.getIsometry(); //Eigen::Transform
    
    Eigen::Rotation2D<double> rotation;
    rotation.fromRotationMatrix(t.linear()); 
    const double theta_rad = rotation.angle();
    const Eigen::Vector2d position = t.translation();
    int x = position.x();
    int y = position.y();

    //int theta = 

    Eigen::Vector3i state_discrete(x, y, theta_rad);
    int state_id = get_state_id(x, y, theta_rad);
    m_state_map[state_id] = state_discrete;

    return state_discrete;
}

// SE2::State Statespace::create_new_state(const double& x,
//                                         const double& y,
//                                         const double& theta) {
//     SE2::State s;
//     Eigen::Isometry2d t = Eigen::Isometry2d::Identity();
//     const Eigen::Rotation2D<double> rot(theta);
//     t.linear() = rot.toRotationMatrix();
//     Eigen::Vector2d trans;
//     trans << x, y;
//     t.translation() = trans;
//     s.setIsometry(t);
//     Eigen::Vector3d discrete_pose;
//     discrete_pose = continuous_pose_to_discrete(x, y, theta);
//     int state_id = get_state_id(discrete_pose.x(), discrete_pose.y(), discrete_pose.w());
//     m_state_map[state_id] = s;
//     return s;
// }

Eigen::Vector3i Statespace::get_or_create_new_state(const int& x,
                                               const int& y,
                                               const int& theta) {
    int state_id = get_state_id(x, y, theta);
    if (m_state_map.find(state_id) != m_state_map.end()) {
        return m_state_map[state_id];
    } else {
        return create_new_state(x, y, theta);
    }
}

Eigen::Vector3i Statespace::get_or_create_new_state(const aikido::statespace::SE2::State& state_continuous) {
    

    // if (m_state_map.find(state_id) != m_state_map.end()) {
    //     return m_state_map[state_id];
    // } else {
    //     return create_new_state(x, y, theta);
    // }
}

// void Statespace::get_path_coordinates(
//     const std::vector<int>& path_state_ids,
//     std::vector<SE2::State> *path_coordinates) const {
//     for (int i = 0; i < path_state_ids.size(); ++i) {
//         int state_id = path_state_ids[i];
//         int x, y, theta;
//         if (get_coord_from_state_id(state_id, &x, &y, &theta)) {
//             path_coordinates->pushback(create_new_state(x,y, theta));
//         }
//     }
// }

int Statespace::get_state_id(const int& x,
                             const int& y,
                             const int& theta) const {
    assert(x < m_width);
    assert(y < m_height);
    assert(theta <= m_num_theta_vals);
    return (theta * m_width * m_height) + y * m_width + x;
}

// bool Statespace::get_coord_from_state_id(const int& state_id,
//                                          int* x,
//                                          int* y,
//                                          int* theta) const {
//     assert(state_id < m_occupancy_grid.size());
//     int theta_val = state_id / (m_width * m_height);
//     state_id = (state_id - theta * m_width * m_height);
//     int y_val = state_id / m_width;
//     int x_val = state_id - y_val * m_width;
//     *x = x_val;
//     *y = y_val;
//     *theta = theta_val;
//     return (is_valid_state(*x, *y, *theta));
// }

bool Statespace::is_valid_state(const int& x,
                                const int& y,
                                const int& theta) const {
    if (!(x >= 0 && x < m_width && y >= 0 && y < m_height)) {
        return false;
    }
    if (!(theta >= 0 && theta < m_num_theta_vals)) {
        return false;
    }
    return true;
}

// double Statespace::get_distance(const SE2::State& source,
//                                 const SE2::State& succ) {
//     return SE2::distance(source, succ);
// }

double Statespace::normalize_angle_rad(const double& theta_rad) const {
    assert(m_bins % 2 == 0);
    double normalized_theta_rad = theta_rad;
    if (abs(normalized_theta_rad) > 2.0 * M_PI) {
        normalized_theta_rad = normalized_theta_rad - static_cast<int>((normalized_theta_rad / (2.0 * M_PI)) * 2.0 * M_PI);
    }
    if (theta_rad < 0) {
        normalized_theta_rad += 2.0 * M_PI;
    }
    return normalized_theta_rad;
}

// double Statespace::discrete_angle_to_continuous(const int& theta) {
//     double rad = 2 * M_PI / m_bins * theta;
//     return theta * (2 * M_PI /m_num_theta_vals);
// }

// int Statespace::continuous_angle_to_discrete(const int& theta_rad) {
//     double bin_size = 2.0 * M_PI / m_bum_theta_vals;
//     normalized_theta_rad = \
//         normalize_angle_rad(theta_rad + bin_size / 2.0) / (2.0 * M_PI) \
//         * m_num_theta_vals;
//     return int(normalized_theta_rad);
// }

// Eigen::Vector2d Statespace::continuous_position_to_discrete(const double& x_m,
//                                                           const double& y_m) {
//     int x = static_cast<int>(x_m / (m_width / m_resolution));
//     int y = static_cast<int>(y_m / (m_height / m_resolution));
//     Eigen::Vector2d position(x, y);
//     return position;
// }

// Eigen::Vector2d Statespace::discrete_position_to_continuous(const int& x,
//                                                           const int& y) {
//     double x_m = x * (m_width / m_resolution);
//     double y_m = y * (m_height / m_resolution);
//     Eigen::Vector2d positon(x_m, y_m);
//     return positon;
// }

// Eigen::vector3d Statespace::discrete_pose_to_continuous(const int& x,
//                                                       const int& y,
//                                                       const int& theta) {
//     Eigen::Vector2d position = discrete_position_to_continuous(x, y);
//     double theta = discrete_angle_to_continuous(theta);
//     Eigen::Vector3d pose(position.x(), position.y(), theta);
//     return pose;
// }

// Eigen::vector3d Statespace::continuous_pose_to_discrete(const double& x_m,
//                                                       const double& y_m,
//                                                       const double& theta_rad) {
//     Eigen::Vector2d position = continuous_position_to_discrete(x_m, y_m);
//     double theta  = continuous_angle_to_discrete(normalize_angle_rad(theta_rad));
//     Eigen::Vector3d pose(position.x(), position.y(), theta);
//     return pose;
// }

}  // namespace statespace
}  // namespace libcozmo

