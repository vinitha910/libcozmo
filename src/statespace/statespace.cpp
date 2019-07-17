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

#include "src/statespace/statespace.h"
#include <Eigen>
#include <assert.h>
#include <cmath>
#include <utility>
#include "aikido/distance/SE2.hpp"
#include "aikido/statespace/SE2.hpp"
#include "cozmo_description/cozmo.hpp"

namespace grid_planner {
namespace statespace {
using aikido::statespace::SE2;

int Statespace::set_start_state(const int& x, const int& y, const int& theta) {
    if (is_valid_state(x, y, theta)) {
        m_start_id = get_state_id(x, y, theta);
        return m_start_id;
    }
    return -1;
}

int Statespace::set_goal_state(const int& x, const int& y, const int& theta) {
    if (is_valid_state(x, y, theta)) {
        m_goal_id = get_state_id(x, y, theta);
        return m_goal_id;
    }
    return -1;
}

SE2::State Statespace::create_new_state(const double& x,
                                        const double& y,
                                        const double& theta) {
    SE2::State s;
    Eigen::Isometry2d t = Eigen::Isometry2d::Identity();
    const Eigen::Rotation2D<double> rot(theta);
    t.linear() = rot.toRotationMatrix();
    Eigen::Vector2d trans;
    trans << x, y;
    t.translation() = trans;
    s.setIsometry(t);
    Eigen::Vector xyth(3);
    xyth = continuous_pose_to_discrete(x, y, theta);
    int tmp_id = get_state_id(xyth[0], xyth[1], xyth[2]);
    m_state_map[tmp_id] = s;
    return s;
}

SE2::State Statespace::get_or_create_new_state(const int& x,
                                               const int& y,
                                               const int& theta) {
    int input_state_id = get_state_id(x, y, theta);
    if (m_state_map.find(tmp_id) != map.end()) {
        return m_state_map[tmp_id];
    } else {
        return create_new_state(x, y, theta);
    }
}

void Statespace::get_path_coordinates(
    const std::vector<int>& path_state_ids,
    std::vector<std::pair<int, int> > *path_coordinates) const {
    for (int i = 0; i < path_state_ids.size(); ++i) {
        int state_id = path_state_ids[i];
        int x, y, theta;
        if (get_coord_from_state_id(state_id, &x, &y, &theta)) {
            path_coordinates->push_back(std::make_pair(x, y, theta));
        }
    }
}

int Statespace::get_state_id(const int& x,
                             const int& y,
                             const int& theta) const {
    assert(x < m_width);
    assert(y < m_height);
    assert(theta <= m_bins);
    return (theta * m_width * m_height) + y * m_width + x;
}

bool Statespace::get_coord_from_state_id(const int& state_id,
                                         int* x,
                                         int* y,
                                         int* theta) const {
    assert(state_id < m_occupancy_grid.size());
    int theta_val = state_id / (m_width * m_height);
    state_id = (state_id - theta * m_width * m_height);
    int y_val = state_id / m_width;
    int x_val = state_id - y_val * m_width;
    *x = x_val;
    *y = y_val;
    *theta = theta_val;
    return (is_valid_state(*x, *y, *theta));
}

bool Statespace::is_valid_state(const int& x,
                                const int& y,
                                const int& theta) const {
    if (!(x >= 0 && x < m_width && y >= 0 && y < m_height)) {
        return false;
    }
    if (!(theta >= 0 && theta < m_bins)) {
        return false;
    }
    int state_id = get_state_id(x, y, theta);
    if (m_occupancy_grid[state_id] == 0) {
        return true;
    }
    return false;
}

double Statespace::get_distance(const SE2::State& source,
                                const SE2::State& succ) {
    return SE2::distance(source, succ);
}

double Statespace::normalize_angle_rad(const double& theta_rad) {
    assert(m_bins % 2 == 0);
    if !(theta_rad >= 0 && theta_rad <= 2 * math.pi) {
        theta_rad = theta_rad % (2 * math.pi);
    }
    return theta_rad;
}

double Statespace::discrete_angle_to_continuous(const int& theta) {
    double rad = 2 * math.pi / m_bins * theta;
    return rad;
}

int Statespace::continuous_angle_to_discrete(cosnt int& theta_rad) {
    int discrete = static_cast<int>(theta_rad / (2 * math.pi / m_bins));
    return discrete;
}

Eigen::Vector Statespace::continuous_position_to_discrete(const double& x_m,
                                                          const double& y_m) {
    int x = static_cast<int>(x_m / (m_width / m_resolution));
    int y = static_cast<int>(y_m / (m_height / m_resolution));
    Eigen::Vector xy(2);
    xy << x, y;
    return xy;
}

Eigen::Vector Statespace::discrete_position_to_continuous(const int& x,
                                                          const int& y) {
    double x_m = x * (m_width / m_resolution);
    double y_m = y * (m_height / m_resolution);
    Eigen::Vector xy(2);
    xy << x_m, y_m;
    return xy;
}

Eigen::vector Statespace::discrete_pose_to_continuous(const int& x,
                                                      const int& y,
                                                      const int& theta) {
    Eigen::Vector xy = discrete_position_to_continuous(x, y);
    Eigen::Vector th(1);
    th << discrete_angle_to_continuous(theta);
    Eigen::Vector ans(xy.size() + th.size());
    ans << xy, th;
    return ans;
}

Eigen::vector Statespace::continuous_pose_to_discrete(const double& x_m,
                                                      const double& y_m,
                                                      const double& theta_rad) {
    Eigen::Vector xy = continuous_position_to_discrete(x_m, y_m);
    Eigen::Vector th(1);
    th << continuous_angle_to_discrete(normalize_angle_rad(theta_rad));
    Eigen::Vector ans(xy.size() + th.size());
    ans << xy, th;
    return ans;
}

}  // namespace statespace
}  // namespace grid_planner

